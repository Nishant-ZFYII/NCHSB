#include "corridor_social_nav/safety_shield_cbf.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include "osqp.h"

namespace corridor_social_nav
{

SafetyShieldCBF::SafetyShieldCBF(const CBFParams & params)
: params_(params)
{
}

double SafetyShieldCBF::stoppingDistance(double v) const
{
  double abs_v = std::abs(v);
  return params_.stopping_a * abs_v * abs_v +
         params_.stopping_b * abs_v +
         params_.stopping_c;
}

double SafetyShieldCBF::effectiveSafeDistance(double v) const
{
  return params_.dSafe() + stoppingDistance(v);
}

double SafetyShieldCBF::computeBarrier(
  const RobotState & robot,
  const PersonState & person) const
{
  double dx = robot.x - person.x;
  double dy = robot.y - person.y;
  double D_sq = dx * dx + dy * dy;
  double d_eff = effectiveSafeDistance(robot.v);
  return D_sq - d_eff * d_eff;
}

SafetyShieldCBF::LinearConstraint SafetyShieldCBF::buildConstraint(
  const RobotState & robot,
  const PersonState & person) const
{
  double dx = robot.x - person.x;
  double dy = robot.y - person.y;

  double h = computeBarrier(robot, person);

  // dh/dt = 2*dx*(v*cos(theta) - vx) + 2*dy*(v*sin(theta) - vy)
  //       = [2*dx*cos(theta) + 2*dy*sin(theta)] * v + [-2*dx*vx - 2*dy*vy]
  double cos_th = std::cos(robot.theta);
  double sin_th = std::sin(robot.theta);

  double a = 2.0 * dx * cos_th + 2.0 * dy * sin_th;
  double b = -2.0 * dx * person.vx - 2.0 * dy * person.vy;

  // CBF condition: dh/dt + alpha*h >= 0  =>  a*v + b + alpha*h >= 0  =>  a*v >= -(b + alpha*h)
  double rhs = -(b + params_.alpha * h);

  return {a, rhs};
}

SafeCommand SafetyShieldCBF::solve(
  const RobotState & robot,
  const std::vector<PersonState> & people,
  double v_nom,
  double omega_nom) const
{
  SafeCommand result;
  result.omega = std::clamp(omega_nom, -params_.omega_max, params_.omega_max);
  result.min_barrier_value = std::numeric_limits<double>::max();
  result.num_active_constraints = 0;
  result.intervened = false;

  if (people.empty()) {
    result.v = std::clamp(v_nom, params_.v_min, params_.v_max);
    return result;
  }

  // Build CBF constraints and min barrier
  std::vector<LinearConstraint> constraints;
  constraints.reserve(people.size());
  for (const auto & person : people) {
    double h = computeBarrier(robot, person);
    result.min_barrier_value = std::min(result.min_barrier_value, h);
    constraints.push_back(buildConstraint(robot, person));
  }

  // Check for trivially infeasible (a≈0 and rhs>0)
  for (const auto & c : constraints) {
    if (std::abs(c.a) < 1e-12 && c.rhs > 0.0) {
      result.v = 0.0;
      result.omega = 0.0;
      result.intervened = true;
      result.num_active_constraints = static_cast<int>(people.size());
      return result;
    }
  }

  // OSQP: min (1/2) x' P x + q' x  s.t.  l <= Ax <= u
  // x = [v, omega], P = I, q = [-v_nom, -omega_nom]
  const c_int n = 2;
  const c_int m = static_cast<c_int>(people.size()) + 2;
  const c_int nnz_A = static_cast<c_int>(people.size()) + 2;
  const c_int nnz_P = 2;

  std::vector<c_float> P_x = {1.0, 1.0};
  std::vector<c_int> P_i = {0, 1};
  std::vector<c_int> P_p = {0, 1, 2};

  std::vector<c_float> q = {-static_cast<c_float>(v_nom), -static_cast<c_float>(omega_nom)};

  std::vector<c_float> A_x(nnz_A);
  std::vector<c_int> A_i(nnz_A);
  std::vector<c_int> A_p = {0, static_cast<c_int>(people.size()) + 1, nnz_A};

  for (size_t k = 0; k < people.size(); ++k) {
    A_x[k] = static_cast<c_float>(constraints[k].a);
    A_i[k] = static_cast<c_int>(k);
  }
  A_x[people.size()] = 1.0;
  A_i[people.size()] = static_cast<c_int>(people.size());
  A_x[people.size() + 1] = 1.0;
  A_i[people.size() + 1] = static_cast<c_int>(people.size() + 1);

  std::vector<c_float> l(m);
  std::vector<c_float> u(m);
  for (size_t k = 0; k < people.size(); ++k) {
    l[k] = static_cast<c_float>(constraints[k].rhs);
    u[k] = OSQP_INFTY;
  }
  l[people.size()] = static_cast<c_float>(params_.v_min);
  u[people.size()] = static_cast<c_float>(params_.v_max);
  l[people.size() + 1] = -static_cast<c_float>(params_.omega_max);
  u[people.size() + 1] = static_cast<c_float>(params_.omega_max);

  csc P_csc = {nnz_P, n, n, P_p.data(), P_i.data(), P_x.data(), -1};
  csc A_csc = {nnz_A, m, n, A_p.data(), A_i.data(), A_x.data(), -1};

  OSQPData data;
  data.n = n;
  data.m = m;
  data.P = &P_csc;
  data.q = q.data();
  data.A = &A_csc;
  data.l = l.data();
  data.u = u.data();

  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;

  OSQPWorkspace * work = nullptr;
  c_int exitflag = osqp_setup(&work, &data, &settings);

  if (exitflag != 0 || work == nullptr) {
    osqp_cleanup(work);
    return solveAnalytical(robot, people, v_nom, omega_nom, result);
  }

  exitflag = osqp_solve(work);

  if (exitflag != 0 || work->info->status_val == OSQP_PRIMAL_INFEASIBLE ||
      work->info->status_val == OSQP_DUAL_INFEASIBLE) {
    osqp_cleanup(work);
    result.v = 0.0;
    result.omega = 0.0;
    result.intervened = true;
    result.num_active_constraints = static_cast<int>(people.size());
    return result;
  }

  result.v = static_cast<double>(work->solution->x[0]);
  result.omega = static_cast<double>(work->solution->x[1]);
  osqp_cleanup(work);

  if (std::abs(result.v - v_nom) > params_.intervention_threshold) {
    result.intervened = true;
  }

  for (const auto & c : constraints) {
    double margin = c.a * result.v - c.rhs;
    if (margin < 1e-6) {
      result.num_active_constraints++;
    }
  }

  return result;
}

SafeCommand SafetyShieldCBF::solveAnalytical(
  const RobotState & robot,
  const std::vector<PersonState> & people,
  double v_nom,
  double omega_nom,
  SafeCommand & result) const
{
  double v_lb = params_.v_min;
  double v_ub = params_.v_max;

  for (const auto & person : people) {
    auto constraint = buildConstraint(robot, person);
    if (std::abs(constraint.a) < 1e-12) {
      if (constraint.rhs > 0.0) {
        v_lb = params_.v_max + 1.0;
      }
      continue;
    }
    double v_bound = constraint.rhs / constraint.a;
    if (constraint.a > 0.0) {
      v_lb = std::max(v_lb, v_bound);
    } else {
      v_ub = std::min(v_ub, v_bound);
    }
  }

  if (v_lb > v_ub) {
    result.v = 0.0;
    result.omega = 0.0;
    result.intervened = true;
    result.num_active_constraints = static_cast<int>(people.size());
    return result;
  }

  result.v = std::clamp(v_nom, v_lb, v_ub);
  result.omega = std::clamp(omega_nom, -params_.omega_max, params_.omega_max);
  if (std::abs(result.v - v_nom) > params_.intervention_threshold) {
    result.intervened = true;
  }
  for (const auto & person : people) {
    auto constraint = buildConstraint(robot, person);
    if (constraint.a * result.v - constraint.rhs < 1e-6) {
      result.num_active_constraints++;
    }
  }
  return result;
}

CBFDiagnostics SafetyShieldCBF::diagnose(
  const RobotState & robot,
  const std::vector<PersonState> & people,
  double v_nom) const
{
  CBFDiagnostics diag;
  diag.v_lower_bound = params_.v_min;
  diag.v_upper_bound = params_.v_max;
  diag.feasible = true;

  for (const auto & person : people) {
    double h = computeBarrier(robot, person);
    diag.barrier_values.push_back(h);

    auto constraint = buildConstraint(robot, person);

    double margin = constraint.a * v_nom - constraint.rhs;
    diag.constraint_margins.push_back(margin);

    if (std::abs(constraint.a) < 1e-12) {
      if (constraint.rhs > 0.0) {
        diag.feasible = false;
      }
      continue;
    }

    double v_bound = constraint.rhs / constraint.a;
    if (constraint.a > 0.0) {
      diag.v_lower_bound = std::max(diag.v_lower_bound, v_bound);
    } else {
      diag.v_upper_bound = std::min(diag.v_upper_bound, v_bound);
    }
  }

  if (diag.v_lower_bound > diag.v_upper_bound) {
    diag.feasible = false;
  }

  return diag;
}

}  // namespace corridor_social_nav
