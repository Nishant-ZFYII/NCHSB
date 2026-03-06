#include "corridor_social_nav/safety_shield_cbf.hpp"
#include <algorithm>
#include <cmath>

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

  // Build CBF constraints: a_i * v >= rhs_i
  // These decompose into lower/upper bounds on v depending on sign of a_i.
  double v_lb = params_.v_min;
  double v_ub = params_.v_max;

  for (const auto & person : people) {
    double h = computeBarrier(robot, person);
    result.min_barrier_value = std::min(result.min_barrier_value, h);

    auto constraint = buildConstraint(robot, person);

    if (std::abs(constraint.a) < 1e-12) {
      // a_i ≈ 0: constraint is rhs <= 0 (always sat) or rhs > 0 (infeasible for any v)
      if (constraint.rhs > 0.0) {
        v_lb = params_.v_max + 1.0;  // force infeasibility
      }
      continue;
    }

    double v_bound = constraint.rhs / constraint.a;

    if (constraint.a > 0.0) {
      // a*v >= rhs  =>  v >= rhs/a  (lower bound)
      v_lb = std::max(v_lb, v_bound);
    } else {
      // a*v >= rhs with a<0  =>  v <= rhs/a  (upper bound, since dividing by negative flips)
      v_ub = std::min(v_ub, v_bound);
    }
  }

  if (v_lb > v_ub) {
    // Infeasible: pedestrian already inside effective safety zone. Emergency stop.
    result.v = 0.0;
    result.omega = 0.0;
    result.intervened = true;
    result.num_active_constraints = static_cast<int>(people.size());
    return result;
  }

  // Project v_nom onto [v_lb, v_ub] — minimal-intervention QP solution
  result.v = std::clamp(v_nom, v_lb, v_ub);

  if (std::abs(result.v - v_nom) > params_.intervention_threshold) {
    result.intervened = true;
  }

  // Count active constraints (those that actually limited v)
  for (const auto & person : people) {
    auto constraint = buildConstraint(robot, person);
    double margin = constraint.a * result.v - constraint.rhs;
    if (margin < 1e-6) {
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
