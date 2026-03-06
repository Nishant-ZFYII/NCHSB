#ifndef CORRIDOR_SOCIAL_NAV__SAFETY_SHIELD_CBF_HPP_
#define CORRIDOR_SOCIAL_NAV__SAFETY_SHIELD_CBF_HPP_

#include <vector>
#include <cmath>
#include <limits>

namespace corridor_social_nav
{

struct RobotState
{
  double x, y, theta;
  double v;
};

struct PersonState
{
  double x, y;
  double vx, vy;
};

struct SafeCommand
{
  double v;
  double omega;
  bool intervened;
  double min_barrier_value;
  int num_active_constraints;
};

struct CBFDiagnostics
{
  std::vector<double> barrier_values;
  std::vector<double> constraint_margins;
  double v_lower_bound;
  double v_upper_bound;
  bool feasible;
};

struct CBFParams
{
  double robot_radius = 0.20;
  double person_radius = 0.25;
  double safety_margin = 0.30;
  double alpha = 1.0;
  double v_max = 1.0;
  double v_min = 0.0;
  double omega_max = 1.5;
  double stopping_a = 0.35;
  double stopping_b = 0.05;
  double stopping_c = 0.01;
  double intervention_threshold = 0.05;

  double dSafe() const { return robot_radius + person_radius + safety_margin; }
};

/**
 * Ackermann-aware CBF/QP safety shield.
 *
 * Barrier function per pedestrian i:
 *   h_i = D_i^2 - d_eff^2
 * where D_i = ||p_robot - p_ped_i|| and d_eff = d_safe + d_stop(|v|).
 *
 * d_stop(v) = a*v^2 + b*|v| + c   (braking distance, calibrated to Ackermann)
 * d_safe = robot_radius + person_radius + safety_margin
 *
 * The braking-distance term makes the effective safety radius grow with speed:
 * faster => bigger safety zone => earlier intervention.
 *
 * CBF constraint (ensures forward invariance of safe set):
 *   dh_i/dt + alpha * h_i >= 0
 *
 * Expanding:
 *   a_i * v_cmd >= -c_i
 * where:
 *   a_i = 2*dx_i*cos(theta) + 2*dy_i*sin(theta)
 *   c_i = -2*dx_i*vx_i - 2*dy_i*vy_i + alpha*h_i
 *   dx_i = x_robot - x_ped_i,  dy_i = y_robot - y_ped_i
 *
 * QP formulation:
 *   min  0.5*(v - v_nom)^2 + 0.5*w*(omega - omega_nom)^2
 *   s.t. a_i * v >= -c_i   for each pedestrian i
 *        v_min <= v <= v_max
 *        -omega_max <= omega <= omega_max
 *
 * Since CBF constraints only involve v (not omega), the QP decouples:
 *   omega_safe = clamp(omega_nom, [-omega_max, omega_max])
 *   v_safe = project v_nom onto the feasible interval [v_lb, v_ub]
 *
 * This is the standard analytical solution for 1D CBF-QPs (Ames et al. 2017).
 */
class SafetyShieldCBF
{
public:
  explicit SafetyShieldCBF(const CBFParams & params);

  SafeCommand solve(
    const RobotState & robot,
    const std::vector<PersonState> & people,
    double v_nom,
    double omega_nom) const;

  CBFDiagnostics diagnose(
    const RobotState & robot,
    const std::vector<PersonState> & people,
    double v_nom) const;

  double stoppingDistance(double v) const;

  double computeBarrier(
    const RobotState & robot,
    const PersonState & person) const;

  double effectiveSafeDistance(double v) const;

  const CBFParams & params() const { return params_; }

private:
  struct LinearConstraint
  {
    double a;     // coefficient of v
    double rhs;   // lower bound: a*v >= rhs
  };

  LinearConstraint buildConstraint(
    const RobotState & robot,
    const PersonState & person) const;

  CBFParams params_;
};

}  // namespace corridor_social_nav

#endif  // CORRIDOR_SOCIAL_NAV__SAFETY_SHIELD_CBF_HPP_
