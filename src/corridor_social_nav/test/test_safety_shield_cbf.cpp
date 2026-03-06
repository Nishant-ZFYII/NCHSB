#include <gtest/gtest.h>
#include "corridor_social_nav/safety_shield_cbf.hpp"
#include <cmath>
#include <vector>

using namespace corridor_social_nav;

/*
 * Test design rationale:
 *
 * The CBF safety shield has several critical properties that must hold:
 *
 * 1. PASSIVITY: When no pedestrians are present, or all are far away,
 *    the shield must pass through the nominal command unchanged.
 *
 * 2. MONOTONIC INTERVENTION: As a pedestrian gets closer, the shield
 *    should progressively reduce speed. There should be no discontinuous
 *    jumps except at the infeasibility boundary.
 *
 * 3. EMERGENCY STOP: When a pedestrian is inside the effective safety zone
 *    (barrier h < 0), the shield must command v=0.
 *
 * 4. VELOCITY-DEPENDENT SAFETY: At higher speeds, the effective safety
 *    radius (d_safe + d_stop(v)) is larger, causing earlier intervention.
 *    This is the Ackermann-aware braking-distance contribution.
 *
 * 5. DIRECTIONAL AWARENESS: A pedestrian behind the robot (in the
 *    direction the robot is moving AWAY from) should not trigger
 *    intervention, while one directly ahead should.
 *
 * 6. RELATIVE VELOCITY: A pedestrian moving away from the robot should
 *    cause less intervention than a stationary one at the same distance.
 *
 * 7. MULTI-PEDESTRIAN: With multiple pedestrians, the most restrictive
 *    constraint should dominate (minimum clearance wins).
 *
 * 8. MINIMAL INTERVENTION: The output should be as close to the nominal
 *    command as the safety constraints allow (QP minimality).
 */

class SafetyShieldCBFTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    params_.robot_radius = 0.20;
    params_.person_radius = 0.25;
    params_.safety_margin = 0.30;
    params_.alpha = 1.0;
    params_.v_max = 1.0;
    params_.v_min = 0.0;
    params_.omega_max = 1.5;
    params_.stopping_a = 0.35;
    params_.stopping_b = 0.05;
    params_.stopping_c = 0.01;
    params_.intervention_threshold = 0.05;
    cbf_ = std::make_unique<SafetyShieldCBF>(params_);
  }

  CBFParams params_;
  std::unique_ptr<SafetyShieldCBF> cbf_;
};

// -------------------------------------------------------------------
// Property 1: PASSIVITY (no peds → passthrough)
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, NoPedestrians_PassthroughNominal)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};
  std::vector<PersonState> people;

  auto result = cbf_->solve(robot, people, 0.8, 0.3);

  EXPECT_DOUBLE_EQ(result.v, 0.8);
  EXPECT_DOUBLE_EQ(result.omega, 0.3);
  EXPECT_FALSE(result.intervened);
}

TEST_F(SafetyShieldCBFTest, NoPedestrians_SpeedClamped)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  std::vector<PersonState> people;

  auto result = cbf_->solve(robot, people, 2.0, 0.0);

  EXPECT_DOUBLE_EQ(result.v, 1.0);
  EXPECT_DOUBLE_EQ(result.omega, 0.0);
}

TEST_F(SafetyShieldCBFTest, PedestrianFarAway_NoIntervention)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};
  std::vector<PersonState> people = {{10.0, 0.0, 0.0, 0.0}};

  auto result = cbf_->solve(robot, people, 0.8, 0.3);

  EXPECT_NEAR(result.v, 0.8, 0.01);
  EXPECT_FALSE(result.intervened);
}

// -------------------------------------------------------------------
// Property 2: MONOTONIC INTERVENTION (closer → slower)
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, CloserPedestrian_LowerSpeed)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto result_far = cbf_->solve(robot, {{3.0, 0.0, 0.0, 0.0}}, 0.8, 0.0);
  auto result_mid = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);
  auto result_near = cbf_->solve(robot, {{1.0, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  EXPECT_GE(result_far.v, result_mid.v);
  EXPECT_GE(result_mid.v, result_near.v);
}

// -------------------------------------------------------------------
// Property 3: EMERGENCY STOP (barrier < 0)
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, PedestrianInsideSafeZone_EmergencyStop)
{
  // d_safe = 0.20 + 0.25 + 0.30 = 0.75
  // d_stop(0.5) = 0.35*0.25 + 0.05*0.5 + 0.01 = 0.0875 + 0.025 + 0.01 = 0.1225
  // d_eff = 0.75 + 0.1225 = 0.8725
  // Place pedestrian at 0.5m (well inside d_eff)
  RobotState robot{0.0, 0.0, 0.0, 0.5};
  std::vector<PersonState> people = {{0.5, 0.0, 0.0, 0.0}};

  auto result = cbf_->solve(robot, people, 0.8, 0.5);

  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.omega, 0.0);
  EXPECT_TRUE(result.intervened);
}

// -------------------------------------------------------------------
// Property 4: VELOCITY-DEPENDENT SAFETY (higher v → earlier intervention)
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, StoppingDistance_QuadraticInSpeed)
{
  EXPECT_NEAR(cbf_->stoppingDistance(0.0), 0.01, 1e-10);

  double d1 = cbf_->stoppingDistance(0.5);
  double d2 = cbf_->stoppingDistance(1.0);
  EXPECT_GT(d2, d1);
  EXPECT_GT(d1, 0.01);

  double expected_d1 = 0.35 * 0.25 + 0.05 * 0.5 + 0.01;
  EXPECT_NEAR(d1, expected_d1, 1e-10);

  double expected_d2 = 0.35 * 1.0 + 0.05 * 1.0 + 0.01;
  EXPECT_NEAR(d2, expected_d2, 1e-10);
}

TEST_F(SafetyShieldCBFTest, HigherSpeed_IntervenesEarlier)
{
  // Pedestrian at 2m ahead. Robot heading +X.
  // At v=0: d_eff = 0.75 + 0.01 = 0.76 → barrier = 4.0 - 0.5776 = 3.42 (positive, no intervention)
  // At v=1: d_eff = 0.75 + 0.41 = 1.16 → barrier = 4.0 - 1.3456 = 2.65 (smaller barrier)
  RobotState slow_robot{0.0, 0.0, 0.0, 0.2};
  RobotState fast_robot{0.0, 0.0, 0.0, 0.8};
  std::vector<PersonState> people = {{2.0, 0.0, 0.0, 0.0}};

  double h_slow = cbf_->computeBarrier(slow_robot, people[0]);
  double h_fast = cbf_->computeBarrier(fast_robot, people[0]);

  EXPECT_GT(h_slow, h_fast);
}

TEST_F(SafetyShieldCBFTest, EffectiveSafeDistance_GrowsWithSpeed)
{
  double d_at_0 = cbf_->effectiveSafeDistance(0.0);
  double d_at_half = cbf_->effectiveSafeDistance(0.5);
  double d_at_full = cbf_->effectiveSafeDistance(1.0);

  EXPECT_NEAR(d_at_0, 0.76, 1e-10);  // d_safe=0.75, d_stop=0.01
  EXPECT_GT(d_at_half, d_at_0);
  EXPECT_GT(d_at_full, d_at_half);
}

// -------------------------------------------------------------------
// Property 5: DIRECTIONAL AWARENESS
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, PedestrianBehind_LessRestrictive)
{
  // Robot at origin heading +X.
  // Ped ahead at (1.5, 0): robot moving toward ped
  // Ped behind at (-1.5, 0): robot moving away from ped
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto result_ahead = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);
  auto result_behind = cbf_->solve(robot, {{-1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  // The ped behind should allow higher speed (moving away from it)
  EXPECT_GE(result_behind.v, result_ahead.v);
}

TEST_F(SafetyShieldCBFTest, PedestrianToSide_PartialEffect)
{
  // Robot heading +X, ped at (0, 1.2) — perpendicular to motion direction.
  // cos(0)=1, sin(0)=0 → a = 2*0*1 + 2*1.2*0 = 0
  // Constraint coefficient a≈0 → constraint is either trivially satisfied or infeasible
  // depending on h. For a ped at (0, 1.2), D=1.2, d_eff≈0.87 → h = 1.44 - 0.76 = positive.
  // With a=0 and h>0: rhs = -(0 + alpha*h) < 0, so constraint 0*v >= negative → always satisfied.
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto result = cbf_->solve(robot, {{0.0, 1.2, 0.0, 0.0}}, 0.8, 0.0);

  EXPECT_NEAR(result.v, 0.8, 0.01);
}

// -------------------------------------------------------------------
// Property 6: RELATIVE VELOCITY
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, PedestrianMovingAway_LessRestrictive)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  // Ped at 1.5m ahead, stationary
  auto result_static = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  // Same ped but moving away at 0.5 m/s in +X
  auto result_away = cbf_->solve(robot, {{1.5, 0.0, 0.5, 0.0}}, 0.8, 0.0);

  EXPECT_GE(result_away.v, result_static.v);
}

TEST_F(SafetyShieldCBFTest, PedestrianApproaching_MoreRestrictive)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto result_static = cbf_->solve(robot, {{2.0, 0.0, 0.0, 0.0}}, 0.8, 0.0);
  auto result_approach = cbf_->solve(robot, {{2.0, 0.0, -0.5, 0.0}}, 0.8, 0.0);

  EXPECT_LE(result_approach.v, result_static.v);
}

// -------------------------------------------------------------------
// Property 7: MULTI-PEDESTRIAN
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, MultiplePedestrians_MostRestrictiveWins)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto result_far = cbf_->solve(robot, {{5.0, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  auto result_both = cbf_->solve(robot,
    {{5.0, 0.0, 0.0, 0.0}, {1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  auto result_close_only = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  // Adding the far pedestrian should not change the result (close one dominates)
  EXPECT_LE(result_both.v, result_far.v);
  EXPECT_NEAR(result_both.v, result_close_only.v, 0.01);
}

TEST_F(SafetyShieldCBFTest, PedestriansOnBothSides_BothConstrain)
{
  // Robot heading +X. Peds ahead-left and ahead-right at close range.
  RobotState robot{0.0, 0.0, 0.0, 0.3};

  auto result_one = cbf_->solve(robot, {{1.2, 0.3, 0.0, 0.0}}, 0.8, 0.0);
  auto result_two = cbf_->solve(robot,
    {{1.2, 0.3, 0.0, 0.0}, {1.2, -0.3, 0.0, 0.0}}, 0.8, 0.0);

  EXPECT_LE(result_two.v, result_one.v + 0.01);
}

// -------------------------------------------------------------------
// Property 8: MINIMAL INTERVENTION (QP optimality)
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, MinimalIntervention_OutputClosestToNominal)
{
  // For a ped ahead at 1.5m, the CBF upper-bounds v (since a<0 for peds ahead).
  // Verify: output = clamp(v_nom, v_lb, v_ub), i.e., as close to v_nom as allowed.
  RobotState robot{0.0, 0.0, 0.0, 0.5};

  auto diag = cbf_->diagnose(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8);
  auto result = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  if (diag.feasible) {
    EXPECT_GE(result.v, diag.v_lower_bound - 1e-10);
    EXPECT_LE(result.v, diag.v_upper_bound + 1e-10);
    double expected = std::clamp(0.8, diag.v_lower_bound, diag.v_upper_bound);
    EXPECT_NEAR(result.v, expected, 1e-10);
  }

  // With ped far away (10m), v_nom=0.8 should be fully feasible (no intervention)
  auto result_far = cbf_->solve(robot, {{10.0, 0.0, 0.0, 0.0}}, 0.8, 0.0);
  EXPECT_NEAR(result_far.v, 0.8, 0.01);
  EXPECT_FALSE(result_far.intervened);
}

TEST_F(SafetyShieldCBFTest, OmegaPassthrough_WhenNoCBFConstraintOnOmega)
{
  // CBF constraints don't involve omega, so it should pass through (clamped to bounds).
  // omega_nom=1.2 is within [-1.5, 1.5], so output should match (OSQP may have tiny numerical diff).
  RobotState robot{0.0, 0.0, 0.0, 0.5};
  std::vector<PersonState> people = {{2.0, 0.0, 0.0, 0.0}};

  auto result = cbf_->solve(robot, people, 0.5, 1.2);

  EXPECT_NEAR(result.omega, 1.2, 1e-5);
}

TEST_F(SafetyShieldCBFTest, OmegaClamped_ToMaxBounds)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  std::vector<PersonState> people;

  auto result = cbf_->solve(robot, people, 0.5, 3.0);

  EXPECT_DOUBLE_EQ(result.omega, 1.5);
}

// -------------------------------------------------------------------
// Barrier value computation tests
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, BarrierValue_PositiveWhenFarAway)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  PersonState person{5.0, 0.0, 0.0, 0.0};

  double h = cbf_->computeBarrier(robot, person);

  // D=5, d_eff=0.75+0.01=0.76, h = 25 - 0.5776 = 24.42
  EXPECT_GT(h, 0.0);
  EXPECT_NEAR(h, 25.0 - 0.76 * 0.76, 1e-6);
}

TEST_F(SafetyShieldCBFTest, BarrierValue_NegativeWhenTooClose)
{
  RobotState robot{0.0, 0.0, 0.0, 1.0};
  PersonState person{0.5, 0.0, 0.0, 0.0};

  double h = cbf_->computeBarrier(robot, person);

  // D=0.5, d_eff=0.75+0.41=1.16, h = 0.25 - 1.3456 < 0
  EXPECT_LT(h, 0.0);
}

TEST_F(SafetyShieldCBFTest, BarrierValue_ZeroAtBoundary)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  double d_eff = cbf_->effectiveSafeDistance(0.0);

  PersonState person{d_eff, 0.0, 0.0, 0.0};

  double h = cbf_->computeBarrier(robot, person);

  EXPECT_NEAR(h, 0.0, 1e-10);
}

// -------------------------------------------------------------------
// Diagnostics tests
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, Diagnostics_ReportsAllBarriers)
{
  RobotState robot{0.0, 0.0, 0.0, 0.3};
  std::vector<PersonState> people = {
    {3.0, 0.0, 0.0, 0.0},
    {1.5, 0.5, 0.0, 0.0},
    {-2.0, 0.0, 0.0, 0.0}
  };

  auto diag = cbf_->diagnose(robot, people, 0.8);

  EXPECT_EQ(diag.barrier_values.size(), 3u);
  EXPECT_EQ(diag.constraint_margins.size(), 3u);
  EXPECT_TRUE(diag.feasible);
}

TEST_F(SafetyShieldCBFTest, Diagnostics_Infeasible_WhenPedInsideZone)
{
  RobotState robot{0.0, 0.0, 0.0, 0.5};
  std::vector<PersonState> people = {{0.3, 0.0, 0.0, 0.0}};

  auto diag = cbf_->diagnose(robot, people, 0.8);

  EXPECT_FALSE(diag.feasible);
}

// -------------------------------------------------------------------
// Edge cases
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, ZeroNominalCommand_NoIntervention)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  std::vector<PersonState> people = {{2.0, 0.0, 0.0, 0.0}};

  auto result = cbf_->solve(robot, people, 0.0, 0.0);

  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_FALSE(result.intervened);
}

TEST_F(SafetyShieldCBFTest, NegativeNominal_ClampedToVMin)
{
  RobotState robot{0.0, 0.0, 0.0, 0.0};
  std::vector<PersonState> people;

  auto result = cbf_->solve(robot, people, -0.5, 0.0);

  EXPECT_DOUBLE_EQ(result.v, 0.0);  // v_min = 0
}

TEST_F(SafetyShieldCBFTest, PedestrianAtRobotPosition_FullStop)
{
  RobotState robot{1.0, 1.0, 0.0, 0.5};
  std::vector<PersonState> people = {{1.0, 1.0, 0.0, 0.0}};

  auto result = cbf_->solve(robot, people, 0.8, 0.5);

  EXPECT_DOUBLE_EQ(result.v, 0.0);
  EXPECT_DOUBLE_EQ(result.omega, 0.0);
  EXPECT_TRUE(result.intervened);
}

TEST_F(SafetyShieldCBFTest, NonZeroHeading_ConstraintRotatesCorrectly)
{
  // Robot heading +Y (theta = pi/2). Ped at (0, 1.5) is directly ahead.
  RobotState robot{0.0, 0.0, M_PI / 2.0, 0.5};

  auto result_ahead = cbf_->solve(robot, {{0.0, 1.5, 0.0, 0.0}}, 0.8, 0.0);

  // Ped at (1.5, 0) is to the right (perpendicular to heading).
  auto result_side = cbf_->solve(robot, {{1.5, 0.0, 0.0, 0.0}}, 0.8, 0.0);

  EXPECT_LE(result_ahead.v, result_side.v + 0.01);
}

// -------------------------------------------------------------------
// Regression: verify the QP produces correct bounds
// -------------------------------------------------------------------

TEST_F(SafetyShieldCBFTest, DiagnosticBounds_MatchSolverOutput)
{
  RobotState robot{0.0, 0.0, 0.0, 0.4};
  std::vector<PersonState> people = {
    {1.8, 0.2, -0.2, 0.0},
    {3.0, -0.5, 0.0, 0.1}
  };

  auto diag = cbf_->diagnose(robot, people, 0.8);
  auto result = cbf_->solve(robot, people, 0.8, 0.0);

  if (diag.feasible) {
    EXPECT_GE(result.v, diag.v_lower_bound - 1e-4);
    EXPECT_LE(result.v, diag.v_upper_bound + 1e-4);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
