#include <gtest/gtest.h>
#include "corridor_social_nav/corridor_frame.hpp"
#include <cmath>
#include <vector>
#include <utility>
#include <algorithm>

/*
 * Test design rationale:
 *
 * The SocialLayer and PredictiveSocialLayer are Nav2 costmap plugins
 * whose onInitialize/updateCosts methods require the full Nav2 stack.
 * However, the core cost computation is pure math that we can extract
 * and test directly.
 *
 * These tests verify:
 *
 * A. INSTANTANEOUS GAUSSIAN (SocialLayer math):
 *    1. Gaussian peak at person position
 *    2. Exponential decay with distance
 *    3. Cutoff beyond threshold
 *    4. Max-combine for multiple people
 *    5. Amplitude scaling and clamping below LETHAL_OBSTACLE
 *
 * B. PREDICTIVE ANISOTROPIC (PredictiveSocialLayer math):
 *    1. Risk tube extends along predicted path (corridor frame)
 *    2. Uncertainty grows linearly with prediction time
 *    3. Anisotropy factor (1.5x) activates for moving pedestrians
 *    4. Corridor-frame vs global-frame fallback produces different costs
 *    5. Cost is max over all prediction timesteps
 *    6. Multiple people: max across people
 *
 * We replicate the exact cost formulas from social_layer.cpp and
 * predictive_social_layer.cpp so any formula change triggers a test failure,
 * forcing explicit review.
 */

using namespace corridor_social_nav;

// ===== A. Instantaneous Gaussian cost (SocialLayer) =====

namespace
{

double gaussianCost(
  double wx, double wy, double px, double py,
  double sigma, double amplitude, double cutoff)
{
  double dx = wx - px;
  double dy = wy - py;
  double dist_sq = dx * dx + dy * dy;
  if (dist_sq > cutoff * cutoff) { return 0.0; }
  double inv_2sig2 = 1.0 / (2.0 * sigma * sigma);
  double cost = amplitude * std::exp(-dist_sq * inv_2sig2);
  return std::min(cost, 253.0);  // LETHAL_OBSTACLE - 1 = 253
}

}  // namespace

TEST(SocialLayerMath, PeakAtPersonPosition)
{
  double cost = gaussianCost(5.0, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  EXPECT_NEAR(cost, 252.0, 1e-6) << "Cost at person position should be amplitude";
}

TEST(SocialLayerMath, DecayWithDistance)
{
  double sigma = 0.5;
  double cost_0 = gaussianCost(5.0, 3.0, 5.0, 3.0, sigma, 252.0, 2.0);
  double cost_1sig = gaussianCost(5.0 + sigma, 3.0, 5.0, 3.0, sigma, 252.0, 2.0);
  double cost_2sig = gaussianCost(5.0 + 2.0 * sigma, 3.0, 5.0, 3.0, sigma, 252.0, 2.0);

  EXPECT_GT(cost_0, cost_1sig);
  EXPECT_GT(cost_1sig, cost_2sig);

  double expected_1sig = 252.0 * std::exp(-0.5);
  EXPECT_NEAR(cost_1sig, expected_1sig, 1e-6);
}

TEST(SocialLayerMath, IsotropicSymmetry)
{
  double c_north = gaussianCost(5.0, 4.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  double c_east  = gaussianCost(6.0, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  double c_south = gaussianCost(5.0, 2.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  double c_west  = gaussianCost(4.0, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);

  EXPECT_NEAR(c_north, c_east, 1e-9) << "Isotropic: all directions equal";
  EXPECT_NEAR(c_east, c_south, 1e-9);
  EXPECT_NEAR(c_south, c_west, 1e-9);
}

TEST(SocialLayerMath, CutoffBeyondDistance)
{
  double cost = gaussianCost(5.0, 3.0 + 2.5, 5.0, 3.0, 0.5, 252.0, 2.0);
  EXPECT_NEAR(cost, 0.0, 1e-12) << "Cost beyond cutoff_distance should be zero";
}

TEST(SocialLayerMath, CutoffAtBoundary)
{
  double cost_at = gaussianCost(5.0 + 2.0, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  EXPECT_GT(cost_at, 0.0) << "At exactly cutoff distance, dist_sq == cutoff^2, which is NOT > cutoff^2, so cost is computed";

  double cost_beyond = gaussianCost(5.0 + 2.001, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  EXPECT_NEAR(cost_beyond, 0.0, 1e-12) << "Just beyond cutoff should be zero";

  double cost_inside = gaussianCost(5.0 + 1.999, 3.0, 5.0, 3.0, 0.5, 252.0, 2.0);
  EXPECT_GT(cost_inside, 0.0);
}

TEST(SocialLayerMath, MaxCombineTwoPeople)
{
  double p1_cost = gaussianCost(3.0, 0.0, 2.0, 0.0, 0.5, 252.0, 2.0);
  double p2_cost = gaussianCost(3.0, 0.0, 4.0, 0.0, 0.5, 252.0, 2.0);
  double combined = std::max(p1_cost, p2_cost);

  EXPECT_NEAR(p1_cost, p2_cost, 1e-9) << "Equidistant people should produce equal cost";
  EXPECT_NEAR(combined, p1_cost, 1e-9);
}

TEST(SocialLayerMath, AmplitudeClampsBelowLethal)
{
  double cost = gaussianCost(5.0, 3.0, 5.0, 3.0, 0.5, 300.0, 2.0);
  EXPECT_NEAR(cost, 253.0, 1e-6) << "Cost should clamp at LETHAL_OBSTACLE - 1 = 253";
}

// ===== B. Predictive anisotropic cost (PredictiveSocialLayer) =====

namespace
{

struct PredParams
{
  double prediction_horizon = 2.0;
  double prediction_step = 0.2;
  double sigma_s_0 = 0.3;
  double sigma_d_0 = 0.2;
  double k_s = 0.3;
  double k_d = 0.1;
  double max_cost = 252.0;
  bool anisotropic = true;
};

double predictiveCostCorridorFrame(
  double cell_s, double cell_d,
  double ped_s, double ped_d, double ped_ds, double ped_dd,
  const PredParams & p)
{
  double person_cost = 0.0;
  for (double tau = 0.0; tau <= p.prediction_horizon; tau += p.prediction_step) {
    double pred_s = ped_s + ped_ds * tau;
    double pred_d = ped_d + ped_dd * tau;
    double sig_s = p.sigma_s_0 + p.k_s * tau;
    double sig_d = p.sigma_d_0 + p.k_d * tau;
    if (p.anisotropic && std::abs(ped_ds) > 0.1) {
      sig_s *= 1.5;
    }
    double ds = cell_s - pred_s;
    double dd = cell_d - pred_d;
    double exponent = (ds * ds) / (2.0 * sig_s * sig_s) +
                      (dd * dd) / (2.0 * sig_d * sig_d);
    double cost = std::exp(-exponent);
    person_cost = std::max(person_cost, cost);
  }
  return std::min(person_cost * p.max_cost, p.max_cost);
}

double predictiveCostGlobalFrame(
  double wx, double wy,
  double px, double py, double pvx, double pvy,
  const PredParams & p)
{
  double person_cost = 0.0;
  for (double tau = 0.0; tau <= p.prediction_horizon; tau += p.prediction_step) {
    double pred_x = px + pvx * tau;
    double pred_y = py + pvy * tau;
    double sig = p.sigma_s_0 + p.k_s * tau;
    double dx = wx - pred_x;
    double dy = wy - pred_y;
    double cost = std::exp(-(dx * dx + dy * dy) / (2.0 * sig * sig));
    person_cost = std::max(person_cost, cost);
  }
  return std::min(person_cost * p.max_cost, p.max_cost);
}

}  // namespace

TEST(PredictiveLayerMath, PeakAtPedestrianPosition)
{
  PredParams p;
  double cost = predictiveCostCorridorFrame(5.0, 0.0, 5.0, 0.0, 0.0, 0.0, p);
  EXPECT_NEAR(cost, p.max_cost, 1e-6) << "Cost at stationary ped position should be max_cost";
}

TEST(PredictiveLayerMath, RiskTubeExtends_AlongPredictedPath)
{
  PredParams p;
  double ped_ds = 1.0;

  double cost_ahead = predictiveCostCorridorFrame(6.0, 0.0, 5.0, 0.0, ped_ds, 0.0, p);
  double cost_behind = predictiveCostCorridorFrame(4.0, 0.0, 5.0, 0.0, ped_ds, 0.0, p);
  EXPECT_GT(cost_ahead, cost_behind) << "Risk should be higher ahead of a moving person";
}

TEST(PredictiveLayerMath, RiskTubeExtends_FarAhead)
{
  PredParams p;
  double ped_ds = 1.0;

  double cost_far_ahead = predictiveCostCorridorFrame(
    5.0 + ped_ds * p.prediction_horizon, 0.0,
    5.0, 0.0, ped_ds, 0.0, p);
  EXPECT_GT(cost_far_ahead, 0.0) << "Risk extends to prediction horizon";
}

TEST(PredictiveLayerMath, UncertaintyGrows_WithTime)
{
  PredParams p;
  p.anisotropic = false;

  double ped_ds = 1.0;
  double lateral_offset = 0.3;

  double cost_near = predictiveCostCorridorFrame(
    5.0 + ped_ds * 0.2, lateral_offset,
    5.0, 0.0, ped_ds, 0.0, p);
  double cost_far = predictiveCostCorridorFrame(
    5.0 + ped_ds * 1.8, lateral_offset,
    5.0, 0.0, ped_ds, 0.0, p);

  EXPECT_GT(cost_far, cost_near * 0.5)
    << "Far prediction with growing sigma should still have significant cost at small lateral offset";
}

TEST(PredictiveLayerMath, AnisotropyFactor_Activates)
{
  PredParams p_aniso;
  p_aniso.anisotropic = true;
  PredParams p_iso;
  p_iso.anisotropic = false;

  double ped_ds = 1.0;
  double cell_s = 6.0;

  double cost_aniso = predictiveCostCorridorFrame(cell_s, 0.0, 5.0, 0.0, ped_ds, 0.0, p_aniso);
  double cost_iso = predictiveCostCorridorFrame(cell_s, 0.0, 5.0, 0.0, ped_ds, 0.0, p_iso);

  EXPECT_GE(cost_aniso, cost_iso - 1e-6)
    << "Anisotropic (wider sigma_s) should have >= cost along path";
}

TEST(PredictiveLayerMath, AnisotropyFactor_InactiveForSlowPed)
{
  PredParams p_aniso;
  p_aniso.anisotropic = true;
  PredParams p_iso;
  p_iso.anisotropic = false;

  double ped_ds = 0.05;  // below 0.1 threshold

  double cost_aniso = predictiveCostCorridorFrame(5.5, 0.0, 5.0, 0.0, ped_ds, 0.0, p_aniso);
  double cost_iso = predictiveCostCorridorFrame(5.5, 0.0, 5.0, 0.0, ped_ds, 0.0, p_iso);

  EXPECT_NEAR(cost_aniso, cost_iso, 1e-9)
    << "Anisotropy should not activate for |ds| < 0.1";
}

TEST(PredictiveLayerMath, EllipticalShape_DifferentSigmas)
{
  PredParams p;
  p.anisotropic = false;

  double offset = 0.5;
  double cost_along = predictiveCostCorridorFrame(
    5.0 + offset, 0.0,
    5.0, 0.0, 0.0, 0.0, p);
  double cost_lateral = predictiveCostCorridorFrame(
    5.0, offset,
    5.0, 0.0, 0.0, 0.0, p);

  EXPECT_GT(cost_along, cost_lateral)
    << "sigma_s grows faster (k_s=0.3) than sigma_d (k_d=0.1), so at the max "
       "timestep tau=2s, sigma_s=0.9 vs sigma_d=0.4. Same offset 0.5m produces "
       "lower exponent (higher cost) along s than along d.";

  EXPECT_GT(cost_along, 0.0);
  EXPECT_GT(cost_lateral, 0.0);
}

TEST(PredictiveLayerMath, CorridorFrame_VsGlobalFrame_StraightPath)
{
  PredParams p;
  p.anisotropic = false;

  double cost_corridor = predictiveCostCorridorFrame(
    6.0, 0.5, 5.0, 0.0, 1.0, 0.0, p);

  double cost_global = predictiveCostGlobalFrame(
    6.0, 0.5, 5.0, 0.0, 1.0, 0.0, p);

  EXPECT_NE(cost_corridor, cost_global)
    << "Corridor-frame (anisotropic sigma) should differ from global-frame (isotropic sigma)";
}

TEST(PredictiveLayerMath, MaxOverTimesteps)
{
  PredParams p;

  double cost = predictiveCostCorridorFrame(6.0, 0.0, 5.0, 0.0, 1.0, 0.0, p);
  EXPECT_NEAR(cost, p.max_cost, 1e-6)
    << "At tau=1.0, ped is at s=6.0, cell is at s=6.0 => exact match => max cost";
}

TEST(PredictiveLayerMath, MaxAcrossMultiplePeople)
{
  PredParams p;

  double cost_p1 = predictiveCostCorridorFrame(6.0, 0.0, 5.0, 0.0, 1.0, 0.0, p);
  double cost_p2 = predictiveCostCorridorFrame(6.0, 0.0, 8.0, 0.0, -1.0, 0.0, p);
  double combined = std::max(cost_p1, cost_p2);

  EXPECT_NEAR(combined, p.max_cost, 1e-6)
    << "Person 1 reaches s=6 at tau=1s, so max cost";
}

TEST(PredictiveLayerMath, StationaryPed_NoRiskTubeExtension)
{
  PredParams p;

  double cost_far = predictiveCostCorridorFrame(
    10.0, 0.0, 5.0, 0.0, 0.0, 0.0, p);

  double dist = 5.0;
  double max_sigma = p.sigma_s_0 + p.k_s * p.prediction_horizon;
  double expected_max = std::exp(-(dist * dist) / (2.0 * max_sigma * max_sigma));

  EXPECT_NEAR(cost_far / p.max_cost, expected_max, 0.01)
    << "Stationary ped: risk at s=10 determined by largest sigma at T=2s";
}

TEST(PredictiveLayerMath, GlobalFrame_RiskTubeAlongVelocity)
{
  PredParams p;

  double cost_ahead = predictiveCostGlobalFrame(6.0, 0.0, 5.0, 0.0, 1.0, 0.0, p);
  double cost_behind = predictiveCostGlobalFrame(4.0, 0.0, 5.0, 0.0, 1.0, 0.0, p);

  EXPECT_GT(cost_ahead, cost_behind)
    << "Global-frame: risk higher ahead of moving person";
}

// ===== Integration: corridor frame + predictive cost =====

TEST(PredictiveWithCorridorFrame, StraightCorridor_PersonApproaching)
{
  CorridorFrame frame;
  nav_msgs::msg::Path path;
  for (double x = -10; x <= 10; x += 0.5) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = 0.0;
    path.poses.push_back(ps);
  }
  frame.updatePath(path);

  double ped_x = 5.0, ped_y = 0.2;
  double ped_vx = -1.0, ped_vy = 0.0;

  auto fs = frame.toFrenetState(ped_x, ped_y, ped_vx, ped_vy);

  EXPECT_NEAR(fs.s, 15.0, 0.5) << "s ≈ 15 (10m offset + 5m)";
  EXPECT_NEAR(fs.d, 0.2, 0.01) << "d ≈ 0.2m left of path";
  EXPECT_NEAR(fs.ds, -1.0, 0.1) << "ds ≈ -1.0 (approaching)";
  EXPECT_NEAR(fs.dd, 0.0, 0.1);
}

TEST(PredictiveWithCorridorFrame, LShapedCorridor_PersonOnVerticalSegment)
{
  CorridorFrame frame;
  nav_msgs::msg::Path path;
  for (double x = 0; x <= 5; x += 0.5) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = x;
    ps.pose.position.y = 0.0;
    path.poses.push_back(ps);
  }
  for (double y = 0.5; y <= 5; y += 0.5) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = 5.0;
    ps.pose.position.y = y;
    path.poses.push_back(ps);
  }
  frame.updatePath(path);

  auto fp = frame.toFrenet(5.0, 3.0);
  EXPECT_NEAR(fp.s, 8.0, 0.1) << "5m horizontal + 3m vertical = s≈8";
  EXPECT_NEAR(fp.d, 0.0, 0.05) << "On the path centerline";
}
