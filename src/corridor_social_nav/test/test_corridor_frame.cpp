#include <gtest/gtest.h>
#include "corridor_social_nav/corridor_frame.hpp"
#include <cmath>
#include <vector>

using namespace corridor_social_nav;

/*
 * Test design rationale:
 *
 * The corridor frame provides the Frenet coordinate transform that is the
 * mathematical backbone of the predictive social costmap.  If toFrenet or
 * toCartesian have bugs, the entire risk-tube prediction will project costs
 * to wrong cells, silently producing incorrect results that look plausible.
 *
 * The tests are organized around six critical properties:
 *
 * 1. ROUND-TRIP IDENTITY: toCartesian(toFrenet(x,y)) == (x,y) for any
 *    point on or near the path.  This is the single most important
 *    invariant; if it fails, predictions will be placed at wrong locations.
 *
 * 2. ARC-LENGTH CONSISTENCY: The s-coordinate must increase monotonically
 *    along the path, and totalLength() must equal the sum of segment lengths.
 *
 * 3. LATERAL SIGN CONVENTION: d > 0 means left of path (CCW from tangent).
 *    Getting this wrong would flip the "keep-right" penalty in the costmap.
 *
 * 4. VELOCITY DECOMPOSITION: ds must be positive when moving along the
 *    path direction, and dd must be zero when moving exactly along-path.
 *
 * 5. MULTI-SEGMENT PATHS: The projection must work correctly across
 *    segment boundaries (L-shaped paths, S-curves).
 *
 * 6. EDGE CASES: Empty path, single-point path, zero-length segments,
 *    query point beyond path endpoints, path with coincident waypoints.
 */

namespace
{

nav_msgs::msg::Path makePath(const std::vector<std::pair<double,double>> & pts)
{
  nav_msgs::msg::Path path;
  for (const auto & [px, py] : pts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.pose.position.x = px;
    ps.pose.position.y = py;
    ps.pose.position.z = 0.0;
    path.poses.push_back(ps);
  }
  return path;
}

}  // namespace

class CorridorFrameTest : public ::testing::Test
{
protected:
  CorridorFrame frame_;
};

// ===== 1. Round-trip identity =====

TEST_F(CorridorFrameTest, RoundTrip_PointOnStraightPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  double qx = 5.0, qy = 0.0;
  auto fp = frame_.toFrenet(qx, qy);
  auto [rx, ry] = frame_.toCartesian(fp.s, fp.d);
  EXPECT_NEAR(rx, qx, 1e-9);
  EXPECT_NEAR(ry, qy, 1e-9);
}

TEST_F(CorridorFrameTest, RoundTrip_PointOffsetFromStraightPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  double qx = 3.0, qy = 1.5;
  auto fp = frame_.toFrenet(qx, qy);
  auto [rx, ry] = frame_.toCartesian(fp.s, fp.d);
  EXPECT_NEAR(rx, qx, 1e-9);
  EXPECT_NEAR(ry, qy, 1e-9);
}

TEST_F(CorridorFrameTest, RoundTrip_LShapedPath)
{
  frame_.updatePath(makePath({{0, 0}, {5, 0}, {5, 5}}));

  double qx = 5.0, qy = 2.5;
  auto fp = frame_.toFrenet(qx, qy);
  auto [rx, ry] = frame_.toCartesian(fp.s, fp.d);
  EXPECT_NEAR(rx, qx, 1e-6);
  EXPECT_NEAR(ry, qy, 1e-6);
}

TEST_F(CorridorFrameTest, RoundTrip_PointLeftOfPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  double qx = 7.0, qy = -0.8;
  auto fp = frame_.toFrenet(qx, qy);
  auto [rx, ry] = frame_.toCartesian(fp.s, fp.d);
  EXPECT_NEAR(rx, qx, 1e-9);
  EXPECT_NEAR(ry, qy, 1e-9);
}

// ===== 2. Arc-length consistency =====

TEST_F(CorridorFrameTest, TotalLength_StraightPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));
  EXPECT_NEAR(frame_.totalLength(), 10.0, 1e-9);
}

TEST_F(CorridorFrameTest, TotalLength_LShapedPath)
{
  frame_.updatePath(makePath({{0, 0}, {5, 0}, {5, 5}}));
  EXPECT_NEAR(frame_.totalLength(), 10.0, 1e-9);
}

TEST_F(CorridorFrameTest, TotalLength_DiagonalPath)
{
  frame_.updatePath(makePath({{0, 0}, {3, 4}}));
  EXPECT_NEAR(frame_.totalLength(), 5.0, 1e-9);
}

TEST_F(CorridorFrameTest, SCoordinate_MonotonicallyIncreasing)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  double prev_s = -1.0;
  for (double x = 0.0; x <= 10.0; x += 0.5) {
    auto fp = frame_.toFrenet(x, 0.0);
    EXPECT_GT(fp.s, prev_s) << "s not monotonically increasing at x=" << x;
    prev_s = fp.s;
  }
}

TEST_F(CorridorFrameTest, SCoordinate_MatchesDistanceAlongPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp = frame_.toFrenet(7.0, 0.0);
  EXPECT_NEAR(fp.s, 7.0, 1e-9);
}

TEST_F(CorridorFrameTest, SCoordinate_CorrectAtSegmentBoundary)
{
  frame_.updatePath(makePath({{0, 0}, {5, 0}, {5, 5}}));

  auto fp = frame_.toFrenet(5.0, 0.0);
  EXPECT_NEAR(fp.s, 5.0, 1e-9);
}

// ===== 3. Lateral sign convention =====

TEST_F(CorridorFrameTest, LateralSign_LeftOfEastPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp_left = frame_.toFrenet(5.0, 1.0);
  EXPECT_GT(fp_left.d, 0.0) << "Point north of east-going path should have d > 0 (left)";

  auto fp_right = frame_.toFrenet(5.0, -1.0);
  EXPECT_LT(fp_right.d, 0.0) << "Point south of east-going path should have d < 0 (right)";
}

TEST_F(CorridorFrameTest, LateralSign_LeftOfNorthPath)
{
  frame_.updatePath(makePath({{0, 0}, {0, 10}}));

  auto fp_left = frame_.toFrenet(-1.0, 5.0);
  EXPECT_GT(fp_left.d, 0.0) << "Point west of north-going path should have d > 0 (left)";

  auto fp_right = frame_.toFrenet(1.0, 5.0);
  EXPECT_LT(fp_right.d, 0.0) << "Point east of north-going path should have d < 0 (right)";
}

TEST_F(CorridorFrameTest, LateralMagnitude_MatchesDistance)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp = frame_.toFrenet(5.0, 2.3);
  EXPECT_NEAR(std::abs(fp.d), 2.3, 1e-9);
}

TEST_F(CorridorFrameTest, LateralZero_OnPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp = frame_.toFrenet(5.0, 0.0);
  EXPECT_NEAR(fp.d, 0.0, 1e-9);
}

// ===== 4. Velocity decomposition =====

TEST_F(CorridorFrameTest, Velocity_AlongPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fs = frame_.toFrenetState(5.0, 0.0, 1.0, 0.0);
  EXPECT_NEAR(fs.ds, 1.0, 1e-9) << "Moving east along east path: ds should be 1.0";
  EXPECT_NEAR(fs.dd, 0.0, 1e-9) << "Moving east along east path: dd should be 0.0";
}

TEST_F(CorridorFrameTest, Velocity_AgainstPath)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fs = frame_.toFrenetState(5.0, 0.0, -1.0, 0.0);
  EXPECT_NEAR(fs.ds, -1.0, 1e-9) << "Moving west along east path: ds should be -1.0";
  EXPECT_NEAR(fs.dd, 0.0, 1e-9);
}

TEST_F(CorridorFrameTest, Velocity_LateralOnly)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fs = frame_.toFrenetState(5.0, 0.0, 0.0, 1.5);
  EXPECT_NEAR(fs.ds, 0.0, 1e-9);
  EXPECT_NEAR(fs.dd, 1.5, 1e-9) << "Moving north (lateral to east path): dd should be 1.5";
}

TEST_F(CorridorFrameTest, Velocity_Diagonal)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  double vx = 1.0, vy = 1.0;
  auto fs = frame_.toFrenetState(5.0, 0.0, vx, vy);
  EXPECT_NEAR(fs.ds, 1.0, 1e-9);
  EXPECT_NEAR(fs.dd, 1.0, 1e-9);
}

TEST_F(CorridorFrameTest, Velocity_OnNorthSegmentOfLPath)
{
  frame_.updatePath(makePath({{0, 0}, {5, 0}, {5, 5}}));

  auto fs = frame_.toFrenetState(5.0, 2.5, 0.0, 1.0);
  EXPECT_NEAR(fs.ds, 1.0, 1e-9) << "Moving north on vertical segment: ds = 1.0";
  EXPECT_NEAR(fs.dd, 0.0, 1e-9);
}

// ===== 5. Multi-segment paths =====

TEST_F(CorridorFrameTest, MultiSegment_ProjectionSelectsClosestSegment)
{
  frame_.updatePath(makePath({{0, 0}, {5, 0}, {5, 5}}));

  auto fp1 = frame_.toFrenet(2.5, 0.0);
  EXPECT_NEAR(fp1.s, 2.5, 1e-9);
  EXPECT_NEAR(fp1.d, 0.0, 1e-9);

  auto fp2 = frame_.toFrenet(5.0, 3.0);
  EXPECT_NEAR(fp2.s, 8.0, 1e-9);
}

TEST_F(CorridorFrameTest, MultiSegment_ThreeSegmentZigZag)
{
  frame_.updatePath(makePath({{0, 0}, {3, 0}, {3, 4}, {6, 4}}));

  double expected_len = 3.0 + 4.0 + 3.0;
  EXPECT_NEAR(frame_.totalLength(), expected_len, 1e-9);
}

// ===== 6. Edge cases =====

TEST_F(CorridorFrameTest, EdgeCase_EmptyPath)
{
  frame_.updatePath(makePath({}));
  EXPECT_FALSE(frame_.isValid());
  EXPECT_NEAR(frame_.totalLength(), 0.0, 1e-12);

  auto fp = frame_.toFrenet(1.0, 2.0);
  EXPECT_NEAR(fp.s, 0.0, 1e-12);
  EXPECT_NEAR(fp.d, 0.0, 1e-12);
}

TEST_F(CorridorFrameTest, EdgeCase_SinglePoint)
{
  frame_.updatePath(makePath({{5, 5}}));
  EXPECT_FALSE(frame_.isValid());
}

TEST_F(CorridorFrameTest, EdgeCase_QueryBeyondPathEnd)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp = frame_.toFrenet(15.0, 0.0);
  EXPECT_NEAR(fp.s, 10.0, 1e-9) << "Query beyond end should clamp to last point s";
}

TEST_F(CorridorFrameTest, EdgeCase_QueryBeforePathStart)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto fp = frame_.toFrenet(-5.0, 0.0);
  EXPECT_NEAR(fp.s, 0.0, 1e-9) << "Query before start should project to s=0";
}

TEST_F(CorridorFrameTest, EdgeCase_ToCartesian_ClampsSToRange)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));

  auto [x1, y1] = frame_.toCartesian(-5.0, 0.0);
  EXPECT_NEAR(x1, 0.0, 1e-9) << "s < 0 should clamp to path start";

  auto [x2, y2] = frame_.toCartesian(20.0, 0.0);
  EXPECT_NEAR(x2, 10.0, 1e-9) << "s > L should clamp to path end";
}

TEST_F(CorridorFrameTest, EdgeCase_PathUpdate_ClearsPrevious)
{
  frame_.updatePath(makePath({{0, 0}, {10, 0}}));
  EXPECT_NEAR(frame_.totalLength(), 10.0, 1e-9);

  frame_.updatePath(makePath({{0, 0}, {3, 4}}));
  EXPECT_NEAR(frame_.totalLength(), 5.0, 1e-9);
}

TEST_F(CorridorFrameTest, EdgeCase_VelocityWithInvalidPath)
{
  frame_.updatePath(makePath({}));
  auto fs = frame_.toFrenetState(1.0, 2.0, 0.5, 0.3);
  EXPECT_NEAR(fs.ds, 0.0, 1e-12);
  EXPECT_NEAR(fs.dd, 0.0, 1e-12);
}
