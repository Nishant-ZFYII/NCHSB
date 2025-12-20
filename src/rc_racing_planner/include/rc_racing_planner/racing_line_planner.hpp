#ifndef RC_RACING_PLANNER__RACING_LINE_PLANNER_HPP_
#define RC_RACING_PLANNER__RACING_LINE_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace rc_racing_planner
{

/**
 * @brief Racing waypoint structure
 */
struct RacingWaypoint
{
  double s;     // Arc-length along path
  double x;     // X position (meters)
  double y;     // Y position (meters)
  double yaw;   // Heading (radians)
  double v;     // Recommended velocity (m/s)
};

/**
 * @brief Racing Line Global Planner
 * 
 * This planner serves a precomputed racing line instead of computing paths dynamically.
 * Designed for narrow corridor navigation with Ackermann steering robots.
 * 
 * Key features:
 * - Loads racing line from YAML file at startup
 * - Projects start/goal poses to nearest waypoints on racing line
 * - Returns path segment between projected points
 * - Supports closed-loop (circuit) paths
 */
class RacingLinePlanner : public nav2_core::GlobalPlanner
{
public:
  RacingLinePlanner() = default;
  ~RacingLinePlanner() = default;

  /**
   * @brief Configure the planner
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  /**
   * @brief Create a path from start to goal
   */
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  /**
   * @brief Load racing line from YAML file
   */
  bool loadRacingLine(const std::string & filename);

  /**
   * @brief Find closest waypoint index for a given pose
   */
  int projectPoseToPath(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Extract path segment between two indices
   */
  nav_msgs::msg::Path extractPathSegment(int start_idx, int goal_idx);

  /**
   * @brief Convert yaw angle to quaternion
   */
  geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  // Members
  std::string racing_line_file_;
  std::vector<RacingWaypoint> racing_waypoints_;
  bool is_closed_loop_;
  
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_;
  std::string name_;
  
  rclcpp::Logger logger_{rclcpp::get_logger("RacingLinePlanner")};
};

}  // namespace rc_racing_planner

#endif  // RC_RACING_PLANNER__RACING_LINE_PLANNER_HPP_

