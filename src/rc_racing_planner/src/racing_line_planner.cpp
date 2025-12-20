#include "rc_racing_planner/racing_line_planner.hpp"

#include <cmath>
#include <fstream>
#include <limits>

#include <yaml-cpp/yaml.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav2_util/node_utils.hpp"

namespace rc_racing_planner
{

void RacingLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  name_ = name;
  tf_ = tf;
  global_frame_ = costmap_ros->getGlobalFrameID();
  
  auto node = node_.lock();
  logger_ = node->get_logger();
  
  RCLCPP_INFO(logger_, "Configuring RacingLinePlanner plugin: %s", name_.c_str());
  
  // Declare and get parameters
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".racing_line_file",
    rclcpp::ParameterValue(""));
  
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".is_closed_loop",
    rclcpp::ParameterValue(true));
  
  node->get_parameter(name_ + ".racing_line_file", racing_line_file_);
  node->get_parameter(name_ + ".is_closed_loop", is_closed_loop_);
  
  if (racing_line_file_.empty()) {
    RCLCPP_ERROR(logger_, "racing_line_file parameter not set!");
    return;
  }
  
  RCLCPP_INFO(logger_, "Racing line file: %s", racing_line_file_.c_str());
  RCLCPP_INFO(logger_, "Closed loop: %s", is_closed_loop_ ? "true" : "false");
  
  // Load racing line
  if (!loadRacingLine(racing_line_file_)) {
    RCLCPP_ERROR(logger_, "Failed to load racing line!");
    return;
  }
  
  RCLCPP_INFO(logger_, "RacingLinePlanner configured with %zu waypoints", 
              racing_waypoints_.size());
}

bool RacingLinePlanner::loadRacingLine(const std::string & filename)
{
  try {
    RCLCPP_INFO(logger_, "Loading racing line from: %s", filename.c_str());
    
    YAML::Node config = YAML::LoadFile(filename);
    
    if (!config["waypoints"]) {
      RCLCPP_ERROR(logger_, "No 'waypoints' key found in YAML file");
      return false;
    }
    
    const auto & waypoints = config["waypoints"];
    racing_waypoints_.clear();
    racing_waypoints_.reserve(waypoints.size());
    
    for (const auto & wp : waypoints) {
      RacingWaypoint rwp;
      rwp.s = wp["s"].as<double>();
      rwp.x = wp["x"].as<double>();
      rwp.y = wp["y"].as<double>();
      rwp.yaw = wp["yaw"].as<double>();
      rwp.v = wp["v"].as<double>();
      
      racing_waypoints_.push_back(rwp);
    }
    
    RCLCPP_INFO(logger_, "Loaded %zu waypoints from racing line", 
                racing_waypoints_.size());
    
    if (!racing_waypoints_.empty()) {
      RCLCPP_INFO(logger_, "  Total path length: %.2f m", 
                  racing_waypoints_.back().s);
      RCLCPP_INFO(logger_, "  First waypoint: (%.2f, %.2f)", 
                  racing_waypoints_.front().x, racing_waypoints_.front().y);
      RCLCPP_INFO(logger_, "  Last waypoint: (%.2f, %.2f)", 
                  racing_waypoints_.back().x, racing_waypoints_.back().y);
    }
    
    return true;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Failed to load racing line: %s", e.what());
    return false;
  }
}

int RacingLinePlanner::projectPoseToPath(const geometry_msgs::msg::PoseStamped & pose)
{
  // Find closest waypoint by Euclidean distance
  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = 0;
  
  double px = pose.pose.position.x;
  double py = pose.pose.position.y;
  
  for (size_t i = 0; i < racing_waypoints_.size(); ++i) {
    double dx = px - racing_waypoints_[i].x;
    double dy = py - racing_waypoints_[i].y;
    double dist = std::sqrt(dx * dx + dy * dy);
    
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = static_cast<int>(i);
    }
  }
  
  RCLCPP_DEBUG(logger_, "Projected pose (%.2f, %.2f) to waypoint %d (dist: %.3f m)",
               px, py, closest_idx, min_dist);
  
  return closest_idx;
}

geometry_msgs::msg::Quaternion RacingLinePlanner::yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

nav_msgs::msg::Path RacingLinePlanner::extractPathSegment(int start_idx, int goal_idx)
{
  nav_msgs::msg::Path path;
  auto node = node_.lock();
  
  path.header.stamp = node->now();
  path.header.frame_id = global_frame_;
  
  int N = static_cast<int>(racing_waypoints_.size());
  
  if (N == 0) {
    RCLCPP_WARN(logger_, "No waypoints loaded, returning empty path");
    return path;
  }
  
  // Handle closed loop path
  if (is_closed_loop_) {
    // For closed loop, we can go either direction
    // Choose the shorter path
    int forward_dist = (goal_idx - start_idx + N) % N;
    int backward_dist = (start_idx - goal_idx + N) % N;
    
    if (forward_dist <= backward_dist) {
      // Go forward
      for (int i = 0; i <= forward_dist; ++i) {
        int idx = (start_idx + i) % N;
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = racing_waypoints_[idx].x;
        pose.pose.position.y = racing_waypoints_[idx].y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = yawToQuaternion(racing_waypoints_[idx].yaw);
        
        path.poses.push_back(pose);
      }
    } else {
      // Go backward (reverse direction on circuit)
      for (int i = 0; i <= backward_dist; ++i) {
        int idx = (start_idx - i + N) % N;
        
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = racing_waypoints_[idx].x;
        pose.pose.position.y = racing_waypoints_[idx].y;
        pose.pose.position.z = 0.0;
        // Flip yaw for reverse direction
        double reverse_yaw = racing_waypoints_[idx].yaw + M_PI;
        if (reverse_yaw > M_PI) reverse_yaw -= 2 * M_PI;
        pose.pose.orientation = yawToQuaternion(reverse_yaw);
        
        path.poses.push_back(pose);
      }
    }
  } else {
    // Open path - just go from start to goal
    int step = (goal_idx >= start_idx) ? 1 : -1;
    
    for (int idx = start_idx; 
         (step > 0) ? (idx <= goal_idx) : (idx >= goal_idx); 
         idx += step) {
      if (idx < 0 || idx >= N) continue;
      
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = racing_waypoints_[idx].x;
      pose.pose.position.y = racing_waypoints_[idx].y;
      pose.pose.position.z = 0.0;
      pose.pose.orientation = yawToQuaternion(racing_waypoints_[idx].yaw);
      
      path.poses.push_back(pose);
    }
  }
  
  RCLCPP_INFO(logger_, "Created path with %zu poses (start_idx=%d, goal_idx=%d)",
              path.poses.size(), start_idx, goal_idx);
  
  return path;
}

nav_msgs::msg::Path RacingLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  RCLCPP_INFO(logger_, "Creating plan from (%.2f, %.2f) to (%.2f, %.2f)",
              start.pose.position.x, start.pose.position.y,
              goal.pose.position.x, goal.pose.position.y);
  
  // Project start and goal to racing line
  int start_idx = projectPoseToPath(start);
  int goal_idx = projectPoseToPath(goal);
  
  RCLCPP_INFO(logger_, "Projected: start_idx=%d, goal_idx=%d", start_idx, goal_idx);
  
  // Extract path along racing line
  nav_msgs::msg::Path path = extractPathSegment(start_idx, goal_idx);
  
  // Add robot's actual start position at the beginning to connect
  // This fixes the "gap" between robot and racing line
  if (!path.poses.empty()) {
    geometry_msgs::msg::PoseStamped start_pose = start;
    start_pose.header = path.header;
    path.poses.insert(path.poses.begin(), start_pose);
  }
  
  // Add actual goal position at the end
  if (!path.poses.empty()) {
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header = path.header;
    path.poses.push_back(goal_pose);
  }
  
  RCLCPP_INFO(logger_, "Final path has %zu poses", path.poses.size());
  
  return path;
}

void RacingLinePlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up RacingLinePlanner");
  racing_waypoints_.clear();
}

void RacingLinePlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating RacingLinePlanner with %zu waypoints",
              racing_waypoints_.size());
}

void RacingLinePlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating RacingLinePlanner");
}

}  // namespace rc_racing_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rc_racing_planner::RacingLinePlanner, nav2_core::GlobalPlanner)

