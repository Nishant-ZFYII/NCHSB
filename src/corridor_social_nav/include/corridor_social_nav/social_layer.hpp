#ifndef CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_
#define CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "corridor_social_nav/msg/tracked_people.hpp"

namespace corridor_social_nav
{

class SocialLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  SocialLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  void matchSize() override;
  bool isClearable() override { return true; }

private:
  void peopleCallback(const corridor_social_nav::msg::TrackedPeople::SharedPtr msg);
  bool transformPeopleToGlobal();

  rclcpp::Subscription<corridor_social_nav::msg::TrackedPeople>::SharedPtr people_sub_;
  corridor_social_nav::msg::TrackedPeople::SharedPtr latest_people_;
  std::mutex people_mutex_;
  rclcpp::Time last_people_time_;

  // TF for frame transformation
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string global_frame_;

  // Transformed positions (in costmap's global_frame, e.g. odom)
  struct TransformedPerson {
    double x, y;
  };
  std::vector<TransformedPerson> transformed_people_;

  double gaussian_sigma_{0.45};
  double amplitude_{160.0};
  double cutoff_distance_{1.2};
  double stale_timeout_{0.5};

  // Previous cycle's bounds for clearing
  double prev_min_x_{1e6}, prev_min_y_{1e6};
  double prev_max_x_{-1e6}, prev_max_y_{-1e6};
  bool has_prev_bounds_{false};
};

}  // namespace corridor_social_nav

#endif  // CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_
