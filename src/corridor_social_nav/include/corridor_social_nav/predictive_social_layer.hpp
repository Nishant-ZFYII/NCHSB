#ifndef CORRIDOR_SOCIAL_NAV__PREDICTIVE_SOCIAL_LAYER_HPP_
#define CORRIDOR_SOCIAL_NAV__PREDICTIVE_SOCIAL_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "corridor_social_nav/msg/tracked_people.hpp"
#include "corridor_social_nav/corridor_frame.hpp"

namespace corridor_social_nav
{

class PredictiveSocialLayer : public nav2_costmap_2d::Layer
{
public:
  PredictiveSocialLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void peopleCallback(const corridor_social_nav::msg::TrackedPeople::SharedPtr msg);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);

  rclcpp::Subscription<corridor_social_nav::msg::TrackedPeople>::SharedPtr people_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  corridor_social_nav::msg::TrackedPeople::SharedPtr latest_people_;
  std::mutex people_mutex_;

  CorridorFrame corridor_frame_;
  std::mutex frame_mutex_;

  // Prediction parameters (paper Section IV-B)
  double prediction_horizon_{2.0};
  double prediction_step_{0.2};
  double sigma_s_0_{0.3};
  double sigma_d_0_{0.2};
  double k_s_{0.3};
  double k_d_{0.1};
  double max_cost_{252.0};
  bool anisotropic_{true};
  bool use_corridor_frame_{true};

  // Corridor geometry costs (paper Section IV-C)
  double wall_clearance_alpha_{50.0};
  double wall_clearance_lambda_{0.3};
  double keep_right_alpha_{10.0};
};

}  // namespace corridor_social_nav

#endif  // CORRIDOR_SOCIAL_NAV__PREDICTIVE_SOCIAL_LAYER_HPP_
