#ifndef CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_
#define CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "corridor_social_nav/msg/tracked_people.hpp"

namespace corridor_social_nav
{

class SocialLayer : public nav2_costmap_2d::Layer
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
  bool isClearable() override { return true; }

private:
  void peopleCallback(const corridor_social_nav::msg::TrackedPeople::SharedPtr msg);

  rclcpp::Subscription<corridor_social_nav::msg::TrackedPeople>::SharedPtr people_sub_;
  corridor_social_nav::msg::TrackedPeople::SharedPtr latest_people_;
  std::mutex people_mutex_;

  double gaussian_sigma_{0.5};
  double amplitude_{252.0};
  double cutoff_distance_{2.0};
};

}  // namespace corridor_social_nav

#endif  // CORRIDOR_SOCIAL_NAV__SOCIAL_LAYER_HPP_
