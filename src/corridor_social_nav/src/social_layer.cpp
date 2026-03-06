#include "corridor_social_nav/social_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace corridor_social_nav
{

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) { return; }

  declareParameter("gaussian_sigma", rclcpp::ParameterValue(0.5));
  declareParameter("amplitude", rclcpp::ParameterValue(252.0));
  declareParameter("cutoff_distance", rclcpp::ParameterValue(2.0));

  node->get_parameter(name_ + ".gaussian_sigma", gaussian_sigma_);
  node->get_parameter(name_ + ".amplitude", amplitude_);
  node->get_parameter(name_ + ".cutoff_distance", cutoff_distance_);

  people_sub_ = node->create_subscription<corridor_social_nav::msg::TrackedPeople>(
    "/tracked_people", rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  current_ = true;
  enabled_ = true;
}

void SocialLayer::peopleCallback(
  const corridor_social_nav::msg::TrackedPeople::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_ = msg;
}

void SocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  if (!latest_people_) { return; }

  for (const auto & person : latest_people_->people) {
    *min_x = std::min(*min_x, person.position.x - cutoff_distance_);
    *min_y = std::min(*min_y, person.position.y - cutoff_distance_);
    *max_x = std::max(*max_x, person.position.x + cutoff_distance_);
    *max_y = std::max(*max_y, person.position.y + cutoff_distance_);
  }
}

void SocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) { return; }

  std::lock_guard<std::mutex> lock(people_mutex_);
  if (!latest_people_) { return; }

  double inv_2sig2 = 1.0 / (2.0 * gaussian_sigma_ * gaussian_sigma_);

  for (const auto & person : latest_people_->people) {
    double px = person.position.x;
    double py = person.position.y;

    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        double dx = wx - px;
        double dy = wy - py;
        double dist_sq = dx * dx + dy * dy;

        if (dist_sq > cutoff_distance_ * cutoff_distance_) { continue; }

        double cost = amplitude_ * std::exp(-dist_sq * inv_2sig2);
        unsigned char grid_cost = static_cast<unsigned char>(
          std::min(cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE - 1)));

        unsigned char old_cost = master_grid.getCost(i, j);
        if (grid_cost > old_cost) {
          master_grid.setCost(i, j, grid_cost);
        }
      }
    }
  }
}

void SocialLayer::reset()
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_.reset();
}

}  // namespace corridor_social_nav

PLUGINLIB_EXPORT_CLASS(corridor_social_nav::SocialLayer, nav2_costmap_2d::Layer)
