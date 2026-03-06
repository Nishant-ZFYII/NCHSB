#include "corridor_social_nav/predictive_social_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <algorithm>

namespace corridor_social_nav
{

void PredictiveSocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) { return; }

  declareParameter("prediction_horizon", rclcpp::ParameterValue(2.0));
  declareParameter("prediction_step", rclcpp::ParameterValue(0.2));
  declareParameter("sigma_s_0", rclcpp::ParameterValue(0.3));
  declareParameter("sigma_d_0", rclcpp::ParameterValue(0.2));
  declareParameter("k_s", rclcpp::ParameterValue(0.3));
  declareParameter("k_d", rclcpp::ParameterValue(0.1));
  declareParameter("max_cost", rclcpp::ParameterValue(252.0));
  declareParameter("anisotropic", rclcpp::ParameterValue(true));
  declareParameter("use_corridor_frame", rclcpp::ParameterValue(true));
  declareParameter("wall_clearance_alpha", rclcpp::ParameterValue(50.0));
  declareParameter("wall_clearance_lambda", rclcpp::ParameterValue(0.3));
  declareParameter("keep_right_alpha", rclcpp::ParameterValue(10.0));

  node->get_parameter(name_ + ".prediction_horizon", prediction_horizon_);
  node->get_parameter(name_ + ".prediction_step", prediction_step_);
  node->get_parameter(name_ + ".sigma_s_0", sigma_s_0_);
  node->get_parameter(name_ + ".sigma_d_0", sigma_d_0_);
  node->get_parameter(name_ + ".k_s", k_s_);
  node->get_parameter(name_ + ".k_d", k_d_);
  node->get_parameter(name_ + ".max_cost", max_cost_);
  node->get_parameter(name_ + ".anisotropic", anisotropic_);
  node->get_parameter(name_ + ".use_corridor_frame", use_corridor_frame_);
  node->get_parameter(name_ + ".wall_clearance_alpha", wall_clearance_alpha_);
  node->get_parameter(name_ + ".wall_clearance_lambda", wall_clearance_lambda_);
  node->get_parameter(name_ + ".keep_right_alpha", keep_right_alpha_);

  people_sub_ = node->create_subscription<corridor_social_nav::msg::TrackedPeople>(
    "/tracked_people", rclcpp::SensorDataQoS(),
    std::bind(&PredictiveSocialLayer::peopleCallback, this, std::placeholders::_1));

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1).transient_local(),
    std::bind(&PredictiveSocialLayer::pathCallback, this, std::placeholders::_1));

  current_ = true;
  enabled_ = true;
}

void PredictiveSocialLayer::peopleCallback(
  const corridor_social_nav::msg::TrackedPeople::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_ = msg;
}

void PredictiveSocialLayer::pathCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  corridor_frame_.updatePath(*msg);
}

void PredictiveSocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  if (!latest_people_) { return; }

  double expand = prediction_horizon_ * 2.0 + 2.0;
  for (const auto & person : latest_people_->people) {
    *min_x = std::min(*min_x, person.position.x - expand);
    *min_y = std::min(*min_y, person.position.y - expand);
    *max_x = std::max(*max_x, person.position.x + expand);
    *max_y = std::max(*max_y, person.position.y + expand);
  }
}

void PredictiveSocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) { return; }

  std::lock_guard<std::mutex> lock(people_mutex_);
  if (!latest_people_) { return; }

  std::lock_guard<std::mutex> flock(frame_mutex_);

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);

      double max_cell_cost = 0.0;

      for (const auto & person : latest_people_->people) {
        double px = person.position.x;
        double py = person.position.y;
        double pvx = person.velocity.x;
        double pvy = person.velocity.y;

        double person_cost = 0.0;

        if (use_corridor_frame_ && corridor_frame_.isValid()) {
          FrenetState fs = corridor_frame_.toFrenetState(px, py, pvx, pvy);
          FrenetPoint cell_fp = corridor_frame_.toFrenet(wx, wy);

          for (double tau = 0.0; tau <= prediction_horizon_; tau += prediction_step_) {
            double pred_s = fs.s + fs.ds * tau;
            double pred_d = fs.d + fs.dd * tau;
            double sig_s = sigma_s_0_ + k_s_ * tau;
            double sig_d = sigma_d_0_ + k_d_ * tau;

            if (anisotropic_ && std::abs(fs.ds) > 0.1) {
              sig_s *= 1.5;
            }

            double ds = cell_fp.s - pred_s;
            double dd = cell_fp.d - pred_d;
            double exponent = (ds * ds) / (2.0 * sig_s * sig_s) +
                              (dd * dd) / (2.0 * sig_d * sig_d);
            double cost = std::exp(-exponent);
            person_cost = std::max(person_cost, cost);
          }
        } else {
          for (double tau = 0.0; tau <= prediction_horizon_; tau += prediction_step_) {
            double pred_x = px + pvx * tau;
            double pred_y = py + pvy * tau;
            double sig = sigma_s_0_ + k_s_ * tau;

            double dx = wx - pred_x;
            double dy = wy - pred_y;
            double cost = std::exp(-(dx * dx + dy * dy) / (2.0 * sig * sig));
            person_cost = std::max(person_cost, cost);
          }
        }

        max_cell_cost = std::max(max_cell_cost, person_cost);
      }

      double final_cost = std::min(max_cell_cost * max_cost_, max_cost_);
      unsigned char grid_cost = static_cast<unsigned char>(final_cost);
      unsigned char old_cost = master_grid.getCost(i, j);
      if (grid_cost > old_cost) {
        master_grid.setCost(i, j, grid_cost);
      }
    }
  }
}

void PredictiveSocialLayer::reset()
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_.reset();
}

}  // namespace corridor_social_nav

PLUGINLIB_EXPORT_CLASS(corridor_social_nav::PredictiveSocialLayer, nav2_costmap_2d::Layer)
