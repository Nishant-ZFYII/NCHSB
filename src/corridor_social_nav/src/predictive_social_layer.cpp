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
  declareParameter("max_cost", rclcpp::ParameterValue(160.0));
  declareParameter("anisotropic", rclcpp::ParameterValue(true));
  declareParameter("use_corridor_frame", rclcpp::ParameterValue(true));
  declareParameter("wall_clearance_alpha", rclcpp::ParameterValue(50.0));
  declareParameter("wall_clearance_lambda", rclcpp::ParameterValue(0.3));
  declareParameter("keep_right_alpha", rclcpp::ParameterValue(10.0));
  declareParameter("stale_timeout", rclcpp::ParameterValue(0.5));

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
  node->get_parameter(name_ + ".stale_timeout", stale_timeout_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  people_sub_ = node->create_subscription<corridor_social_nav::msg::TrackedPeople>(
    "/tracked_people", rclcpp::SensorDataQoS(),
    std::bind(&PredictiveSocialLayer::peopleCallback, this, std::placeholders::_1));

  path_sub_ = node->create_subscription<nav_msgs::msg::Path>(
    "/plan", rclcpp::QoS(1).transient_local(),
    std::bind(&PredictiveSocialLayer::pathCallback, this, std::placeholders::_1));

  last_people_time_ = node->now();
  current_ = true;
  enabled_ = true;
  matchSize();

  RCLCPP_INFO(node->get_logger(),
    "PredictiveSocialLayer: max_cost=%.0f horizon=%.1f global_frame=%s",
    max_cost_, prediction_horizon_, global_frame_.c_str());
}

void PredictiveSocialLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void PredictiveSocialLayer::peopleCallback(
  const corridor_social_nav::msg::TrackedPeople::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_ = msg;
  auto node = node_.lock();
  if (node) {
    last_people_time_ = node->now();
  }
}

void PredictiveSocialLayer::pathCallback(
  const nav_msgs::msg::Path::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  corridor_frame_.updatePath(*msg);
}

bool PredictiveSocialLayer::transformPeopleToGlobal()
{
  transformed_people_.clear();
  if (!latest_people_ || latest_people_->people.empty()) {
    return false;
  }

  std::string source_frame = latest_people_->header.frame_id;
  if (source_frame.empty()) {
    source_frame = "map";
  }

  if (source_frame == global_frame_) {
    for (const auto & person : latest_people_->people) {
      transformed_people_.push_back(
        {person.position.x, person.position.y,
         person.velocity.x, person.velocity.y});
    }
    return true;
  }

  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      global_frame_, source_frame, tf2::TimePointZero,
      tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    auto node = node_.lock();
    if (node) {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 2000,
        "PredictiveSocialLayer: cannot transform %s -> %s: %s",
        source_frame.c_str(), global_frame_.c_str(), ex.what());
    }
    return false;
  }

  for (const auto & person : latest_people_->people) {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header = latest_people_->header;
    pt_in.point = person.position;
    tf2::doTransform(pt_in, pt_out, tf_stamped);

    // Transform velocity as a vector (rotation only, no translation)
    geometry_msgs::msg::Vector3Stamped vel_in, vel_out;
    vel_in.header = latest_people_->header;
    vel_in.vector = person.velocity;
    tf2::doTransform(vel_in, vel_out, tf_stamped);

    transformed_people_.push_back(
      {pt_out.point.x, pt_out.point.y,
       vel_out.vector.x, vel_out.vector.y});
  }
  return true;
}

void PredictiveSocialLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y,
  double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(people_mutex_);

  if (has_prev_bounds_) {
    *min_x = std::min(*min_x, prev_min_x_);
    *min_y = std::min(*min_y, prev_min_y_);
    *max_x = std::max(*max_x, prev_max_x_);
    *max_y = std::max(*max_y, prev_max_y_);
  }

  auto node = node_.lock();
  if (node) {
    double age = (node->now() - last_people_time_).seconds();
    if (age > stale_timeout_) {
      latest_people_.reset();
      transformed_people_.clear();
    }
  }

  transformPeopleToGlobal();

  double cur_min_x = 1e6, cur_min_y = 1e6;
  double cur_max_x = -1e6, cur_max_y = -1e6;

  if (!transformed_people_.empty()) {
    double expand = prediction_horizon_ * 2.0 + 2.0;
    for (const auto & tp : transformed_people_) {
      cur_min_x = std::min(cur_min_x, tp.x - expand);
      cur_min_y = std::min(cur_min_y, tp.y - expand);
      cur_max_x = std::max(cur_max_x, tp.x + expand);
      cur_max_y = std::max(cur_max_y, tp.y + expand);
    }

    *min_x = std::min(*min_x, cur_min_x);
    *min_y = std::min(*min_y, cur_min_y);
    *max_x = std::max(*max_x, cur_max_x);
    *max_y = std::max(*max_y, cur_max_y);
  }

  prev_min_x_ = cur_min_x;
  prev_min_y_ = cur_min_y;
  prev_max_x_ = cur_max_x;
  prev_max_y_ = cur_max_y;
  has_prev_bounds_ = !transformed_people_.empty();
}

void PredictiveSocialLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) { return; }

  if (getSizeInCellsX() != master_grid.getSizeInCellsX() ||
      getSizeInCellsY() != master_grid.getSizeInCellsY()) {
    matchSize();
  }

  // Clear internal layer in update region
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      setCost(i, j, nav2_costmap_2d::FREE_SPACE);
    }
  }

  // Stamp predicted Gaussians using transformed positions
  if (transformed_people_.empty()) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    return;
  }

  {
    std::lock_guard<std::mutex> flock(frame_mutex_);

    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master_grid.mapToWorld(i, j, wx, wy);

        double max_cell_cost = 0.0;

        for (const auto & tp : transformed_people_) {
          double person_cost = 0.0;

          if (use_corridor_frame_ && corridor_frame_.isValid()) {
            FrenetState fs = corridor_frame_.toFrenetState(tp.x, tp.y, tp.vx, tp.vy);
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
              double pred_x = tp.x + tp.vx * tau;
              double pred_y = tp.y + tp.vy * tau;
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
        unsigned char old_cost = getCost(i, j);
        if (grid_cost > old_cost) {
          setCost(i, j, grid_cost);
        }
      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void PredictiveSocialLayer::reset()
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_.reset();
  transformed_people_.clear();
  has_prev_bounds_ = false;
  resetMaps();
}

}  // namespace corridor_social_nav

PLUGINLIB_EXPORT_CLASS(corridor_social_nav::PredictiveSocialLayer, nav2_costmap_2d::Layer)
