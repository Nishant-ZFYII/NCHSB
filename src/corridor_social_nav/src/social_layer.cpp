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

  declareParameter("gaussian_sigma", rclcpp::ParameterValue(0.45));
  declareParameter("amplitude", rclcpp::ParameterValue(160.0));
  declareParameter("cutoff_distance", rclcpp::ParameterValue(1.2));
  declareParameter("stale_timeout", rclcpp::ParameterValue(0.5));

  node->get_parameter(name_ + ".gaussian_sigma", gaussian_sigma_);
  node->get_parameter(name_ + ".amplitude", amplitude_);
  node->get_parameter(name_ + ".cutoff_distance", cutoff_distance_);
  node->get_parameter(name_ + ".stale_timeout", stale_timeout_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  people_sub_ = node->create_subscription<corridor_social_nav::msg::TrackedPeople>(
    "/tracked_people", rclcpp::SensorDataQoS(),
    std::bind(&SocialLayer::peopleCallback, this, std::placeholders::_1));

  last_people_time_ = node->now();
  current_ = true;
  enabled_ = true;
  matchSize();

  RCLCPP_INFO(node->get_logger(),
    "SocialLayer: amp=%.0f sigma=%.2f cutoff=%.1f global_frame=%s",
    amplitude_, gaussian_sigma_, cutoff_distance_, global_frame_.c_str());
}

void SocialLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(),
    master->getOriginX(), master->getOriginY());
}

void SocialLayer::peopleCallback(
  const corridor_social_nav::msg::TrackedPeople::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_ = msg;
  auto node = node_.lock();
  if (node) {
    last_people_time_ = node->now();
  }
}

bool SocialLayer::transformPeopleToGlobal()
{
  transformed_people_.clear();
  if (!latest_people_ || latest_people_->people.empty()) {
    return false;
  }

  std::string source_frame = latest_people_->header.frame_id;
  if (source_frame.empty()) {
    source_frame = "map";
  }

  // If already in the costmap's global frame, no transform needed
  if (source_frame == global_frame_) {
    for (const auto & person : latest_people_->people) {
      transformed_people_.push_back({person.position.x, person.position.y});
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
        "SocialLayer: cannot transform %s -> %s: %s",
        source_frame.c_str(), global_frame_.c_str(), ex.what());
    }
    return false;
  }

  for (const auto & person : latest_people_->people) {
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header = latest_people_->header;
    pt_in.point = person.position;
    tf2::doTransform(pt_in, pt_out, tf_stamped);
    transformed_people_.push_back({pt_out.point.x, pt_out.point.y});
  }
  return true;
}

void SocialLayer::updateBounds(
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

  // Transform people into costmap's global frame
  transformPeopleToGlobal();

  double cur_min_x = 1e6, cur_min_y = 1e6;
  double cur_max_x = -1e6, cur_max_y = -1e6;

  if (!transformed_people_.empty()) {
    for (const auto & tp : transformed_people_) {
      cur_min_x = std::min(cur_min_x, tp.x - cutoff_distance_);
      cur_min_y = std::min(cur_min_y, tp.y - cutoff_distance_);
      cur_max_x = std::max(cur_max_x, tp.x + cutoff_distance_);
      cur_max_y = std::max(cur_max_y, tp.y + cutoff_distance_);
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

void SocialLayer::updateCosts(
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

  // Stamp Gaussians using the already-transformed positions
  if (!transformed_people_.empty()) {
    double inv_2sig2 = 1.0 / (2.0 * gaussian_sigma_ * gaussian_sigma_);

    for (const auto & tp : transformed_people_) {
      for (int j = min_j; j < max_j; ++j) {
        for (int i = min_i; i < max_i; ++i) {
          double wx, wy;
          master_grid.mapToWorld(i, j, wx, wy);

          double dx = wx - tp.x;
          double dy = wy - tp.y;
          double dist_sq = dx * dx + dy * dy;

          if (dist_sq > cutoff_distance_ * cutoff_distance_) { continue; }

          double cost = amplitude_ * std::exp(-dist_sq * inv_2sig2);
          unsigned char grid_cost = static_cast<unsigned char>(
            std::min(cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE - 1)));

          unsigned char old_cost = getCost(i, j);
          if (grid_cost > old_cost) {
            setCost(i, j, grid_cost);
          }
        }
      }
    }
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

void SocialLayer::reset()
{
  std::lock_guard<std::mutex> lock(people_mutex_);
  latest_people_.reset();
  transformed_people_.clear();
  has_prev_bounds_ = false;
  resetMaps();
}

}  // namespace corridor_social_nav

PLUGINLIB_EXPORT_CLASS(corridor_social_nav::SocialLayer, nav2_costmap_2d::Layer)
