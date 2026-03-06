#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "corridor_social_nav/msg/tracked_people.hpp"
#include <cmath>
#include <algorithm>

namespace corridor_social_nav
{

class SafetyShieldNode : public rclcpp::Node
{
public:
  SafetyShieldNode()
  : Node("safety_shield")
  {
    declare_parameter("stopping_a", 0.35);
    declare_parameter("stopping_b", 0.05);
    declare_parameter("stopping_c", 0.01);
    declare_parameter("safety_margin", 0.3);
    declare_parameter("cbf_alpha", 1.0);
    declare_parameter("v_max", 1.0);
    declare_parameter("delta_max", 0.4887);
    declare_parameter("intervention_threshold", 0.05);

    get_parameter("stopping_a", stopping_a_);
    get_parameter("stopping_b", stopping_b_);
    get_parameter("stopping_c", stopping_c_);
    get_parameter("safety_margin", safety_margin_);
    get_parameter("cbf_alpha", cbf_alpha_);
    get_parameter("v_max", v_max_);
    get_parameter("delta_max", delta_max_);
    get_parameter("intervention_threshold", intervention_threshold_);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        processCommand(msg);
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = msg->twist.twist.linear.x;
      });

    people_sub_ = create_subscription<corridor_social_nav::msg::TrackedPeople>(
      "/tracked_people", rclcpp::SensorDataQoS(),
      [this](const corridor_social_nav::msg::TrackedPeople::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(people_mutex_);
        latest_people_ = msg;
      });

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel_safe", 10);

    RCLCPP_INFO(get_logger(),
      "Safety shield initialized: s_stop = %.2fv^2 + %.2fv + %.2f, margin=%.2f",
      stopping_a_, stopping_b_, stopping_c_, safety_margin_);
  }

private:
  double stoppingDistance(double v) const
  {
    return stopping_a_ * v * v + stopping_b_ * std::abs(v) + stopping_c_;
  }

  double minClearance()
  {
    std::lock_guard<std::mutex> lock(people_mutex_);
    if (!latest_people_ || latest_people_->people.empty()) {
      return 100.0;
    }

    double min_dist = 100.0;
    for (const auto & person : latest_people_->people) {
      double dx = person.position.x - robot_x_;
      double dy = person.position.y - robot_y_;
      double dist = std::hypot(dx, dy);
      min_dist = std::min(min_dist, dist);
    }
    return min_dist;
  }

  void processCommand(const geometry_msgs::msg::TwistStamped::SharedPtr cmd)
  {
    double v_nom = cmd->twist.linear.x;
    double w_nom = cmd->twist.angular.z;

    double d_clear = minClearance();
    double s_stop = stoppingDistance(std::abs(current_speed_));
    double h = d_clear - s_stop - safety_margin_;

    double v_safe = v_nom;
    double w_safe = w_nom;

    if (h < 0.0) {
      v_safe = 0.0;
      w_safe = 0.0;
      intervention_count_++;
    } else if (h < safety_margin_) {
      double scale = h / safety_margin_;
      v_safe = v_nom * scale;
      w_safe = w_nom * scale;
      if (std::abs(v_nom - v_safe) > intervention_threshold_) {
        intervention_count_++;
      }
    }

    v_safe = std::clamp(v_safe, 0.0, v_max_);
    total_count_++;

    auto safe_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    safe_msg->header = cmd->header;
    safe_msg->twist.linear.x = v_safe;
    safe_msg->twist.angular.z = w_safe;
    safe_cmd_pub_->publish(std::move(safe_msg));
  }

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<corridor_social_nav::msg::TrackedPeople>::SharedPtr people_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_pub_;

  corridor_social_nav::msg::TrackedPeople::SharedPtr latest_people_;
  std::mutex people_mutex_;

  double current_speed_{0.0};
  double robot_x_{0.0}, robot_y_{0.0};

  double stopping_a_, stopping_b_, stopping_c_;
  double safety_margin_, cbf_alpha_;
  double v_max_, delta_max_;
  double intervention_threshold_;

  uint64_t intervention_count_{0};
  uint64_t total_count_{0};
};

}  // namespace corridor_social_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<corridor_social_nav::SafetyShieldNode>());
  rclcpp::shutdown();
  return 0;
}
