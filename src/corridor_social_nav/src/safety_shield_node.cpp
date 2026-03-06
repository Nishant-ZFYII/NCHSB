#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include "corridor_social_nav/msg/tracked_people.hpp"
#include "corridor_social_nav/safety_shield_cbf.hpp"
#include <mutex>

namespace corridor_social_nav
{

class SafetyShieldNode : public rclcpp::Node
{
public:
  SafetyShieldNode()
  : Node("safety_shield")
  {
    declare_parameter("robot_radius", 0.20);
    declare_parameter("person_radius", 0.25);
    declare_parameter("safety_margin", 0.30);
    declare_parameter("alpha", 1.0);
    declare_parameter("v_max", 1.0);
    declare_parameter("v_min", 0.0);
    declare_parameter("omega_max", 1.5);
    declare_parameter("stopping_a", 0.35);
    declare_parameter("stopping_b", 0.05);
    declare_parameter("stopping_c", 0.01);
    declare_parameter("intervention_threshold", 0.05);

    CBFParams p;
    p.robot_radius = get_parameter("robot_radius").as_double();
    p.person_radius = get_parameter("person_radius").as_double();
    p.safety_margin = get_parameter("safety_margin").as_double();
    p.alpha = get_parameter("alpha").as_double();
    p.v_max = get_parameter("v_max").as_double();
    p.v_min = get_parameter("v_min").as_double();
    p.omega_max = get_parameter("omega_max").as_double();
    p.stopping_a = get_parameter("stopping_a").as_double();
    p.stopping_b = get_parameter("stopping_b").as_double();
    p.stopping_c = get_parameter("stopping_c").as_double();
    p.intervention_threshold = get_parameter("intervention_threshold").as_double();

    cbf_ = std::make_unique<SafetyShieldCBF>(p);

    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        processCommand(msg);
      });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        robot_state_.v = msg->twist.twist.linear.x;
        robot_state_.x = msg->pose.pose.position.x;
        robot_state_.y = msg->pose.pose.position.y;
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        robot_state_.theta = 2.0 * std::atan2(qz, qw);
      });

    people_sub_ = create_subscription<corridor_social_nav::msg::TrackedPeople>(
      "/tracked_people", rclcpp::SensorDataQoS(),
      [this](const corridor_social_nav::msg::TrackedPeople::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(people_mutex_);
        latest_people_ = msg;
      });

    safe_cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_safe", 10);
    barrier_pub_ = create_publisher<std_msgs::msg::Float64>("/cbf/min_barrier", 10);
    intervention_pub_ = create_publisher<std_msgs::msg::Bool>("/cbf/intervention", 10);

    RCLCPP_INFO(get_logger(),
      "CBF safety shield: d_safe=%.2f, alpha=%.1f, d_stop=%.2fv^2+%.2fv+%.2f",
      cbf_->params().dSafe(), cbf_->params().alpha,
      cbf_->params().stopping_a, cbf_->params().stopping_b, cbf_->params().stopping_c);
  }

private:
  void processCommand(const geometry_msgs::msg::TwistStamped::SharedPtr cmd)
  {
    RobotState robot;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      robot = robot_state_;
    }

    std::vector<PersonState> people;
    {
      std::lock_guard<std::mutex> lock(people_mutex_);
      if (latest_people_) {
        for (const auto & p : latest_people_->people) {
          people.push_back({p.position.x, p.position.y, p.velocity.x, p.velocity.y});
        }
      }
    }

    auto result = cbf_->solve(robot, people, cmd->twist.linear.x, cmd->twist.angular.z);

    auto safe_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    safe_msg->header = cmd->header;
    safe_msg->twist.linear.x = result.v;
    safe_msg->twist.angular.z = result.omega;
    safe_cmd_pub_->publish(std::move(safe_msg));

    auto barrier_msg = std::make_unique<std_msgs::msg::Float64>();
    barrier_msg->data = result.min_barrier_value;
    barrier_pub_->publish(std::move(barrier_msg));

    auto interv_msg = std::make_unique<std_msgs::msg::Bool>();
    interv_msg->data = result.intervened;
    intervention_pub_->publish(std::move(interv_msg));

    if (result.intervened) {
      intervention_count_++;
    }
    total_count_++;

    if (total_count_ % 200 == 0) {
      double rate = (total_count_ > 0)
        ? 100.0 * static_cast<double>(intervention_count_) / static_cast<double>(total_count_)
        : 0.0;
      RCLCPP_INFO(get_logger(),
        "CBF stats: %lu/%lu interventions (%.1f%%), min_h=%.3f",
        intervention_count_, total_count_, rate, result.min_barrier_value);
    }
  }

  std::unique_ptr<SafetyShieldCBF> cbf_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<corridor_social_nav::msg::TrackedPeople>::SharedPtr people_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr barrier_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr intervention_pub_;

  corridor_social_nav::msg::TrackedPeople::SharedPtr latest_people_;
  std::mutex people_mutex_;

  RobotState robot_state_{};
  std::mutex state_mutex_;

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
