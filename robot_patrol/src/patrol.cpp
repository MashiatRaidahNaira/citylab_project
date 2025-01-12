#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    scan_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = scan_callback_group;
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
        options1);

    timer_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // The angle must be between −𝜋/2 and 𝜋/2
    const float min_angle = -M_PI / 2.0;
    const float max_angle = M_PI / 2.0;
    float largest_distance = 0.0;
    float largest_distance_angle = 0.0;
    float closest_distance = std::numeric_limits<float>::max();
    float closest_distance_angle = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      float angle = msg->angle_min + i * msg->angle_increment;
      if (angle >= min_angle && angle <= max_angle) {
        float dist = msg->ranges[i];
        if (!std::isinf(dist) && dist > largest_distance) {
          largest_distance = dist;
          largest_distance_angle = angle;
        } else if (!std::isinf(dist) && dist < closest_distance) {
          closest_distance = dist;
          closest_distance_angle = angle;
        }
      }
    }

    if (closest_distance <= 0.35) {
      direction_ = largest_distance_angle;
      if (closest_distance_angle >= 0.0) {
        direction_ -= 2.5;
      } else {
        direction_ += 2.5;
      }
    } else {
      direction_ = 0.0;
    }

    RCLCPP_INFO(this->get_logger(), "The Closest distance is %.2f",
                closest_distance);
    RCLCPP_INFO(this->get_logger(), "The Safest direction is %.2f", direction_);
  }

  void timer_callback() {
    geometry_msgs::msg::Twist twist_msg;

    twist_msg.linear.x = 0.1;

    twist_msg.angular.z = direction_ / 2;

    publisher_->publish(twist_msg);

    RCLCPP_INFO(this->get_logger(), "Linear velocity is %f",
                twist_msg.linear.x);
    RCLCPP_INFO(this->get_logger(), "Angular velocity is %f",
                twist_msg.linear.z);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group;
  rclcpp::TimerBase::SharedPtr timer_;
  float direction_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> node = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}