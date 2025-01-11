#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>

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
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // The angle must be between −𝜋/2 and 𝜋/2
    const float min_angle = -M_PI / 2.0;
    const float max_angle = M_PI / 2.0;
    float largest_distance = 0.0;
    float largest_distance_angle = 0.0;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      float angle = msg->angle_min + i * msg->angle_increment;
      if (angle >= min_angle && angle <= max_angle) {
        float dist = msg->ranges[i];
        if (!std::isinf(dist) && dist > largest_distance) {
          largest_distance = dist;
          largest_distance_angle = angle;
        }
      }
    }
    direction_ = largest_distance_angle;
    RCLCPP_INFO(this->get_logger(), "The Largest distance is %.2f",
                largest_distance);
    RCLCPP_INFO(this->get_logger(), "The Safest direction is %.2f", direction_);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group;
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