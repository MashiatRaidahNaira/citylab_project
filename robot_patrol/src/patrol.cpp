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
    size_t closest_index;

    /*# Calculate the index for the specific angle (0 degrees in this case)
    specific_angle = 0.0  # Change this value if you're looking for a different
    angle*/
    // index = int((specific_angle - angle_min) / angle_increment)
    // So, -15 degrees, index = (-15-(-180)) / 0.5 = 330 [All radian values
    // converted to degrees], & 15 degrees, index = (15-(-180)) / 0.5 = 390

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
          if (i >= 330 && i <= 390) {
            closest_index = i;
          }
        }
      }
    }

    direction_ = largest_distance_angle;
    twist_msg = this->angularSpeed(closest_distance, direction_,
                                   closest_distance_angle, closest_index);

    RCLCPP_INFO(this->get_logger(), "The Closest distance is %.2f",
                closest_distance);
    RCLCPP_INFO(this->get_logger(), "The closest distance angle is %.2f",
                closest_distance_angle);
    RCLCPP_INFO(this->get_logger(), "The closest index is %.2ld",
                closest_index);
    RCLCPP_INFO(this->get_logger(), "The Safest direction is %.2f", direction_);
  }

  geometry_msgs::msg::Twist angularSpeed(float closest_distance,
                                         float direction_,
                                         float closest_distance_angle,
                                         size_t closest_index) {

    auto msg = geometry_msgs::msg::Twist();

    if (closest_distance <= 0.35) {
      if ((closest_distance_angle >= 0.0) ||
          ((closest_index > 360) && (closest_index <= 390))) {
        msg.linear.x = 0.1;
        direction_ = -1.4;
        msg.angular.z = direction_ / 2;
        if (closest_distance < 0.30) {
          msg.linear.x = 0.0;
          msg.angular.z = direction_ / 2;
        }
      } else if ((closest_distance_angle < 0.0) ||
                 ((closest_index >= 330) && (closest_index < 360))) {
        msg.linear.x = 0.1;
        direction_ = 1.4;
        msg.angular.z = direction_ / 2;
        if (closest_distance < 0.30) {
          msg.linear.x = 0.0;
          msg.angular.z = direction_ / 2;
        }
      }
    } else {
      msg.linear.x = 0.1;
      direction_ = 0.0;
      msg.linear.z = direction_ / 2;
    }

    return msg;
  }

  void timer_callback() {
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
  geometry_msgs::msg::Twist twist_msg;
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