#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <map>
#include <memory>
#include <ranges>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_service_node") {
    scan_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = scan_callback_group;
    subscription_ = this->create_subscription<LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
        options1);

    timer_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::timer_callback, this), timer_callback_group);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    client_ = this->create_client<GetDirection>("direction_service");
  }

  bool is_service_done() const { return this->service_done_; }

private:
  void laser_callback(const LaserScan::SharedPtr msg) { last_laser_ = msg; }

  void timer_callback() {
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *last_laser_;

    auto laser_data = request->laser_data;

    std::vector<float> laser_data_ranges = laser_data.ranges;

    // Define a map to store the regions and their minimum values
    std::map<std::string, float> regions;

    // Define the region boundaries
    std::vector<std::pair<int, int>> region_boundaries = {
        {0, 220}, {220, 440}, {440, 660} // last region ends at 659
    };

    // Define the region names
    std::vector<std::string> region_names = {"right", "front", "left"};

    // Process each region and find the minimum value, ensuring it's capped at
    // 10
    for (size_t i = 0; i < region_boundaries.size(); ++i) {
      auto [start, end] = region_boundaries[i];
      auto subrange = std::ranges::subrange(laser_data_ranges.begin() + start,
                                            laser_data_ranges.begin() + end);
      // Use std::ranges::min_element to find the minimum value in the subrange
      // and cap it at 10
      regions[region_names[i]] =
          std::min(*std::ranges::min_element(subrange), 10.0f);
    }

    // Use a lambda to bind both the response callback and regions
    auto result_future = client_->async_send_request(
        request,
        [this, regions](rclcpp::Client<GetDirection>::SharedFuture future) {
          this->response_callback(future, regions);
        });
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future,
                         const std::map<std::string, float> &regions) {
    geometry_msgs::msg::Twist twist_msg;

    // Now check for the response after a timeout of 1 second
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      service_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Response: %s",
                  response->direction.c_str());

      float linear_x = 0.0;
      float angular_z = 0.0;
      static int stuck_counter = 0; // Track how long it's stuck

      // Check obstacle conditions
      bool front_blocked = regions.at("front") < 0.35;
      bool left_blocked = regions.at("left") < 0.32;
      bool right_blocked = regions.at("right") < 0.32;
      bool stuck =
          (front_blocked && left_blocked) || (front_blocked && right_blocked);

      if (stuck) {
        stuck_counter++;

        if (stuck_counter >= 3) { // Applying escape maneuver
          RCLCPP_WARN(this->get_logger(),
                      "Stuck detected! Forcing a strong right turn.");
          linear_x = 0.0;
          angular_z = -1.0;  // Force strong turn to break loop
          stuck_counter = 0; // Reset counter
        }
      } else {
        stuck_counter = 0; // Reset if no longer stuck
      }

      // First, process the direction received from the service
      if (response->direction == "Move forward" || regions.at("front") > 0.35) {
        RCLCPP_INFO(this->get_logger(), "Service response: Moving forward");
        linear_x = 0.1;
        angular_z = 0.0;

        // Adjust based on obstacle conditions
        if (front_blocked) {
          linear_x = 0.0;
          angular_z = -0.5; // Default to turning right
          RCLCPP_WARN(this->get_logger(), "Obstacle in front, turning right.");
        }
      } else if (response->direction == "Turn left" ||
                 regions.at("left") > 0.35) {
        RCLCPP_INFO(this->get_logger(), "Service response: Turning left.");
        linear_x = 0.1;
        angular_z = 0.5;

        // Adjust based on obstacle conditions
        if (left_blocked) {
          angular_z = -0.5; // If left is blocked, turn right
          RCLCPP_WARN(this->get_logger(), "Obstacle on left, turning right.");
        }
        if (front_blocked) {
          linear_x = 0.0;

          if (right_blocked) {
            angular_z = 0.5; // Keep turning left if right is also blocked
            RCLCPP_WARN(this->get_logger(),
                        "Obstacles in front and right, continuing left turn.");
          } else {
            angular_z = -0.5; // Otherwise, turn right
            RCLCPP_WARN(this->get_logger(),
                        "Obstacle in front, turning right instead.");
          }
        }
      } else if (response->direction == "Turn right" ||
                 regions.at("right") > 0.35) {
        RCLCPP_INFO(this->get_logger(), "Service response: Turning right.");
        linear_x = 0.1;
        angular_z = -0.5;

        // Adjust based on obstacle conditions
        if (right_blocked) {
          angular_z = 0.5; // If right is blocked, turn left
          RCLCPP_WARN(this->get_logger(), "Obstacle on right, turning left.");
        }
        if (front_blocked) {
          linear_x = 0.0;

          if (left_blocked) {
            angular_z = -0.5; // Keep turning right if left is also blocked
            RCLCPP_WARN(this->get_logger(),
                        "Obstacles in front and left, continuing right turn.");
          } else {
            angular_z = 0.5; // Otherwise, turn left
            RCLCPP_WARN(this->get_logger(),
                        "Obstacle in front, turning left instead.");
          }
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "Unknown direction: %s",
                    response->direction.c_str());
        linear_x = 0.0;
        angular_z = 0.4;
      }

      // Publish movement command
      twist_msg.linear.x = linear_x;
      twist_msg.angular.z = angular_z;
      publisher_->publish(twist_msg);

      RCLCPP_INFO(this->get_logger(), "Linear velocity: %f",
                  twist_msg.linear.x);
      RCLCPP_INFO(this->get_logger(), "Angular velocity: %f",
                  twist_msg.angular.z);
    } else {
      RCLCPP_WARN(this->get_logger(), "Response not ready yet. Waiting...");
    }
  }

  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr timer_callback_group;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<GetDirection>::SharedPtr client_;
  LaserScan::SharedPtr last_laser_;
  bool service_done_ = false;
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