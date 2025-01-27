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

    // Call take_action with the regions
    take_action(regions);

    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::response_callback, this, std::placeholders::_1));
  }

  void take_action(const std::map<std::string, float> &regions) {
    geometry_msgs::msg::Twist twist_msg;

    float linear_x = 0.0;
    float angular_z = 0.0;

    // Determine the action based on region conditions
    if (regions.at("front") > 0.35) {
      RCLCPP_INFO(this->get_logger(), "Service response: Move forward");
      linear_x = 0.1;
      angular_z = 0.0;
    } else if (regions.at("left") < 0.35) {
      RCLCPP_INFO(this->get_logger(), "Service response: Turn left");
      linear_x = 0.1;
      angular_z = 0.5;
      if (regions.at("front") < 0.35 || regions.at("left") < 0.35 ||
          regions.at("right") < 0.35) {
        linear_x = 0.0;
        angular_z = 0.5;
      }
    } else if (regions.at("right") < 0.35) {
      RCLCPP_INFO(this->get_logger(), "Service response: Turn right");
      linear_x = 0.1;
      angular_z = -0.5;
      if (regions.at("front") < 0.35 || regions.at("left") < 0.35 ||
          regions.at("right") < 0.35) {
        linear_x = 0.0;
        angular_z = 0.5;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service response: Unknown case");
      if (regions.at("front") < 0.35 || regions.at("left") < 0.35 ||
          regions.at("right") < 0.35) {
        linear_x = 0.0;
        angular_z = 0.4;
      }
    }

    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;
    // Publish the Twist message
    publisher_->publish(twist_msg);

    // Log the action taken
    RCLCPP_INFO(this->get_logger(), "Linear velocity: %f", twist_msg.linear.x);
    RCLCPP_INFO(this->get_logger(), "Angular velocity: %f",
                twist_msg.angular.z);
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    geometry_msgs::msg::Twist twist_msg;

    // Now check for the response after a timeout of 1 second
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      service_done_ = true;
      RCLCPP_INFO(this->get_logger(), "Response: %s",
                  response->direction.c_str());
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Response not ready yet. Waiting until ready...");
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