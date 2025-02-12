#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <functional>
#include <memory>
#include <ranges>
#include <vector>

using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service") {
    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::direction_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void
  direction_callback(const std::shared_ptr<GetDirection::Request> request,
                     const std::shared_ptr<GetDirection::Response> response) {
    auto laser_data = request->laser_data;

    std::vector<float> laser_data_ranges = laser_data.ranges;

    int range_total = laser_data.ranges.size();
    RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);

    // Define the section boundaries
    std::vector<std::pair<int, int>> section_boundaries = {
        {165, 247}, {248, 412}, {413, 495} // last region ends at 495
    };

    // Define the section names
    std::vector<std::string> section_names = {"right", "front", "left"};

    // Calculate the total distance for each section
    float total_dist_sec_right = 0.0f;
    float total_dist_sec_front = 0.0f;
    float total_dist_sec_left = 0.0f;

    // Process each section and calculate the total distance
    auto subrange_right = std::ranges::subrange(
        laser_data_ranges.begin() + section_boundaries[0].first,
        laser_data_ranges.begin() + section_boundaries[0].second);
    total_dist_sec_right =
        std::accumulate(subrange_right.begin(), subrange_right.end(), 0.0f);

    auto subrange_front = std::ranges::subrange(
        laser_data_ranges.begin() + section_boundaries[1].first,
        laser_data_ranges.begin() + section_boundaries[1].second);
    total_dist_sec_front =
        std::accumulate(subrange_front.begin(), subrange_front.end(), 0.0f);

    auto subrange_left = std::ranges::subrange(
        laser_data_ranges.begin() + section_boundaries[2].first,
        laser_data_ranges.begin() + section_boundaries[2].second);
    total_dist_sec_left =
        std::accumulate(subrange_left.begin(), subrange_left.end(), 0.0f);

    // Output the total distances for each section
    RCLCPP_INFO(this->get_logger(), "Total distance in right section: %.2f",
                total_dist_sec_right);
    RCLCPP_INFO(this->get_logger(), "Total distance in front section: %.2f",
                total_dist_sec_front);
    RCLCPP_INFO(this->get_logger(), "Total distance in left section: %.2f",
                total_dist_sec_left);

    // Determine the safest direction (section with the largest total distance)
    if (total_dist_sec_right >= total_dist_sec_front &&
        total_dist_sec_right >= total_dist_sec_left) {
      RCLCPP_INFO(this->get_logger(),
                  "Safest direction: Right with total distance: %.2f",
                  total_dist_sec_right);
      response->direction = "right";
    } else if (total_dist_sec_front >= total_dist_sec_right &&
               total_dist_sec_front >= total_dist_sec_left) {
      RCLCPP_INFO(this->get_logger(),
                  "Safest direction: Front with total distance: %.2f",
                  total_dist_sec_front);
      response->direction = "forward";
    } else if (total_dist_sec_left >= total_dist_sec_right &&
               total_dist_sec_left >= total_dist_sec_front) {
      RCLCPP_INFO(this->get_logger(),
                  "Safest direction: Left with total distance: %.2f",
                  total_dist_sec_left);
      response->direction = "left";
    } else {
      RCLCPP_INFO(this->get_logger(), "No valid direction found.");
      response->direction = "None";
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}