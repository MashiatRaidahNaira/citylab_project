#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection;
using LaserScan = sensor_msgs::msg::LaserScan;
using std::placeholders::_1;

class TestServerClient : public rclcpp::Node {
private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  bool service_done_ = false;

  void laser_callback(const LaserScan::SharedPtr msg) {
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;
    auto result_future = client_->async_send_request(
        request, std::bind(&TestServerClient::response_callback, this, _1));
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    // Get response value
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Response: %s",
                response->direction.c_str());
    service_done_ = true;
  }

public:
  TestServerClient() : Node("Test_server_client") {
    client_ = this->create_client<GetDirection>("direction_service");
    subscription_ = this->create_subscription<LaserScan>(
        "scan", 1, std::bind(&TestServerClient::laser_callback, this, _1));

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
  }

  bool is_service_done() const { return this->service_done_; }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto test_server_client = std::make_shared<TestServerClient>();
  while (!test_server_client->is_service_done()) {
    rclcpp::spin_some(test_server_client);
  }

  rclcpp::shutdown();
  return 0;
}