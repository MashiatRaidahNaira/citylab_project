#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <cmath>

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

    client = this->create_client<GetDirection>("direction_service");
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
    auto result_future = client_->async_send_request(
        request,
        std::bind(&Patrol::response_callback, this, std::placeholders::_1));
  }

  void response_callback(rclcpp::Client<GetDirection>::SharedFuture future) {
    geometry_msgs::msg::Twist twist_msg;

    // Now check for the response after a timeout of 1 second
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      service_done_ = true;

      if (response->direction == "Move forward") {
        RCLCPP_INFO(this->get_logger(), "Service returned Front");
        twist_msg.linear.x = 0.1;
        twist_msg.angular.z = 0.0;
      } else if (response->direction == "Turn right") {
        RCLCPP_INFO(this->get_logger(), "Service returned Right");
        twist_msg.linear.x = 0.1;
        twist_msg.angular.z = -0.5;
      } else if (response->direction == "Turn left") {
        RCLCPP_INFO(this->get_logger(), "Service returned Left");
        twist_msg.linear.x = 0.1;
        twist_msg.angular.z = 0.5;
      } else {
        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
      }

      publisher_->publish(twist_msg);

      RCLCPP_INFO(this->get_logger(), "Linear velocity is %f",
                  twist_msg.linear.x);
      RCLCPP_INFO(this->get_logger(), "Angular velocity is %f",
                  twist_msg.linear.z);

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