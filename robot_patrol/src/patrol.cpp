#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int size = static_cast<int>(msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "Laser array size is: %d", size);
    RCLCPP_INFO(this->get_logger(), "Laser Scan Value[Front]: %.2f",
                msg->ranges[330]);
    RCLCPP_INFO(this->get_logger(), "Laser Scan Value[Right]: %.2f",
                msg->ranges[0]);
    RCLCPP_INFO(this->get_logger(), "Laser Scan Value[Left]: %.2f",
                msg->ranges[659]);
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> node = std::make_shared<Patrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}