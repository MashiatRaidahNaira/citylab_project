#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class GoToPose : public rclcpp::Node {
public:
  using GoToPose_i = custom_interfaces::action::GoToPose;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPose_i>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPose_i>(
        this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    odom1_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom1_callback_group_;

    subscription1_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&GoToPose::odom_callback, this, _1), options1);

    RCLCPP_INFO(this->get_logger(), "GoToPose Action Server initialized!");
  }

private:
  rclcpp_action::Server<GoToPose_i>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::CallbackGroup::SharedPtr odom1_callback_group_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription1_;

  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose_i::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    desired_pos_ = goal->goal_pos;
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // Converting quaternion to Euler angles
    auto q = msg->pose.pose.orientation;
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    current_pos_.theta = std::atan2(siny_cosp, cosy_cosp);
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose_i::Feedback>();
    auto &message = feedback->current_pos;
    auto result = std::make_shared<GoToPose_i::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto go_to_pose_action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(go_to_pose_action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}