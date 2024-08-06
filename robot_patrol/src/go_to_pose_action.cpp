/* Implement Action Server of go_to_pose action*/

#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <robot_patrol/action/go_to_pose.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <thread>

using namespace std::chrono_literals;
using namespace std::placeholders;
class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;
  using Feedback = GoToPoseAction::Feedback;
  using Result = GoToPoseAction::Result;
  using ActionServer = rclcpp_action::Server<GoToPoseAction>;
  using OdomeMsg = nav_msgs::msg::Odometry;
  using TwistMsg = geometry_msgs::msg::Twist;
  using Goal = GoToPoseAction::Goal;
  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action", options) {

    this->action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "go_to_pose", std::bind(&GoToPose::goal_callback, this, _1, _2),
        std::bind(&GoToPose::cancel_callback, this, _1),
        std::bind(&GoToPose::accepted_callback, this, _1));

    sub_ = this->create_subscription<OdomeMsg>(
        "odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
    pub_ = this->create_publisher<TwistMsg>("cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Subscription<OdomeMsg>::SharedPtr sub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr pub_;

  geometry_msgs::msg::Pose2D
      current_pos_; // save current_position (float linear.x and angular.z)
  geometry_msgs::msg::Pose2D
      desired_pos_; // get the goal value from client (float x,y,theta)

  float distance_ = 0.0;
  float angle_ = 0.0;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    /*Get the postion not twist*/
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = msg->pose.pose.orientation.z;
  }

  void calculate_direction() {
    // Calculate distance and angle to goal
    double dx = desired_pos_.x - current_pos_.x;
    double dy = desired_pos_.y - current_pos_.y;
    distance_ = std::sqrt(dx * dx + dy * dy);
    angle_ = std::atan2(dy, dx) - current_pos_.theta;

    // Normalize angle
    while (angle_ > M_PI)
      angle_ -= 2 * M_PI;
    while (angle_ < -M_PI)
      angle_ += 2 * M_PI;
  }

  rclcpp_action::GoalResponse
  goal_callback(const rclcpp_action::GoalUUID &uuid,
                std::shared_ptr<const GoToPoseAction::Goal> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    desired_pos_ = goal_handle->goal_pos;
    (void)uuid;
    RCLCPP_INFO(
        this->get_logger(),
        "desired_pos_.x: %f, desired_pos_.y: %f, desired_pos_.theta: %f",
        desired_pos_.x, desired_pos_.y, desired_pos_.theta);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancel_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void execute_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<GoToPose::Feedback>();
    auto result = std::make_shared<GoToPose::Result>();

    auto rate = std::make_shared<rclcpp::Rate>(10);
    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        stop();
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      calculate_direction();

      // Check if we've reached the goal
      if (distance_ < 0.1 && std::abs(angle_) < 0.1) {
        stop();
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        return;
      }

      // Create and publish Twist message
      auto twist_msg = geometry_msgs::msg::Twist();
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = angle_;
      pub_->publish(twist_msg);

      // Publish feedback
      feedback->current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);

      rate->sleep();
    } // end while

    // If we're here, something went wrong
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Goal aborted");
  }

  void
  accepted_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    auto execute_in_thread = [this, goal_handle]() {
      return this->execute_callback(goal_handle);
    };
    std::thread{execute_in_thread}.detach();
  }

  void stop() {
    TwistMsg cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    pub_->publish(cmd_vel);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}