/* Implement Action Server of go_to_pose action*/

#include <chrono>
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

  TwistMsg current_twist;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    /*Get the postion not twist*/
    current_pos_ = ConvertOdometryToPose2D(msg);
    current_twist = ConvertOdometryToTwist(msg);
    RCLCPP_INFO(this->get_logger(),
                "current_twist.linear.x : %f, current_twist.linear.y: %f, "
                "current_twist.angular.z : %f",
                current_twist.linear.x, current_twist.linear.y,
                current_twist.angular.z);
  }

  geometry_msgs::msg::Pose2D
  ConvertOdometryToPose2D(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::Pose2D pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.theta = msg->pose.pose.orientation.z;
    return pose;
  }

  nav_msgs::msg::Odometry::SharedPtr
  ConvertPose2DToOdometry(const geometry_msgs::msg::Pose2D pose) {
    nav_msgs::msg::Odometry::SharedPtr msg;
    msg->pose.pose.position.x = pose.x;
    msg->pose.pose.position.y = pose.y;
    msg->pose.pose.orientation.z = pose.theta;
    return msg;
  }
  /*Function to convert Odometry to Twist*/
  TwistMsg
  ConvertOdometryToTwist(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::Twist twist;

    // Convert position to linear velocity
    twist.linear.x = msg->pose.pose.position.x;
    twist.linear.y = msg->pose.pose.position.y;

    // Convert orientation quaternion to Euler angles
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Set angular velocity to yaw (theta)
    twist.angular.z = yaw;

    return twist;
  }

  /*Function to convert Twist to Odometry*/
  nav_msgs::msg::Odometry
  ConvertTwistToOdometry(const geometry_msgs::msg::Twist &twist) {
    nav_msgs::msg::Odometry odom;

    // Convert linear velocity to position
    odom.pose.pose.position.x = twist.linear.x;
    odom.pose.pose.position.y = twist.linear.y;

    // Convert angular velocity (z) to orientation quaternion
    tf2::Quaternion q;
    q.setRPY(0, 0, twist.angular.z);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    return odom;
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
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();

    RCLCPP_INFO(this->get_logger(), "Action server is executing");

    while (rclcpp::ok()) {
      if (goal_handle->is_active()) {
        if (desired_pos_.x == current_pos_.x &&
            desired_pos_.y == current_pos_.y &&
            desired_pos_.theta == current_pos_.theta) {
          result->status = true;
          goal_handle->succeed(result);
          auto odometry = ConvertPose2DToOdometry(current_pos_);
          auto cmd_vel = ConvertOdometryToTwist(odometry);
          pub_->publish(cmd_vel);
          break;
        } else {
          feedback->current_pos.x = sqrt((current_pos_.x - desired_pos_.x) *
                                             (current_pos_.x - desired_pos_.x) +
                                         (current_pos_.y - desired_pos_.y) *
                                             (current_pos_.y - desired_pos_.y));
          feedback->current_pos.y = current_pos_.y;
          feedback->current_pos.theta = current_pos_.theta;
          goal_handle->publish_feedback(feedback);
          auto odometry = ConvertPose2DToOdometry(feedback->current_pos);
          auto cmd_vel = ConvertOdometryToTwist(odometry);
          pub_->publish(cmd_vel);
        }
      } else {
        if (goal_handle->is_canceling()) {
          result->status = false;
          goal_handle->canceled(result);
        }
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void
  accepted_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    auto execute_in_thread = [this, goal_handle]() {
      return this->execute_callback(goal_handle);
    };
    std::thread{execute_in_thread}.detach();
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