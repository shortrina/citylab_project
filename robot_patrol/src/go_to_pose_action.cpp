
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

  geometry_msgs::msg::Pose
      current_pos_; // save current_position (float linear.x and angular.z)
  geometry_msgs::msg::Pose2D
      desired_pos_; // get the goal value from client (float x,y,theta)
  double roll_, pitch_, yaw_ = 0.0;
  /* Initail position of turtlebot3 burger robot ([go_to_pose_action]: pose.x:
   * 0.085723, pose.y: 0.499925, pose.theta: -0.001164)*/

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    /*Get the postion not twist*/
    current_pos_ = msg->pose.pose;
    tf2::Quaternion q(current_pos_.orientation.x, current_pos_.orientation.y,
                      current_pos_.orientation.z, current_pos_.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
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
    rclcpp::Rate loop_rate(10); // Hz for each control loop
    float delta_x = 0.0;
    float delta_y = 0.0;
    float distance = 0.0;
    float angle_to_goal = 0.0;
    float angle_diff = 0.0;
    TwistMsg cmd_vel;
    float linear_speed_x = 0.0;
    float angular_speed_z = 0.0;

    // RCLCPP_INFO(this->get_logger(), "Action server is executing");

    while (rclcpp::ok()) {
      if (goal_handle->is_active()) {
        // RCLCPP_INFO(this->get_logger(), "goal_handle->is_active()");
        delta_x = desired_pos_.x - current_pos_.position.x;
        delta_y = desired_pos_.y - current_pos_.position.y;
        distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        angle_to_goal = std::atan2(delta_y, delta_x);
        angle_diff = angle_to_goal - yaw_;
        angle_diff = std::atan2(std::sin(angle_diff), std::cos(angle_diff));
        if (distance < 0.1) {
          result->status = true;
          stop();
          goal_handle->succeed(result);
          break;
        } else {
          /*RCLCPP_INFO(this->get_logger(),
                      "Calculate position and return feedback");*/

          if (std::fabs(angle_diff) > 0.1) {
            linear_speed_x = 0.0;
            // compute the angular speed based on the difference between the
            // desired theta and the current theta
            angular_speed_z = 0.5 * angle_diff;
          } else {
            linear_speed_x = 0.2 * distance;
            // compute the angular speed based on the difference between the
            // direction and the current theta
            angular_speed_z = 0.0;
          }

          if (angular_speed_z > (M_PI / 2))
            angular_speed_z = M_PI / 2;
          else if (angular_speed_z < -(M_PI / 2))
            angular_speed_z = -(M_PI / 2);

          cmd_vel.linear.x = linear_speed_x;
          cmd_vel.angular.z = angular_speed_z;
          pub_->publish(cmd_vel);

          feedback->current_pos.x = current_pos_.position.x;
          feedback->current_pos.y = current_pos_.position.y;
          // feedback->current_pos.theta = current_pos_.theta * 180 / M_PI;
          feedback->current_pos.theta = angle_diff;
          goal_handle->publish_feedback(feedback);
        }
      } else if (goal_handle->is_canceling()) {
        stop();
        result->status = false;
        goal_handle->canceled(result);
        break;
      }
      loop_rate.sleep();
    }
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