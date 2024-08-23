
/* Implement Action Server of go_to_pose action*/

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "geometry_msgs/msg/detail/pose__struct.hpp"
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
  const double POSITION_THRESHOLD = 0.1;
  const double ORIENTATION_THRESHOLD = 0.1;

  geometry_msgs::msg::Pose2D
      current_pos_; // save current_position  which is free space (float
                    // linear.x and angular.z)
  geometry_msgs::msg::Pose2D
      desired_pos_; // get the goal position which is free space from client
                    // (float x,y,theta)(theta = degree)
  /* Initail position of turtlebot3 burger robot ([go_to_pose_action]: pose.x:
   * 0.085723, pose.y: 0.499925, pose.theta: -0.001164)*/
  double roll_, pitch_, yaw_ = 0.0;
  // position of free space
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    /*Get the postion not twist*/
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll_, pitch_, yaw_);
    current_pos_.theta = yaw_;
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

    // convert radians
    desired_pos_.theta = desired_pos_.theta * (M_PI / 180.0);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  cancel_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  double normalize_angle(double angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
    return angle;
  }

  enum class RobotState {
    ROTATING_TO_GOAL,
    MOVING_TO_GOAL,
    FINAL_ORIENTATION,
    DONE
  };

  void execute_callback(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();
    rclcpp::Rate loop_rate(10); // Hz for each control loop

    float delta_x = 0.0;
    float delta_y = 0.0;
    float delta_theta = 0.0;
    float distance = 0.0;
    float direction = 0.0;
    TwistMsg cmd_vel;
    float linear_speed_x = 0.0;
    float angular_speed_z = 0.0;
    RobotState current_state = RobotState::ROTATING_TO_GOAL;

    while (rclcpp::ok()) {
      if (goal_handle->is_active()) {

        delta_x = desired_pos_.x - current_pos_.x;
        delta_y = desired_pos_.y - current_pos_.y;
        delta_theta = normalize_angle(desired_pos_.theta - current_pos_.theta);

        if ((desired_pos_.x == 0.0) || (desired_pos_.y == 0.0))
          direction = 0.0;
        else
          direction = normalize_angle(std::atan2(delta_y, delta_x) -
                                      current_pos_.theta);

        distance = std::sqrt((delta_x * delta_x) + (delta_y * delta_y));
        switch (current_state) {
        case RobotState::ROTATING_TO_GOAL:
          // Implement rotation logic
          if (std::fabs(delta_theta) <= ORIENTATION_THRESHOLD) {
            current_state = RobotState::MOVING_TO_GOAL;
          } else {
            RCLCPP_INFO(this->get_logger(), "ROTATING_TO_GOAL");
            linear_speed_x = 0.0;
            angular_speed_z = 0.5 * delta_theta;
          }
          break;
        case RobotState::MOVING_TO_GOAL:
          // Implement movement logic
          if (std::fabs(delta_x) <= POSITION_THRESHOLD &&
              std::fabs(delta_y) <= POSITION_THRESHOLD) {
            current_state = RobotState::FINAL_ORIENTATION;
          } else {
            RCLCPP_INFO(this->get_logger(), "MOVING_TO_GOAL");
            // Move towards the goal position
            if ((desired_pos_.y == 0.0) || (desired_pos_.x == 0.0)) {
              linear_speed_x = -0.1;
              angular_speed_z = 0.0;
            } else {
              linear_speed_x = 0.5 * distance;

              if ((std::fabs(delta_x) > 0.1) || (std::fabs(delta_y) > 0.1)) {
                RCLCPP_INFO(this->get_logger(), "Direction: %f", direction);
                angular_speed_z = 0.5 * direction;
              } else {
                angular_speed_z = 0.0;
              }
            }
          }
          break;
        case RobotState::FINAL_ORIENTATION:
          // Implement final orientation adjustment
          if (std::fabs(delta_theta) <= ORIENTATION_THRESHOLD) {
            current_state = RobotState::DONE;
          } else {
            RCLCPP_INFO(this->get_logger(), "FINAL_ORIENTATION");
            // Fine-tune final orientation
            linear_speed_x = 0.0;
            angular_speed_z = 0.2 * delta_theta;
          }
          break;
        case RobotState::DONE:
          // Set result and exit
          linear_speed_x = 0.0;
          angular_speed_z = 0.0;

          stop();
          result->status = true;
          goal_handle->succeed(result);
          break;
        }
        RCLCPP_INFO(this->get_logger(),
                    "delta_x : %f | "
                    "delta_y : %f",
                    delta_x, delta_y);

        RCLCPP_INFO(this->get_logger(),
                    "delta_theta : %f | "
                    "direction : %f | "
                    "distance : %f",
                    delta_theta, direction, distance);
        RCLCPP_INFO(this->get_logger(), "\n linear_speed_x : %f",
                    linear_speed_x);
        RCLCPP_INFO(this->get_logger(), "angular_speed_z : %f",
                    angular_speed_z);

        cmd_vel.linear.x = linear_speed_x;
        cmd_vel.angular.z = angular_speed_z;
        pub_->publish(cmd_vel);

        feedback->current_pos.x = current_pos_.x;
        feedback->current_pos.y = current_pos_.y;
        feedback->current_pos.theta =
            current_pos_.theta * (180 / M_PI); // convert radians to degrees
        // feedback->current_pos.theta = current_pos_.theta;
        goal_handle->publish_feedback(feedback);

      } else if (goal_handle->is_canceling()) {
        stop();
        result->status = false;
        goal_handle->canceled(result);
        break;
      } // end if active
      loop_rate.sleep();
    } // end while
  }   // end execute

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
}; // end of class

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto action_server = std::make_shared<GoToPose>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}