#include "../include/robot_patrol/patrol.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>

void Patrol::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(),"range_size: %ld", msg->ranges.size());

  laser_msg = msg->ranges;
  has_received_laser_ = true;

#if 0
    auto min_dist_it = std::min_element(laser_msg.begin(),laser_msg.end());
    int min_xth = std::distance(laser_msg.begin(), min_dist_it);
    RCLCPP_INFO(this->get_logger(), "min_value: %f, min_xth : %d", *min_dist_it, min_xth);

#endif
}

void Patrol::navigation_loop() {
  // RCLCPP_INFO(this->get_logger(), "navigation Start");

  if (!has_received_laser_) {
    // RCLCPP_INFO(this->get_logger(), "Waiting for first laser scan...");
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  auto min_dist_it = std::min_element(laser_msg.begin(), laser_msg.end());
  int min_xth = std::distance(laser_msg.begin(), min_dist_it);
  RCLCPP_INFO(this->get_logger(), "min_value: %f, min_xth : %d", *min_dist_it,
              min_xth);

  if ((*min_dist_it) < 0.30) {
    /*Calculate the actual angular velocity of z-axis(Robot got the z-axis)*/
    direction_ = laser_angle_min + ((min_xth)*laser_angle_increment);

  } else {
    direction_ = 0.0;
  }

  /*Publish the velocity*/
  cmd_vel->linear.x = -0.1;
  cmd_vel->angular.z = direction_;
  RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
  pub_->publish(std::move(cmd_vel));
  RCLCPP_INFO(this->get_logger(), "Publish_cmd and navigation End");
}

void Patrol::stop() {
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel->linear.x = 0.0;
  cmd_vel->angular.z = 0.0;
  pub_->publish(std::move(cmd_vel));
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> patrol = std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
