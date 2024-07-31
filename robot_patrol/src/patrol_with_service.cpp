#include "../include/robot_patrol/patrol_with_service.hpp"
#include <custom_interfaces/srv/get_direction.hpp>
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

  last_laser_ = msg;
}

void Patrol::navigation_loop() {
    auto request = std::make_shared<DirectionMsg::Request>();
    request->laser_data = *last_laser_;
    auto result = client_->async_send_request(request);
    if (rclcpp::ok()) {
      auto response = result.get();
      RCLCPP_INFO(this->get_logger(), "Direction: %s",
                  response->direction.c_str());

      if (response->direction == "Left") {
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = 0.1;
        cmd_vel->angular.z = 0.5;
        pub_->publish(std::move(cmd_vel));

      } else if (response->direction == "Right") {
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = 0.1;
        cmd_vel->angular.z = -0.5;
        pub_->publish(std::move(cmd_vel));
      } else if (response->direction == "Forward") {
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = 0.1;
        cmd_vel->angular.z = 0.0;
        pub_->publish(std::move(cmd_vel));
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service");
    }

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
