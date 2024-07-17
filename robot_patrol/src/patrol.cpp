#include "../include/robot_patrol/patrol.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>

/* The orientationn is front (go Forward)*/

void Patrol::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(),"range_size: %ld", msg->ranges.size());

  laser_msg = msg->ranges;
  has_received_laser_ = true;

#if 0
    auto min_dist_it = std::min_element(laser_msg.begin(),laser_msg.end());
    int min_xth = std::distance(laser_msg.begin(), min_dist_it);
    RCLCPP_INFO(this->get_logger(), "min_value: %f, min_xth : %d", *min_dist_it, min_xth);

#endif
#if 0
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 0 , msg->ranges[0]); //front(-pi)
        RCLCPP_INFO(this->get_logger(), "ranges[%d]: %f", 164 , msg->ranges[164]); //front(-pi)
        RCLCPP_INFO(this->get_logger(), "ranges[%d]  : %f", 247, msg->ranges[247]);
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 330 , msg->ranges[330]);
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 412, msg->ranges[412]); //back(0)
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 494 , msg->ranges[494]);
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f \n\n", 659 , msg->ranges[659]); // front(pi)
#endif
}

// laser scan range indies from ranges[165] to ranges[475]
// ranges[165] = -90 degree, ranges[475] = 90 degree
void Patrol::navigation_loop() {
  // RCLCPP_INFO(this->get_logger(), "navigation Start");

  if (!has_received_laser_) {
    // RCLCPP_INFO(this->get_logger(), "Waiting for first laser scan...");
    return;
  }

  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  RCLCPP_INFO(this->get_logger(),
              "================== Navigation Start ====================");

  auto scan_start = laser_msg.begin() + 165; // ranges[165] ==> -90(-pi/2)
  auto scan_end = laser_msg.begin() + 475;   // ranges[475] ==> 90(pi/2)
  int scan_mid_index = 165;
  //   float avoid_distance = 0.35; // 30cm
  //    int turn_right = -1;         // turn clockwise
  //    int turn_left = 1;           // turn counter clock wise

  int turn_direction = 0;
  auto min_it = std::min_element(scan_start, scan_end);
  float min_dist = *min_it;
  int min_xth = std::distance(scan_start, min_it);

  //  auto max_it = std::max_element(scan_start, scan_end);
  //  float max_dist = *max_it;
  //  int max_xth = std::distance(scan_start, max_it);
  RCLCPP_INFO(this->get_logger(), "min_dist: %f, min_xth : %d", min_dist,
              min_xth);
  // RCLCPP_INFO(this->get_logger(), "max_dist: %f, max_xth : %d", max_dist,
  //           max_xth);

  if (min_xth < scan_mid_index) {
    turn_direction = 1;
    min_xth = min_xth + 30;
  } else {
    turn_direction = -1;
    min_xth = (360 - min_xth);
  }

#if 0
  if (min_xth <= 60) {
    turn_direction = 1;
    min_xth = 30 + min_xth;
    RCLCPP_INFO(this->get_logger(), " Turn LEFT ");
  } else if ((min_xth >= 61) && (min_xth < 120)) {
    turn_direction = 1;
    min_xth = min_xth;
    RCLCPP_INFO(this->get_logger(), " Turn LEFT ");
  } else if ((min_xth >= 121) && (min_xth <= 165)) {
    turn_direction = 1;
    min_xth = min_xth;
    RCLCPP_INFO(this->get_logger(), " Turn LEFT ");
  } else if ((min_xth >= 166) && (min_xth < 226)) {
    turn_direction = -1;
    min_xth = (330 - min_xth);
    RCLCPP_INFO(this->get_logger(), " Turn RIGHT ");
  } else if ((min_xth >= 226) && (min_xth < 286)) {
    turn_direction = -1;
    min_xth = (330 - min_xth);
    RCLCPP_INFO(this->get_logger(), " Turn RIGHT ");
  } else {
    turn_direction = -1;
    min_xth = (330 - min_xth) + 30;
    RCLCPP_INFO(this->get_logger(), " Turn RIGHT ");
  }
#endif

  if (previous_angle == 0.0) {
    if (min_dist <= 0.35) {
      direction_ = turn_direction * min_xth * laser_angle_increment;
      RCLCPP_INFO(this->get_logger(), "direction : %f", direction_);
    } else if ((min_dist >= 0.14) && (min_dist <= 0.20)) {
      x_direction = 1;
      direction_ = turn_direction * min_xth * laser_angle_increment;
    } else if (min_dist <= 0.13) {
      x_direction = -1;
      direction_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "Go Backward, too close to Wall");
    }
  } else {
    if (min_dist > 0.35) {
      direction_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "previous_angle != 0.0");
    } else if ((min_dist >= 0.14) && (min_dist <= 0.20)) {
      x_direction = 1;
      direction_ = turn_direction * min_xth * laser_angle_increment;
    } else if (min_dist <= 0.13) {
      x_direction = -1;
      direction_ = 0.0;
      RCLCPP_INFO(this->get_logger(), "Go Backward, too close to Wall");
    } else {
      direction_ = turn_direction * min_xth * laser_angle_increment;
    }
  }

  previous_angle = direction_;
  RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
  // int scan_size = scan_end - scan_start;
  /*Publish the velocity*/
  // cmd_vel->linear.x = -0.1;
  // cmd_vel->angular.z = 0.0;

  cmd_vel->linear.x = 0.1;
  cmd_vel->angular.z = direction_;

  // RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
  pub_->publish(std::move(cmd_vel));
  RCLCPP_INFO(this->get_logger(), "Publish_cmd and navigation End\n\n\n");
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
