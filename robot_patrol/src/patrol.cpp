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
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 164 , msg->ranges[164]); //front(-pi)
        RCLCPP_INFO(this->get_logger(), "ranges[%d] : %f", 247, msg->ranges[247]);
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
  bool check_left_side = false;
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();

  RCLCPP_INFO(this->get_logger(),
              "================== Navigation Start ====================");

  // left side laser scan range indies from ranges[0](0 degree) to
  // ranges[165](-90 degree)
  // auto scan_start = laser_msg.begin() + (164); // ranges[0] ==> (-pi)
  // auto scan_end = laser_msg.end()-(164); // ranges[165] ==> -90(-pi/2)
  auto left_scan_start = laser_msg.begin() + 331; // ranges[331] ==> 0
  auto left_scan_end = laser_msg.begin() + 495;   // ranges[495] ==> 270(pi/2)

  auto left_min_it = std::min_element(left_scan_start, left_scan_end);
  float left_min_dist = *left_min_it;
  int left_min_xth = std::distance(left_scan_start, left_min_it);

  //    auto left_max_it = std::max_element(left_scan_start, left_scan_end);
  //    float left_max_dist = *left_max_it;
  //    int left_max_xth = std::distance(left_scan_start, left_max_it);

  // RCLCPP_INFO(this->get_logger(), "left_max_dist: %f, left_max_xth : %d",
  // left_max_dist, left_max_xth);
  RCLCPP_INFO(this->get_logger(), "left_min_dist: %f, left_min_xth : %d",
              left_min_dist, left_min_xth);

  auto right_scan_start = laser_msg.begin() + 165; // ranges[165] ==> -90(-pi/2)
  auto right_scan_end = laser_msg.begin() + 330;   // ranges[330] ==> 0

  auto right_min_it = std::min_element(right_scan_start, right_scan_end);
  float right_min_dist = *right_min_it;
  int right_min_xth = std::distance(right_scan_start, right_min_it);

  //    auto right_max_it = std::max_element(right_scan_start, right_scan_end);
  //    float right_max_dist = *right_max_it;
  //    int right_max_xth = std::distance(right_scan_start, right_max_it);

  // RCLCPP_INFO(this->get_logger(), "right_max_dist: %f, right_max_xth : %d",
  // right_max_dist, right_max_xth);
  RCLCPP_INFO(this->get_logger(), "right_min_dist: %f, right_min_xth : %d \n",
              right_min_dist, right_min_xth);

  if (previous_angle == 0.0) {
    if (right_min_dist > 0.35) {
      if (left_min_dist < 0.30) {
        check_left_side = true;
        RCLCPP_INFO(this->get_logger(), " ** Go Turn Right ** ");
      } else {
        direction_ = 0.0;
        RCLCPP_INFO(this->get_logger(), " ** Go Forward ** ");
      }

    } else {
      if (((right_min_dist >= 0.20) && (right_min_dist <= 0.30)) &&
          ((left_min_dist >= 0.20) && (left_min_dist <= 0.30))) {
        direction_ = 180 * laser_angle_increment;
        RCLCPP_INFO(this->get_logger(), "turn right ** 90 degree ** ");
      } else if ((right_min_xth > 140) && (right_min_xth <= 165)) {
        direction_ = 150 * laser_angle_increment; // turn left 20 degree
        RCLCPP_INFO(this->get_logger(), "turn left ** 75 degree ** ");
      } else if ((right_min_xth >= 115) && (right_min_xth <= 140)) {
        direction_ = 130 * laser_angle_increment; // turn left 20 degree
        RCLCPP_INFO(this->get_logger(), "turn left ** 65 degree ** ");
      } else if ((right_min_xth >= 55) && (right_min_xth <= 115)) {
        direction_ = 90 * laser_angle_increment; // turn left 20 degree
        RCLCPP_INFO(this->get_logger(), "turn left ** 45 degree ** ");
      } else {
        if (right_min_dist < 0.25) {
          direction_ = 40 * laser_angle_increment; // turn left 35 degree
          RCLCPP_INFO(this->get_logger(), "turn left ** 20 degree ** ");
        } else {
          direction_ = 0.0;
          RCLCPP_INFO(this->get_logger(), " ** Go Forward ** ");
        }
      }
    }
  } else { // previous_angle != 0.0
#if 0
        if(right_min_dist <= 0.20)
        {
            if((right_min_xth >=120)&&(right_min_xth <=165))
            {
                direction_ = 60 * laser_angle_increment; //turn left 20 degree
                RCLCPP_INFO(this->get_logger(), "turn left ** 30 degree ** ");
            }else{
                direction_ = 40 * laser_angle_increment; //turn left 35 degree
                RCLCPP_INFO(this->get_logger(), "turn left ** 20 degree ** ");
            }
        }
        else
#endif
    {
      if (left_min_dist < 0.30) {
        check_left_side = true;
        RCLCPP_INFO(this->get_logger(), " ** Go Turn Right ** ");
      } else {
        direction_ = 0.0;
        RCLCPP_INFO(this->get_logger(), " ** Go Forward ** ");
      }
    }
  }

  if (check_left_side) {
    if (previous_angle == 0.0) {
      if ((left_min_xth >= 0) && (left_min_xth <= 25)) {
        direction_ = -150 * laser_angle_increment; // turn right 30 degree;
        RCLCPP_INFO(this->get_logger(), " turn right ** 75 degree ** ");
      } else if ((left_min_xth > 25) && (left_min_xth <= 55)) {
        direction_ = -130 * laser_angle_increment; // turn right 30 degree;
        RCLCPP_INFO(this->get_logger(), " turn right ** 75 degree ** ");
      } else if ((left_min_xth > 55) && (left_min_xth <= 110)) {
        direction_ = -90 * laser_angle_increment; // turn right 30 degree;
        RCLCPP_INFO(this->get_logger(), " turn right ** 45 degree ** ");
      } else if ((left_min_xth > 110) && (left_min_xth <= 165)) {
        if (right_min_dist <= 2.5) {
          direction_ = -50 * laser_angle_increment; // turn right 30 degree;
          RCLCPP_INFO(this->get_logger(), " turn right ** 25 degree ** ");
        } else {
          direction_ = 0.0;
          RCLCPP_INFO(this->get_logger(), " ** Go Forward ** ");
        }
      }
    } else {
#if 0
            if(left_min_dist < 0.20)
            {
                direction_ =  -60 * laser_angle_increment; //turn right 30 degree; 
                RCLCPP_INFO(this->get_logger(), " ** turn right 30 degree ** ");
            }        
            else
#endif
      {
        direction_ = 0.0;
        RCLCPP_INFO(this->get_logger(), " ** Go Forward ** ");
      }
    }
  }

  previous_angle = direction_;
  RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
  // int scan_size = scan_end - scan_start;
  /*Publish the velocity*/
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
