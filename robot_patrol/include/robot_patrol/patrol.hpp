#ifndef ROBOT_PATROL_PATROL_HPP
#define ROBOT_PATROL_PATROL_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

// For the parameters, templates of seconds or ms
using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("robot_patrol") {

    // Initialize your publisher, timer, etc. here
    // Initialize one Reentrant callback group object
    navi_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions navigation_options;
    navigation_options.callback_group = navi_callback_group_;

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::scanCallback, this, std::placeholders::_1),
        navigation_options);

    navigation_timer_ = this->create_wall_timer(
        500ms, std::bind(&Patrol::navigation_loop, this), navi_callback_group_);

    laser_msg = std::vector<float>(660, 0.0);
  } // end of constructor

  ~Patrol() { stop(); }

  void navigation_loop();
  void stop();

private:
  /*Define variable*/
  float pi = 3.1415926;
  float laser_angle_min = -3.14;
  float laser_angle_max = 3.14;
  float laser_angle_increment =
      0.00954; // rad/s(0.00954*2 = 0.01908 ,0.0191radians == 1.01908 degree)
  float laser_distance_min = 0.12; // 12cm
  float laser_distance_max = 30.0; // 3M

  float previous_angle = 0.0; // previous motion status of robot , 0 : go
                              // straight, or previous rotate motion value.

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr navigation_timer_;
  rclcpp::CallbackGroup::SharedPtr navi_callback_group_;

  std::vector<float> laser_msg;
  std::atomic<bool> has_received_laser_;

  float wait_t = 0.5;
  float direction_ = 0.0;

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
};

#endif