#ifndef PATROL_WITH_SERVICE_HPP
#define PATROL_WITH_SERVICE_HPP

#include <custom_interfaces/srv/get_direction.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>

using DirectionMsg = custom_interfaces::srv::GetDirection;
using LaserScanMsg = sensor_msgs::msg::LaserScan;

using namespace std::chrono_literals;
using namespace std::placeholders;

// For the parameters, templates of seconds or ms

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_with_service")
        {
    // Initialize your publisher, timer, etc. here
    // Initialize one Reentrant callback group object
    navi_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_cb_group_ = navi_callback_group_;
    sub_cb_group_ = timer_cb_group_;
    navigation_options.callback_group = navi_callback_group_;

    client_ = this->create_client<custom_interfaces::srv::GetDirection>(
        "direction_service",rclcpp::ServicesQoS() ,
        navi_callback_group_);

    pub_  =  this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_  =  this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Patrol::scanCallback, this, std::placeholders::_1),navigation_options);
                
    navigation_timer_ = this->create_wall_timer(500ms, std::bind(&Patrol::navigation_loop, this),navi_callback_group_); 

  }//end of constructor

  ~Patrol() { stop(); }

  void stop();

private:
  rclcpp::CallbackGroup::SharedPtr navi_callback_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  rclcpp::SubscriptionOptions navigation_options;

  rclcpp::Client<DirectionMsg>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  
  rclcpp::TimerBase::SharedPtr navigation_timer_; // controller
  std::shared_ptr<LaserScanMsg> last_laser_; //store the last laser information

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void navigation_loop();
};

#endif // PATROL_WITH_SERVICE_HPP
