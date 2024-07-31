/*Client of direction service*/

#include <custom_interfaces/srv/get_direction.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <thread>

using DirectionMsg = custom_interfaces::srv::GetDirection;
using LaserScanMsg = sensor_msgs::msg::LaserScan;

using namespace std::chrono_literals;
using namespace std::placeholders;

class TestService : public rclcpp::Node {
public:
  TestService() : Node("service_client") {
    client_cb_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_cb_group_ = client_cb_group_;
    sub_cb_group_ = timer_cb_group_;
    sub_options.callback_group = sub_cb_group_;

    client_ = this->create_client<custom_interfaces::srv::GetDirection>(
        "direction_service", rmw_qos_profile_services_default,
        client_cb_group_);
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&TestService::scan_callback, this, _1), sub_options);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&TestService::timer_callback, this), timer_cb_group_);
  }

private:
  rclcpp::CallbackGroup::SharedPtr client_cb_group_;
  rclcpp::CallbackGroup::SharedPtr timer_cb_group_;
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;

  rclcpp::SubscriptionOptions sub_options;
  rclcpp::Client<DirectionMsg>::SharedPtr client_;
  rclcpp::Subscription<LaserScanMsg>::SharedPtr sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<LaserScanMsg> laser_info_;

  void response_callback(rclcpp::Client<DirectionMsg>::Response response) {
    auto status = response.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success");
      auto response = future.get();
      RCLCPP_INFO(this->get_logger(), "Get Direction: %s from Server",
                  response->direction.c_str());
      if (response->direction == "Left") {
        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>();
        cmd_vel->linear.x = 0.1;
        cmd_vel->angular.z = 0.5;
        pub_->publish(std::move(cmd_vel));
        RCLCPP_INFO(this->get_logger(), "Go Left");
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
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void timer_callback() {
    auto request = std::make_shared<DirectionMsg::Request>();
    request->laser_data = *laser_info_;
    client_->async_send_request(request,
                                std::bind(&TestService::response_callback, this,
                                          std::placeholders::_1));
  }

  void scan_callback(const std::shared_ptr<LaserScanMsg> msg) {
    laser_info_ = msg;
    RCLCPP_INFO(this->get_logger(), "Scan: %f", laser_info_->ranges[0]);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto client_node = std::make_shared<TestService>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(client_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}