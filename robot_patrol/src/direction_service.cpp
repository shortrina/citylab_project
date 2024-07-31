/*Server of direction service*/

#include <custom_interfaces/srv/get_direction.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>

using DirectionMsg = custom_interfaces::srv::GetDirection;
using LaserScanMsg = sensor_msgs::msg::LaserScan;

using namespace std::chrono_literals;
using namespace std::placeholders;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("service_server") {
    server_ = this->create_service<DirectionMsg>(
        "direction_service",
        std::bind(&DirectionService::get_direction_callback, this, _1, _2));
  }

private:
  rclcpp::Service<DirectionMsg>::SharedPtr server_;

  void get_direction_callback(
      const std::shared_ptr<DirectionMsg::Request> request,
      const std::shared_ptr<DirectionMsg::Response> response) {
    auto laser_info = request->laser_data;
    std::array<float, 3> total_dist_sec = {0.0, 0.0, 0.0};
    auto left_begin = laser_info.ranges.begin() + 165;
    auto left_end = laser_info.ranges.begin() + 285;
    auto mid_begin = laser_info.ranges.begin() + 286;
    auto mid_end = laser_info.ranges.begin() + 390;
    auto right_begin = laser_info.ranges.begin() + 391;
    auto right_end = laser_info.ranges.begin() + 476;

    RCLCPP_INFO(this->get_logger(), "Get Request from Client");
    for (auto it = left_begin; it != left_end; it++) {
      total_dist_sec[0] += *it;
    }
    for (auto it = mid_begin; it != mid_end; it++) {
      total_dist_sec[1] += *it;
    }
    for (auto it = right_begin; it != right_end; it++) {
      total_dist_sec[2] += *it;
    }

    RCLCPP_INFO(this->get_logger(), "left : %f, mid : %f, right : %f",
                total_dist_sec[0], total_dist_sec[1], total_dist_sec[2]);
    auto max_dist =
        *std::max_element(total_dist_sec.begin(), total_dist_sec.end());
    RCLCPP_INFO(this->get_logger(), "Sending Response to Client");
    if (max_dist == total_dist_sec[0]) {
      RCLCPP_INFO(this->get_logger(), "Left");
      response->direction = "Left";
    } else if (max_dist == total_dist_sec[1]) {
      RCLCPP_INFO(this->get_logger(), "Mid");
      response->direction = "Forward";
    } else {
      RCLCPP_INFO(this->get_logger(), "Right");
      response->direction = "Right";
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}