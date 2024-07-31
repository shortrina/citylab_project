cmake_minimum_required(VERSION 3.8)
project(robot_patrol)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

#include directories
include_directories(include)

add_executable(service_server_node src/direction_service.cpp)
add_executable(patrol_with_service_node src/patrol_with_service.cpp)
#add_executable(service_client_node src/test_service.cpp)

ament_target_dependencies(service_server_node rclcpp geometry_msgs sensor_msgs custom_interfaces)
#ament_target_dependencies(service_client_node rclcpp geometry_msgs sensor_msgs custom_interfaces)
ament_target_dependencies(patrol_with_service_node rclcpp geometry_msgs sensor_msgs std_msgs custom_interfaces)


#add_executable(robot_patrol src/robot_patrol.cpp)
#add_executable(robot_patrol_node src/patrol.cpp)

#add_dependencies(robot_patrol ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#ament_target_dependencies(robot_patrol_node rclcpp geometry_msgs sensor_msgs)

install(TARGETS
    service_server_node
    patrol_with_service_node
  DESTINATION lib/${PROJECT_NAME})


#install the launch directoy
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
