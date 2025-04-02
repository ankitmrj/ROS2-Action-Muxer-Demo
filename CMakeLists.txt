cmake_minimum_required(VERSION 3.8)
project(action_muxer_demo)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server
  rclcpp
  rclcpp_action
  std_msgs
  custom_interfaces
)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client
  rclcpp
  rclcpp_action
  custom_interfaces
)

add_executable(demo_publisher src/demo_publisher.cpp)
ament_target_dependencies(demo_publisher
  rclcpp
  std_msgs
)

install(TARGETS
  action_server
  action_client
  demo_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
