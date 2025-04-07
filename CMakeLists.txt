cmake_minimum_required(VERSION 3.8)
project(action_muxer)

# Enable C++17
set(CMAKE_CXX_STANDARD 17)

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add the action definition
rosidl_generate_interfaces(action_muxer
  "action/Timer.action"
)

# Add executables
add_executable(timer_action_server src/timer_action_server.cpp)
ament_target_dependencies(timer_action_server rclcpp rclcpp_action std_msgs)
rosidl_target_interfaces(timer_action_server
  action_muxer "rosidl_typesupport_cpp")

add_executable(topic_goal_client src/topic_goal_sender.cpp)
ament_target_dependencies(topic_goal_client rclcpp rclcpp_action std_msgs)
rosidl_target_interfaces(topic_goal_client
  action_muxer "rosidl_typesupport_cpp")

# Install targets
install(TARGETS
  timer_action_server
  topic_goal_client
  DESTINATION lib/action_muxer)

# Export package
ament_package()
