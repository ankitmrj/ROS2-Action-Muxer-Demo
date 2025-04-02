# ROS 2 Action Muxer Demo

## Overview
Demonstrates priority-based action preemption in ROS 2 (C++).

## Nodes
- `action_server`: Handles goals with preemption capability
- `action_client`: Example action client
- `demo_publisher`: Triggers new goals via topic messages

## Build Instructions
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone [your-repo-url]
cd ~/ros2_ws
colcon build --packages-select action_muxer_demo
source install/setup.bash
