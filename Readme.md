# 🚀 ROS 2 Action Muxer

`action_muxer` is a ROS 2 C++ package that demonstrates dynamic action preemption via topic-based goal input. Useful in robotics systems where high-priority goals (like emergency stop) must interrupt lower-priority actions.

---

## 📂 Folder Structure

```
action_muxer/
├── action/
│   └── Timer.action          # Custom action file
├── src/
│   ├── timer_action_server.cpp  # Action server
│   └── topic_goal_client.cpp    # Topic-triggered action client
├── CMakeLists.txt            # Build configuration
├── package.xml               # Package metadata
└── README.md                 # This file
```

---

## 🧠 Custom Action Definition (`action/Timer.action`)

```action
int32 duration
---
bool success
---
string feedback
```

- `duration`: Time (in seconds) to simulate work.
- `success`: Whether the goal completed successfully.
- `feedback`: Status updates during execution.

---

## 🔧 Build Instructions

```bash
# 1. Source ROS 2
source /opt/ros/humble/setup.bash

# 2. Create workspace (if not already done)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 3. Clone or copy the package
git clone <your-repo-url> action_muxer

# 4. Build the package
cd ~/ros2_ws
colcon build --packages-select action_muxer

# 5. Source the overlay
source install/setup.bash
```

---

## ▶️ Run Instructions

### 🖥️ Terminal 1 – Start the Action Server

```bash
ros2 run action_muxer timer_action_server
```

### 🖥️ Terminal 2 – Start the Topic-based Client

```bash
ros2 run action_muxer topic_goal_client
```

### 🖥️ Terminal 3 – Publish a Goal

```bash
ros2 topic pub /goal_input std_msgs/msg/Int32 "{data: 5}"
```

📌 Try publishing rapidly:

```bash
ros2 topic pub /goal_input std_msgs/msg/Int32 "{data: 2}" -r 3
```

This will preempt earlier goals dynamically.

---

## 💡 How It Works

- The **client** listens to `/goal_input` (type: `std_msgs/msg/Int32`)
- Each message → cancels any running goal and sends a new one
- The **server** waits for 5 seconds unless canceled
- Provides feedback + handles abortion cleanly

---

## 📦 Dependencies

- ROS 2 Humble or later
- rclcpp
- rclcpp_action
- std_msgs
- ament_cmake

---

## 👤 Author

**Ankit Srivastav**  
Executive Member, Drone & IoT Club  
MMMUT Gorakhpur  
📧 ankitsrivastav.resume@gmail.com

---

## 📄 License

Licensed under the [MIT License](https://opensource.org/licenses/MIT).
