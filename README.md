# ROS2_Ackermann_Navigation_Nav2
This ROS 2 package provides a complete Ackermann-drive robot setup including URDF model, LiDAR/IMU sensors, and controller plugins integrated with Nav2 for autonomous navigation. Supports simulation in Gazebo and RViz, enabling realistic path planning, obstacle avoidance, and smooth vehicle-like steering control.

---

## 🚗 Ackermann Drive Concept

Ackermann steering geometry enables realistic car-like motion for autonomous robots. Unlike differential-drive robots that turn by varying wheel speeds, an Ackermann robot steers using the front wheels by changing their angle.

During a turn, the inner wheel needs to turn at a sharper angle than the outer wheel so that all wheels follow circular paths with a common rotation center. This prevents wheel slip, ensures smooth cornering, and improves handling stability.

### 🔑 Key Benefits
- Realistic vehicle-like steering control
- Better traction and reduced tire wear
- Accurate path tracking for autonomous navigation
- Suitable for high-speed or long-wheelbase robots

### 📌 Applications
- Autonomous cars & research platforms
- Self-driving RC cars
- Ackermann mobile robots in ROS 2
- Robotics competitions & simulation projects (Gazebo/RViz)

Ackermann drive is ideal for projects needing dynamic and stable navigation using Nav2, allowing smooth motion planning and obstacle avoidance similar to real vehicles.

<img width="375" height="263" alt="Ackermann-steering-principle" src="https://github.com/user-attachments/assets/d48cb663-87de-40b5-b965-d4846efc1f5f" />

---

## ✨ Features

- ⚙️ Complete Ackermann-drive robot URDF integrated with LiDAR & IMU sensors
- 🚀 Fully compatible with Nav2 for autonomous navigation
- 🛰️ Supports Gazebo simulation and RViz visualization
- 🧭 Realistic steering and motion behavior similar to real vehicles
- 🛑 Obstacle avoidance, path planning & smoothing
- 🔌 Includes controller plugins and configurable parameters of Ackermann
- 📡 TF frame structure and sensor integration for localization & navigation

---

## 🎓 Learning Outcomes

By using this package, you will:

- Understand the concept and kinematics of Ackermann steering geometry
- Learn to configure an Ackermann-drive robot in ROS 2 URDF/XACRO
- Integrate sensors (LiDAR, IMU, Odometry) for navigation and mapping
- Launch Nav2 and execute autonomous path planning
- Configure controllers for real-car-like steering
- Visualize robot movement in RViz and simulate in Gazebo
- Execute end-to-end autonomous navigation using Nav2

---

## Installation

### Make Workspace
```bash
mkdir robot_ws/
```

### Change Workspace
```bash
cd robot_ws
```

### Make src
```bash
mkdir src/
```

### Change Workspace
```bash
cd src
```

### Clone This Repository
```bash
git clone https://github.com/yashbhaskar/ROS2_Ackermann_Navigation_Nav2.git
```

### Install Nav2
```bash
sudo apt update
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup
```

### (Optional recommended packages)
```bash
sudo apt install ros-humble-nav2-common
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-behavior-tree
```

### Change Workspace
```bash
cd ..
```

### Build the Package
```bash
colcon build --packages-select ackermann
source install/setup.bash
```

---

## 🚀 How to Run

### Launch robot in gazebo and Start autonoumous navigation
```bash
ros2 launch ackermann main.launch.py
```

- Now rviz is open and robot spawn in robot’s initial pose on the map and start navigation.
- Then use Nav2 Goal / 2D Goal Pose to send a navigation goal. The robot will autonomously plan a path and move toward the target, avoiding obstacles and reaching the goal successfully.
- Currently, this project uses static transform publishing to maintain a stable map frame during autonomous navigation, without running a localization system. 
- I am actively working on parameter tuning for Nav2, as the robot can occasionally get stuck in certain navigation conditions. These improvements will be updated in future commits.


---

## Photos

<img width="1848" height="1050" alt="1" src="https://github.com/user-attachments/assets/5705b1c4-5932-45c3-b10c-0d744826e7f9" />
<img width="1848" height="1050" alt="2" src="https://github.com/user-attachments/assets/4475763a-a5a1-4a48-91fc-137168b18820" />
<img width="1848" height="1050" alt="3" src="https://github.com/user-attachments/assets/f4bb0856-72fa-45f3-8eb8-e53eb6ec47f6" />
<img width="1848" height="1050" alt="4" src="https://github.com/user-attachments/assets/fc62cfad-3266-4e43-a81c-e021eab1f4c6" />
<img width="1848" height="1050" alt="5" src="https://github.com/user-attachments/assets/14e8c847-6267-4b3e-9c05-3a4fe7f5d1d2" />
<img width="1848" height="1050" alt="6" src="https://github.com/user-attachments/assets/d21c45ae-7393-472a-ac87-bf4e81b5a97e" />

---

## ✉️ Contact

📧 Yash Bhaskar – ybbhaskar19@gmail.com

📌 GitHub: https://github.com/yashbhaskar
