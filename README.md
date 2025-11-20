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
