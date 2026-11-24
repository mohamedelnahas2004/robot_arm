# Arctos 6-DOF Robotic Arm: ROS 1 → ROS 2 Migration

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros)
![MoveIt2](https://img.shields.io/badge/MoveIt2-2.0-orange?style=for-the-badge)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)

A ROS 2 migration of the Arctos robotic arm, **derived from and based on** the original [Arctos-Robotics/ROS](https://github.com/Arctos-Robotics/ROS) repository. Targets **ROS 2 Jazzy**, **MoveIt 2**, **ros2_control**, and **Gazebo Harmonic**.

</div>

---

## 📌 Origin

This project is **derived from the [Arctos-Robotics/ROS](https://github.com/Arctos-Robotics/ROS) repository**. The robot design, the original ROS 1 packages, and the underlying structure come from the Arctos Robotics authors. This repository migrates that work from ROS 1 to ROS 2.

- **Original repository:** https://github.com/Arctos-Robotics/ROS
- **Original stack:** ROS 1, `catkin`, `ros_control`, Gazebo Classic
- **This repository:** ROS 2 Jazzy, `colcon`/`ament`, `ros2_control`, Gazebo Harmonic

All credit for the original design and ROS 1 implementation goes to the Arctos Robotics authors. Please refer to their repository for the original license and documentation.

---

## 🔄 Overview

| Area | Original (ROS 1) | This port (ROS 2) |
|------|------------------|-------------------|
| Build system | `catkin` | `colcon` / `ament` |
| Client library | `rospy` / `roscpp` | `rclpy` / `rclcpp` |
| Motion planning | MoveIt | MoveIt 2 (`pick_ik`, STOMP, Pilz) |
| Control | `ros_control` | `ros2_control` |
| Launch files | XML `.launch` | Python `.launch.py` |
| Simulation | Gazebo Classic | Gazebo Harmonic |

---

## 🛠️ What Changed

### Build system and packages
- `catkin` → `ament` / `colcon`. `CMakeLists.txt` and `package.xml` were rewritten (format 3).
- `arm_control` was rebuilt as an `ament_python` package with `console_scripts` entry points.
- Dependencies moved to their ROS 2 equivalents: `rospy` → `rclpy`, MoveIt → MoveIt 2 (`moveit_py`).

### Nodes and code
- `rospy` nodes rewritten with `rclpy` (`Node` classes, timers, QoS profiles).
- ROS 1 services and `actionlib` → ROS 2 services and `rclpy.action` clients (e.g. the `MoveGroup` action).
- TF: `tf` → `tf2_ros` (`Buffer`, `TransformListener`).

### Launch files
- XML `.launch` → Python `.launch.py` (`LaunchDescription`, `Node`).
- `<arg>` / `<param>` → `DeclareLaunchArgument` and parameter dictionaries.

### Robot description (URDF / Xacro)
- Meshes referenced via `package://robot_description/...` and scaled (`0.001`).
- Joints changed from `continuous` to `revolute` with real limits.
- Joint names normalized to `joint_1` … `joint_6`, `jaw1`, `jaw2`.
- Added a `world` link and a fixed base joint.

### Control
- `ros_control` (`.trans` transmissions + controller YAML) → `ros2_control`:
  - `<ros2_control>` hardware interfaces defined in Xacro (`FakeSystem` for mock hardware, `gz_ros2_control/GazeboSimSystem` for simulation).
  - `ros2_controllers.yaml` with `joint_trajectory_controller` and `joint_state_broadcaster`.
- MoveIt controller config rewritten as `moveit_controllers.yaml`.

### MoveIt
- New MoveIt 2 configuration (SRDF, `joint_limits.yaml`, `kinematics.yaml`) generated with the Setup Assistant.
- IK solver: `pick_ik`. Planners: STOMP and Pilz.
- `moveit_servo.yaml` added for real-time servoing.

### Simulation
- Gazebo Classic → Gazebo Harmonic (`gz sim`).
- `libgazebo_ros_control` → `gz_ros2_control-system`.
- Classic `.gazebo` / `.world` files → SDF world (`pick_and_place.sdf`).

---

## 📦 Packages

| Package | Description |
|---------|-------------|
| `arm_control` | Python control package (`ament_python`) |
| `robot_arm_moveit_config` | MoveIt 2 pipelines, `ros2_control` config, and launch files |
| `robot_description` | URDF/Xacro model, STL meshes, RViz config, and Gazebo world |

---

## 🛠️ Prerequisites

* Ubuntu 24.04 / ROS 2 Jazzy
* Gazebo Harmonic
* MoveIt 2 & ros2_control

---

## ⚙️ Installation

```bash
# 1. System dependencies
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo apt install ros-jazzy-pick-ik ros-jazzy-stomp-moveit ros-jazzy-moveit-servo

# 2. Create the workspace and clone the repository
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/mohamedelnahas2004/robot_arm.git

# 3. Install missing ROS dependencies
cd ~/robot_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --symlink-install
source install/setup.bash
```

---

## 🕹️ Run the System

```bash
ros2 launch robot_arm_moveit_config sim_moveit.launch.py
```

---

## 🙏 Acknowledgements

This work is based on [Arctos-Robotics/ROS](https://github.com/Arctos-Robotics/ROS). Thanks to the Arctos Robotics authors for the original robot design and ROS 1 packages that made this migration possible.

---

## 👥 Team

| Name | GitHub |
|------|--------|
| Eyad Mezed | [@EyadMzed](https://github.com/eyadmzed) |
| Mohamed Abdel Aal | [@mohamedelnahas2004](https://github.com/mohamedelnahas2004) |
| Andrew Usama | [@Andrew-Usama](https://github.com/Andrew-Usama) |

---

## 📄 License

This project is licensed under the MIT License. See [LICENSE](LICENSE). The original Arctos Robotics code remains under its own license, so keep their license notice with any files derived from their repository.
