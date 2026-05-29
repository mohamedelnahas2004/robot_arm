# 6-DOF Robotic Arm

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros)
![MoveIt2](https://img.shields.io/badge/MoveIt2-2.0-orange?style=for-the-badge)
![OpenCV](https://img.shields.io/badge/OpenCV-Vision-green?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

A fully integrated 6-DOF robotic arm featuring dual operating modes: an autonomous computer-vision sorting pipeline and a low-latency gesture-controlled teleoperation system. Built for ROS 2 Jazzy and Gazebo Harmonic, and designed for real-world hardware deployment.

</div>

---

## 🚀 Operating Modes

### 1. Autonomous Vision-Sorting Mode
The arm operates independently using an overhead camera mounted perpendicular to the workspace. 
* **Perception:** Utilizes OpenCV to actively scan the workspace, identifying target objects (balls) and the drop-zone (box).
* **Dynamic Logic:** The system continuously generates pick-and-place coordinates for detected balls, automatically filtering out and ignoring objects that have successfully been placed inside the box.
* **Kinematics:** Powered by **MoveIt 2** utilizing the `pick_ik` solver for collision-aware motion planning and trajectory execution.

*(Insert a GIF or link to your Autonomous Mode video here)*

### 2. Gesture Teleoperation Mode
A decoupled, low-latency manual control mode driven entirely by human hand tracking.
* **Perception:** Utilizes a standard webcam and **MediaPipe** to track hand landmarks in real-time.
* **Smart Macros:** Moving the open hand translates to X/Y workspace movement. Closing the hand into a fist triggers an automated "plunge-and-grasp" macro. Opening the hand instantly releases the payload. A "thumbs-up" gesture triggers a system-wide E-STOP, freezing all motor commands.
* **Kinematics:** Bypasses MoveIt entirely. Utilizes a highly optimized, custom **Damped Least Squares (DLS) IK Solver** written specifically for this robot's geometry to ensure fluid, zero-lag tracking.

*(Insert a GIF or link to your Teleoperation Mode video here)*

---

## 📦 Packages

| Package | Description |
|---------|-------------|
| `arm_control` | ROS 2 controllers, hardware interfaces, CV nodes, and custom IK solvers |
| `robot_arm_moveit_config` | MoveIt 2 motion planning pipelines, STOMP/pick_ik configurations |
| `robot_description` | URDF/Xacro models, STL/DAE meshes, and RViz visualizations |

---

## 🛠️ Prerequisites

* Ubuntu 24.04 / ROS 2 Jazzy
* Gazebo Harmonic
* MoveIt 2 & ros2_control
* OpenCV & MediaPipe
* Python `numpy < 2`

---

## ⚙️ Installation

```bash
# 1. Install system dependencies and Python libraries
sudo apt update
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo apt install ros-jazzy-pick-ik ros-jazzy-stomp-moveit ros-jazzy-cv-bridge ros-jazzy-moveit-servo
pip install opencv-python ultralytics mediapipe "numpy<2" --break-system-packages

# 2. Create the workspace and clone the repository
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone https://github.com/mohamedelnahas2004/robot_arm.git

# 3. Install missing ROS dependencies automatically
cd ~/robot_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install
source install/setup.bash
```

---

## 🕹️ Run the System

**Launch Autonomous Sorting Mode:**
```bash
ros2 launch robot_arm_moveit_config sim_moveit.launch.py
```

**Launch Gesture Teleoperation Mode:**
```bash
ros2 launch robot_arm_moveit_config sim_moveit.launch.py mode:=teleop
```

---

## 👥 Team

| Name | Role / GitHub |
|------|--------|
| Mohamed Abdel Aal | [@mohamedelnahas2004](https://github.com/mohamedelnahas2004) |
| Eyad Mezed | [@eyadmzed](https://github.com/eyadmzed) |
| Mahmoud Farrag | [@mamfmahmoud](https://github.com/mamfmahmoud) |
| Andrew Usama | [@Andrew-Usama](https://github.com/Andrew-Usama) |
| Mohamed Abd Elmonem | [@men3emhoa](https://github.com/men3emhoa) |

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
