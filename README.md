# Robot Arm

<div align="center">

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue?style=for-the-badge&logo=ros)
![MoveIt2](https://img.shields.io/badge/MoveIt2-2.0-orange?style=for-the-badge)
![License](https://img.shields.io/badge/License-MIT-green?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Active-brightgreen?style=for-the-badge)

A 6-DOF Robotic Arm project built with ROS2, MoveIt2, and ros2_control for motion planning and control.

</div>

---

## Packages

| Package | Description |
|---------|-------------|
| `arm_control` | ROS2 controllers and hardware interfaces |
| `robot_arm_moveit_config` | MoveIt2 motion planning configuration |
| `robot_description` | URDF/Xacro models, meshes, and RViz configuration |

---

## Repository Structure

```text
robot_arm/
├── src/
│   ├── robot_description/
│   │   ├── arm_control/             # the nodes
│   │   ├── resources/      
│   │   └── test/
│   │
│   ├── robot_arm_moveit_config/
│   │   ├── config/             # MoveIt planning pipelines and kinematics
│   │   ├── launch/             # MoveIt demo and execution launch files
│   │   └── rviz/               # RViz visualization configurations
│   │
│   └── robot_description/
│       ├── config/             # MoveIt2 and ros2_control configuration files
│       ├── meshes/             # CAD visual and collision STL/DAE assets
│       ├── urdf/               # Robot model definitions and Xacro macros
│       └── launch/             # RViz and robot_state_publisher launch files
│
└── README.md
```

---

## Prerequisites

- ROS2 Jazzy
- MoveIt2
- ros2_control
- colcon
- trac-ik
- stomp
- gazebo harmonic

---

## Installation

```bash
sudo apt update
sudo apt install ros-jazzy-pick-ik ros-jazzy-stomp-moveit ros-jazzy-cv-bridge ros-jazzy-moveit-servo
pip install opencv-python ultralytics mediapipe "numpy<2" --break-system-packages
git clone https://github.com/mohamedelnahas2004/robot_arm.git
cd ~/robot_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Run

```bash
#autonomous mode:
source install/setup.bash
ros2 launch robot_arm_moveit_config sim_moveit.launch.py

#teleoperated mode:
source install/setup.bash
ros2 launch robot_arm_moveit_config sim_moveit.launch.py mode:=teleop
```

---

## Team

| Name | GitHub |
|------|--------|
| Mohamed Abdel Aal | [@mohamedelnahas2004](https://github.com/mohamedelnahas2004) |
| Eyad Mezed | [@eyadmzed](https://github.com/eyadmzed) |
| Mahmoud Farrag | [@mamfmahmoud](https://github.com/mamfmahmoud) |
| Andrew Usama | [@Andrew-Usama](https://github.com/Andrew-Usama) |
| Mohamed Abd Elmonem | [@men3emhoa](https://github.com/men3emhoa) |

---

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
