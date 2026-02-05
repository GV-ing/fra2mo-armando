# 🤖 FRA2MO ARMANDO

**Mobile robot with manipulator arm for Pick & Place applications**

## 📖 Description

FRA2MO ARMANDO is a ROS2 project that implements a differential mobile robot equipped with a 4 degrees of freedom (DOF) manipulator arm and a gripper. The robot is designed to perform autonomous **Pick & Place** tasks in simulated environments using Gazebo Ignition.

### Main Features

- 🚗 **Differential mobile base** with Nav2 control
- 🦾 **4-DOF manipulator arm** with inverse kinematics (IKPy)
- 🔧 **Gripper** for pick and release operations
- 🗺️ **SLAM** for environment mapping (slam_toolbox)
- 📍 **AMCL localization** for autonomous navigation
- 🔍 **Autonomous exploration** with explore_lite
- 📷 **Sensors**: LiDAR and Camera

---

## 📋 Prerequisites

### Operating System
- Ubuntu 22.04 LTS
- ROS2 Humble 

### ROS2 Dependencies
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-explore-lite \
                 ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-nav2-simple-commander
```

### 1. Install required libraries
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdep humble -y
pip3 install ikpy
```

### 2. Make scripts executable
```bash
cd ~/ros2_ws/src/fra2mo_armando
chmod +x scripts/*.py
```

### 3. Build the workspace
```bash
cd ~/ros2_ws
colcon build --packages-select fra2mo_armando
source install/setup.bash
```

---

## 🗂️ Package Structure

```
fra2mo_armando/
├── config/                    # Configuration files
│   ├── amcl.yaml             # AMCL parameters
│   ├── navigation.yaml       # Nav2 parameters
│   ├── slam.yaml             # SLAM parameters
│   ├── explore.yaml          # Exploration parameters
│   └── fra2mo_armando_controllers.yaml  # ros2_control controllers
├── launch/                    # Launch files
│   ├── launch_world.launch.py          # Gazebo + Full robot
│   ├── display_fra2mo_armando.py       # RViz visualization only
│   ├── fra2mo_slam.launch.py           # SLAM
│   ├── fra2mo_amcl.launch.py           # AMCL localization
│   ├── fra2mo_navigation.launch.py     # Nav2 navigation
│   ├── fra2mo_explore.launch.py        # Autonomous exploration
│   └── ik_pick_place.launch.py         # Pick & Place task
├── maps/                      # Saved maps
│   └── mia_mappa.yaml
├── meshes/                    # Robot 3D meshes
├── models/                    # Gazebo models
├── rviz_conf/                 # RViz configurations
├── scripts/                   # Python scripts
│   ├── ik_pick_place.py      # Pick & Place with IK
│   ├── pick_place_demo.py    # Pick & Place demo
│   ├── follow_waypoints.py   # Waypoints navigation
│   ├── reach_goal.py         # Single goal navigation
│   └── test_arm.py           # Arm test
├── src/                       # C++ sources
│   ├── pick_place.cpp        # Pick & Place C++ node
│   └── odom_bl_tf.cpp        # TF publisher odom->base_link
├── urdf/                      # Robot URDF/Xacro description
│   └── fra2mo.urdf.xacro
├── worlds/                    # Gazebo worlds
│   └── office_small.sdf
├── CMakeLists.txt
└── package.xml
```

---

## 🚀 Usage

### 1. Start Gazebo Simulation

Launch the Gazebo world with the robot:
```bash
ros2 launch fra2mo_armando launch_world.launch.py
```

### 2. RViz Visualization Only (without Gazebo)
```bash
ros2 launch fra2mo_armando display_fra2mo_armando.py
```

### 3. SLAM - Environment Mapping

In a new terminal (with Gazebo already running):
```bash
ros2 launch fra2mo_armando fra2mo_slam.launch.py
```

To save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/fra2mo_armando/maps/mia_mappa
```

### 4. AMCL Localization

With map already available:
```bash
ros2 launch fra2mo_armando fra2mo_amcl.launch.py
```

### 5. Autonomous Navigation
```bash
ros2 launch fra2mo_armando fra2mo_navigation.launch.py
```

### 6. Autonomous Exploration

Exploration with integrated SLAM:
```bash
ros2 launch fra2mo_armando fra2mo_explore.launch.py
```

---

## 🎯 Pick & Place Task

### Full launch (Gazebo + Task)
```bash
ros2 launch fra2mo_armando ik_pick_place.launch.py
```

### Task only (with Gazebo already running)
```bash
ros2 run fra2mo_armando ik_pick_place.py
```

The task performs:
1. Navigation towards the defined pillars
2. Obstacle detection via LiDAR
3. Pick sequence execution with inverse kinematics
4. Navigation towards the deposit box
5. Place sequence execution

---

## 🎮 Manual Control

### Keyboard teleoperation
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Direct arm control
```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, 0.5, -1.0, -0.5, 0.0]}"
```

Format: `[joint1, joint2, joint3, joint4, gripper]`

---

## 📡 Main Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Base velocity commands |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR data |
| `/videocamera` | `sensor_msgs/Image` | Camera images |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | Arm commands |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL estimated pose |
| `/model/fra2mo/odometry` | `nav_msgs/Odometry` | Robot odometry |

---

## ⚙️ Configuration Parameters

### Controller Type
The launch file supports different controller types:
```bash
ros2 launch fra2mo_armando launch_world.launch.py controller_type:=position_controller
```

Options:
- `position_controller` (default)
- `joint_trajectory_controller`

---

## 👥 Author

- **GV-ing** - [GitHub](https://github.com/GV-ing)

---

## 📄 License

This project is developed for educational purposes as part of the Robotics Lab course.

---

## 🔗 References

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Ignition](https://gazebosim.org/)
- [IKPy - Inverse Kinematics](https://github.com/Phylliade/ikpy)
