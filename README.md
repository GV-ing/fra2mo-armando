# 🤖 FRA2MO ARMANDO

**Robot mobile con braccio manipolatore per applicazioni di Pick & Place**

## 📖 Descrizione

FRA2MO ARMANDO è un progetto ROS2 che implementa un robot mobile differenziale equipaggiato con un braccio manipolatore a 4 gradi di libertà (DOF) e un gripper. Il robot è progettato per eseguire task autonomi di **Pick & Place** in ambienti simulati tramite Gazebo Ignition.

### Caratteristiche Principali

- 🚗 **Base mobile differenziale** con controllo tramite Nav2
- 🦾 **Braccio manipolatore 4-DOF** con cinematica inversa (IKPy)
- 🔧 **Gripper** per operazioni di presa e rilascio
- 🗺️ **SLAM** per mappatura dell'ambiente (slam_toolbox)
- 📍 **Localizzazione AMCL** per navigazione autonoma
- 🔍 **Esplorazione autonoma** con explore_lite
- 📷 **Sensori**: LiDAR e Camera

---

## 📋 Prerequisiti

### Sistema Operativo
- Ubuntu 22.04 LTS
- ROS2 Humble 

### Dipendenze ROS2
```bash
sudo apt update
sudo apt install ros-humble-nav2-bringup ros-humble-slam-toolbox ros-humble-explore-lite \
                 ros-humble-ros-gz ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-nav2-simple-commander
```

### 1. Installazione librerie necessarie
```bash
cd ~/ros2_ws
rosdep install -i --from-path src --rosdep humble -y
pip3 install ikpy
```

### 2. Rendi eseguibili gli script
```bash
cd ~/ros2_ws/src/fra2mo_armando
chmod +x scripts/*.py
```

### 3. Build del workspace
```bash
cd ~/ros2_ws
colcon build --packages-select fra2mo_armando
source install/setup.bash
```

---

## 🗂️ Struttura del Package

```
fra2mo_armando/
├── config/                    # File di configurazione
│   ├── amcl.yaml             # Parametri AMCL
│   ├── navigation.yaml       # Parametri Nav2
│   ├── slam.yaml             # Parametri SLAM
│   ├── explore.yaml          # Parametri esplorazione
│   └── fra2mo_armando_controllers.yaml  # Controller ros2_control
├── launch/                    # Launch files
│   ├── launch_world.launch.py          # Gazebo + Robot completo
│   ├── display_fra2mo_armando.py       # Solo visualizzazione RViz
│   ├── fra2mo_slam.launch.py           # SLAM
│   ├── fra2mo_amcl.launch.py           # Localizzazione AMCL
│   ├── fra2mo_navigation.launch.py     # Navigazione Nav2
│   ├── fra2mo_explore.launch.py        # Esplorazione autonoma
│   └── ik_pick_place.launch.py         # Task Pick & Place
├── maps/                      # Mappe salvate
│   └── mia_mappa.yaml
├── meshes/                    # Mesh 3D del robot
├── models/                    # Modelli Gazebo
├── rviz_conf/                 # Configurazioni RViz
├── scripts/                   # Script Python
│   ├── ik_pick_place.py      # Pick & Place con IK
│   ├── pick_place_demo.py    # Demo Pick & Place
│   ├── follow_waypoints.py   # Navigazione waypoints
│   ├── reach_goal.py         # Navigazione a goal singolo
│   └── test_arm.py           # Test braccio
├── src/                       # Sorgenti C++
│   ├── pick_place.cpp        # Nodo Pick & Place C++
│   └── odom_bl_tf.cpp        # Publisher TF odom->base_link
├── urdf/                      # Descrizione robot URDF/Xacro
│   └── fra2mo.urdf.xacro
├── worlds/                    # Mondi Gazebo
│   └── office_small.sdf
├── CMakeLists.txt
└── package.xml
```

---

## 🚀 Utilizzo

### 1. Avvio Simulazione Gazebo

Lancia il mondo Gazebo con il robot:
```bash
ros2 launch fra2mo_armando launch_world.launch.py
```

### 2. Solo Visualizzazione RViz (senza Gazebo)
```bash
ros2 launch fra2mo_armando display_fra2mo_armando.py
```

### 3. SLAM - Mappatura dell'ambiente

In un nuovo terminale (con Gazebo già attivo):
```bash
ros2 launch fra2mo_armando fra2mo_slam.launch.py
```

Per salvare la mappa:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/fra2mo_armando/maps/mia_mappa
```

### 4. Localizzazione AMCL

Con mappa già disponibile:
```bash
ros2 launch fra2mo_armando fra2mo_amcl.launch.py
```

### 5. Navigazione Autonoma
```bash
ros2 launch fra2mo_armando fra2mo_navigation.launch.py
```

### 6. Esplorazione Autonoma

Esplorazione con SLAM integrato:
```bash
ros2 launch fra2mo_armando fra2mo_explore.launch.py
```

---

## 🎯 Task Pick & Place

### Avvio completo (Gazebo + Task)
```bash
ros2 launch fra2mo_armando ik_pick_place.launch.py
```

### Solo Task (con Gazebo già running)
```bash
ros2 run fra2mo_armando ik_pick_place.py
```

Il task esegue:
1. Navigazione verso i pilastri definiti
2. Rilevamento ostacolo tramite LiDAR
3. Esecuzione sequenza di Pick con cinematica inversa
4. Navigazione verso la box di deposito
5. Esecuzione sequenza di Place

---

## 🎮 Controllo Manuale

### Teleoperazione da tastiera
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Controllo diretto del braccio
```bash
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [1.0, 0.5, -1.0, -0.5, 0.0]}"
```

Formato: `[joint1, joint2, joint3, joint4, gripper]`

---

## 📡 Topic Principali

| Topic | Tipo | Descrizione |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Comandi velocità base |
| `/scan` | `sensor_msgs/LaserScan` | Dati LiDAR |
| `/videocamera` | `sensor_msgs/Image` | Immagini camera |
| `/position_controller/commands` | `std_msgs/Float64MultiArray` | Comandi braccio |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Posa stimata AMCL |
| `/model/fra2mo/odometry` | `nav_msgs/Odometry` | Odometria robot |

---

## ⚙️ Parametri di Configurazione

### Controller Type
Il launch file supporta diversi tipi di controller:
```bash
ros2 launch fra2mo_armando launch_world.launch.py controller_type:=position_controller
```

Opzioni:
- `position_controller` (default)
- `joint_trajectory_controller`

---

## 👥 Autore

- **GV-ing** - [GitHub](https://github.com/GV-ing)

---

## 📄 Licenza

Questo progetto è sviluppato per scopi didattici nell'ambito del corso di Robotics Lab.

---

## 🔗 Riferimenti

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [Gazebo Ignition](https://gazebosim.org/)
- [IKPy - Inverse Kinematics](https://github.com/Phylliade/ikpy)
