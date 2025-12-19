# 🤖 FRA2MO ARMANDO



## 📋 PREREQUISITI

### 1. Istallazione librerie necessarie
```bash
rosdep install -i --from-path src --rosdistro <distro> -y
pip3 install ikpy
```

### 2. Rendi eseguibile lo script
```bash
cd ~/ros2_image_fold/fra2mo_armando
chmod +x scripts/ik_pick_place.py
```

### 3. Build del workspace
```bash
cd ~/ros2_image_fold/fra2mo_armando
colcon build
source install/setup.bash
```

---

## 🚀 UTILIZZO

### Gazebo
```bash
ros2 launch fra2mo_armando launch_world.launch.py
```

### Gazebo
```bash
ros2 launch fra2mo_armando display_fra2mo_armando.py
```

### Comandi manuali
```bash
# Terminal 1
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
```bash
# Terminal 2
ros2 topic pub /position_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0, 0.5, -1.0, -0.5, 0.0]}"
```

### Task
```bash
ros2 launch fra2mo_armando ik_pick_place.launch.py
```

### Solo Task (Gazebo già running)
```bash
# Se Gazebo è già attivo:
ros2 run fra2mo_armando ik_pick_place.py
```


