# Project 3 — Terminal Commands

## 1. Wall-follower activity

### Terminal 0 — Build

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select my_robot_control
source install/setup.bash
```

### Terminal 1 — Robot bringup

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py
```

### Terminal 2A — Differential wall follower

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_control my_robot_wallfollower_differential.launch.xml
```

### Terminal 2B — Basic holonomic wall follower

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_control my_robot_wallfollower_holonomic.launch.py
```

### Terminal 2C — Improved holonomic wall follower

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_control my_robot_wallfollower_holonomic_obstacle.launch.py \
  follow_distance:=0.35 \
  max_front_length:=0.50 \
  forward_speed:=0.20 \
  lateral_speed:=0.20 \
  turn_speed:=1.00 \
  detection_limit:=1.20 \
  lateral_gain:=2.0 \
  alignment_gain:=2.0
```

## 2. Autonomous navigation

### Terminal 0 — Build

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select my_robot_navigation2 my_robot_nav_control
source install/setup.bash
```

### Terminal 1 — Robot bringup

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py
```

### Terminal 2 — Localization and Nav2

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_navigation2 navigation2_robot.launch.py \
  use_sim_time:=false \
  map_file:=my_lab_map.yaml \
  params_file:=rubot_real_lidar_nav.yaml
```

### Terminal 3 — Laboratory-map waypoints

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_nav_control nav_waypoints.launch.py \
  wp_file:=waypoints_hw_lab_0511.yaml \
  set_initial_pose:=false
```

## 3. AI traffic-sign navigation

### Terminal 0 — Activate the environment and build

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source .venv-yolo/bin/activate
colcon build --symlink-install \
  --packages-select my_robot_ai_perception my_robot_ai_navigation
source install/setup.bash
```

### Terminal 1 — Robot bringup

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup my_robot_bringup_hw.launch.py
```

### Terminal 2 — Localization and Nav2

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_navigation2 navigation2_robot.launch.py \
  use_sim_time:=false \
  map_file:=my_lab_map.yaml \
  params_file:=rubot_real_lidar_nav.yaml
```

### Terminal 3 — Traffic-sign route

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_ai_navigation traffic_sign_navigation.launch.py \
  config_file:=traffic_sign_navigation_real.yaml
```

### Terminal 4 — YOLO classifier

```bash
cd ~/my_rUBot_mecanum
source /opt/ros/humble/setup.bash
source .venv-yolo/bin/activate
source install/setup.bash
ros2 launch my_robot_ai_perception traffic_sign_classifier.launch.py \
  config_file:=traffic_sign_classifier_real.yaml
```
