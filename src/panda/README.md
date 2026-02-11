# panda

ROS 2 Jazzy package for Panda robot description, Gazebo simulation assets, controllers, and Panda launches.

## What This Package Owns

- Panda URDF/Xacro + meshes + worlds
- Panda ros2_control configs
- Panda launches (`panda_view`, `panda_sim`, `panda_sim_control`, `panda_pick_and_place`)
- wrist-mounted RGB-D camera sensor in URDF (`panda_wrist_eye_link`)

## Build

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select panda
source install/setup.bash
```

## Launch Files

```bash
# RViz model view
ros2 launch panda panda_view.launch.py

# Empty Gazebo world + Panda + wrist camera topic bridge
ros2 launch panda panda_sim.launch.py

# Empty Gazebo world + Panda + controllers + wrist camera topic bridge
ros2 launch panda panda_sim_control.launch.py

# Pick-and-place world (table + objects) + external RGB-D + wrist RGB-D + RViz
ros2 launch panda panda_pick_and_place.launch.py
```

## Recent Package Changes

- Resource paths are now set directly in Panda Gazebo launches (`GZ_SIM_RESOURCE_PATH`, `IGN_GAZEBO_RESOURCE_PATH`), so manual exports are not required.
- Added eye-in-hand RGB-D sensor mounted before the gripper on `panda_link8`, oriented outward.
- `panda_pick_and_place.launch.py` now bridges both cameras:
  - external world camera topics
  - wrist camera topics
- `view_rgbd.rviz` now shows external and wrist camera views together.

## Camera Topics

External RGB-D camera (from the world):
- `/camera/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

Wrist RGB-D camera (from Panda URDF sensor):
- `/wrist_eye/image_raw`
- `/wrist_eye/depth/image_raw`
- `/wrist_eye/depth/camera_info`

Quick check:

```bash
ros2 topic list | rg "camera|wrist_eye"
```

## Example Control Node

```bash
ros2 run panda example_position
```
