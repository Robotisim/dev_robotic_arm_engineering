# motion

ROS 2 Python package for Panda trajectory demos (IK + path generation).

## What This Package Owns

- demo nodes (`motion_01_*` to `motion_05_*`)
- demo launch files in `launch/`
- a Panda wrapper launch: `panda_bringup.launch.py`

## Build

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select motion
source install/setup.bash
```

If you want to use `start_panda:=true` or `panda_bringup.launch.py`, build `panda` too:

```bash
colcon build --symlink-install --packages-select panda motion
source install/setup.bash
```

## Launch Files

Start Panda using this package wrapper:

```bash
ros2 launch motion panda_bringup.launch.py
```

Run demos:

```bash
ros2 launch motion demo_01_point_to_point.launch.py
ros2 launch motion demo_02_straight_line.launch.py
ros2 launch motion demo_03_circle_bump.launch.py
ros2 launch motion demo_04_rectangle_shape.launch.py
ros2 launch motion demo_05_workspace_reject.launch.py
```

If Panda is already running:

```bash
ros2 launch motion demo_01_point_to_point.launch.py start_panda:=false
```

## Demo Launch Arguments

All `demo_0X_*.launch.py` files support:

- `start_panda` (default `true`)
- `start_delay_sec` (default `6.0` in launch; node default `2.5`)
- `marker_preview_sec` (default `1.0`)
- `trajectory_start_offset_sec` (default `0.8`)
- `trajectory_topic` (default `/joint_trajectory_controller/joint_trajectory`)

## Demo Topics

- Trajectory command: `/joint_trajectory_controller/joint_trajectory`
- Marker output: `/motion/trajectory_markers`
- Status text: `/motion/status`

## Demos Summary

- `motion_01_point_to_point`: point-to-point Cartesian targets
- `motion_02_straight_line`: straight-line Cartesian path
- `motion_03_circle_bump`: circle path with Z bump profile
- `motion_04_rectangle_shape`: closed rectangle path
- `motion_05_workspace_reject`: out-of-bounds path rejection example
