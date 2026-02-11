# arm_sim_bringup

ROS 2 Jazzy bringup package for:
- a custom 4DOF stick arm (`stick_arm_4dof.urdf.xacro`)
- a Panda bringup wrapper launch (`panda_gz_control.launch.py`)

## What This Package Owns

- URDF/Xacro and controllers for the stick arm
- RViz + Gazebo launches for the stick arm
- a wrapper launch that forwards to `panda/panda_sim_control.launch.py`

## Prerequisites

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro
```

## Build

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select arm_sim_bringup
source install/setup.bash
```

If your default Python is Conda and misses `catkin_pkg`:

```bash
colcon build --symlink-install --packages-select arm_sim_bringup --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Launch Files

Stick arm in RViz:

```bash
ros2 launch arm_sim_bringup stick_arm_rviz.launch.py
```

Stick arm in Gazebo + ros2_control:

```bash
ros2 launch arm_sim_bringup stick_arm_gz_control.launch.py
```

Optional args:

```bash
ros2 launch arm_sim_bringup stick_arm_gz_control.launch.py rviz:=false world:=empty.sdf
```

Panda Gazebo control wrapper:

```bash
ros2 launch arm_sim_bringup panda_gz_control.launch.py
```

Optional args:

```bash
ros2 launch arm_sim_bringup panda_gz_control.launch.py use_sim_time:=true
```

## Stick Arm Control Check

After launching `stick_arm_gz_control.launch.py`:

```bash
ros2 control list_controllers
```

Expected active controllers:
- `joint_state_broadcaster`
- `stick_arm_controller`

Send a sample trajectory:

```bash
ros2 topic pub --once /stick_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4],
  points: [
    {positions: [0.0, 0.5, -0.4, 0.2], time_from_start: {sec: 2}},
    {positions: [1.0, 0.3, 0.2, -0.6], time_from_start: {sec: 5}}
  ]
}"
```

## Notes

- `panda_gz_control.launch.py` is a wrapper owned by this package.
- Panda model, Panda sensors, and Panda-specific controller details are defined in the `panda` package.
