# Robotic Arm Engineering Workspace

ROS 2 Jazzy workspace for simulation and control demos with:
- `arm_sim_bringup`: custom 4DOF stick arm + Panda wrapper launch
- `panda`: Panda simulation/control package (Gazebo + ros2_control)
- `motion`: numbered Panda trajectory demos (IK + path generation)

## 1) Prerequisites

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
  ros-jazzy-xacro \
  ros-jazzy-moveit-resources-panda-description
```

## 2) Build

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

If your default Python is Conda and misses `catkin_pkg`:

```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## 3) Launch Files by Category

### A) Bring Robot Up (Visualization / Simulation)

Stick arm RViz:

```bash
ros2 launch arm_sim_bringup stick_arm_rviz.launch.py
```

Stick arm Gazebo + ros2_control:

```bash
ros2 launch arm_sim_bringup stick_arm_gz_control.launch.py
```

Panda Gazebo + ros2_control (wrapper from `arm_sim_bringup` to `panda`):

```bash
ros2 launch arm_sim_bringup panda_gz_control.launch.py
```

Direct Panda bringup launch (from `motion`, wraps `panda` package):

```bash
ros2 launch motion panda_bringup.launch.py
```

Panda pick-and-place world launch (external + wrist RGB-D):

```bash
ros2 launch panda panda_pick_and_place.launch.py
```

### B) Run Motion Control Demos (Panda)

Auto-start Panda + run demo:

```bash
ros2 launch motion demo_01_point_to_point.launch.py
ros2 launch motion demo_02_straight_line.launch.py
ros2 launch motion demo_03_circle_bump.launch.py
ros2 launch motion demo_04_rectangle_shape.launch.py
ros2 launch motion demo_05_workspace_reject.launch.py
```

If Panda is already running, avoid relaunching simulator:

```bash
ros2 launch motion demo_01_point_to_point.launch.py start_panda:=false
```

### C) Manual Control / Verification Commands

List controllers:

```bash
ros2 control list_controllers
```

Publish stick-arm sample trajectory:

```bash
ros2 topic pub --once /stick_arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: [joint1, joint2, joint3, joint4],
  points: [
    {positions: [0.0, 0.5, -0.4, 0.2], time_from_start: {sec: 2}},
    {positions: [1.0, 0.3, 0.2, -0.6], time_from_start: {sec: 5}}
  ]
}"
```

Panda demos command topic:
- `/joint_trajectory_controller/joint_trajectory`

## 4) Recommended Run Flows

### Flow 1: Stick Arm Teaching
1. `ros2 launch arm_sim_bringup stick_arm_rviz.launch.py`
2. `ros2 launch arm_sim_bringup stick_arm_gz_control.launch.py`
3. Send a sample `JointTrajectory` command

### Flow 2: Panda + Motion Demos
1. `ros2 launch motion panda_bringup.launch.py`
2. In another terminal: `ros2 launch motion demo_02_straight_line.launch.py start_panda:=false`

### Flow 3: One-command Demo Start
1. `ros2 launch motion demo_03_circle_bump.launch.py`

## 5) Troubleshooting

- Controllers not active:
  - Check `ros2 control list_controllers`
  - Ensure `joint_state_broadcaster` and trajectory controller are `active`
- No motion:
  - Confirm demo publishes to `/joint_trajectory_controller/joint_trajectory`
- Launch errors with Conda Python:
  - Rebuild with `-DPython3_EXECUTABLE=/usr/bin/python3`

## 6) Package-specific Docs

- `src/arm_sim_bringup/README.md`
- `src/motion/README.md`
- `src/panda/README.md`
