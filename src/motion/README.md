# motion

ROS 2 Python package for Panda trajectory demos (IK + path generation).

## What This Package Owns

- demo nodes (`motion_01_*` to `motion_06_*`) and arm-hand pose-goal wrapper launch (`demo_07_*`)
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
ros2 launch motion demo_06_moveit_pose_goal.launch.py
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py
```

If Panda is already running:

```bash
ros2 launch motion demo_01_point_to_point.launch.py start_panda:=false
```

## Demo Launch Arguments

Trajectory demos `demo_01_*` to `demo_05_*` support:

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
- `motion_06_moveit_pose_goal`: send a target pose to MoveIt (`panda_arm`) for IK + planning + execution
- `motion_07_moveit_arm_hand_pose_goal` (launch wrapper): pose goal for `panda_arm_hand` using fingertip TCP (`panda_hand_tcp`)

## MoveIt Pose Goal Demo

This demo sends one pose goal (x/y/z + quaternion) to MoveIt action `/move_action`.
MoveIt handles IK, planning, and execution.
Before sending the goal, it logs the current `target_link` pose from TF.

```bash
ros2 launch motion demo_06_moveit_pose_goal.launch.py \
  target_x:=0.50 target_y:=0.10 target_z:=0.35
```

If Panda + MoveIt are already running:

```bash
ros2 launch motion demo_06_moveit_pose_goal.launch.py start_panda:=false start_delay_sec:=0.5
```

Disable current-pose logging if needed:

```bash
ros2 launch motion demo_06_moveit_pose_goal.launch.py \
  log_current_pose_before_goal:=false
```

## Arm-Hand Pose Goal Demo (Fingertip TCP)

Use this when you want pose goals for planning group `panda_arm_hand` referenced at the fingers/tool center.
Workflow: run a predefined Panda environment first, then run this node.
The node logs current pose first, then logs and sends the target goal pose.

```bash
# Terminal 1: predefined Panda environment
ros2 launch panda panda_pick_and_place_cubes.launch.py

# Terminal 2: pose-goal node
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py \
  target_x:=0.06 target_y:=0.50 target_z:=0.30 \
  target_qx:=1.0 target_qy:=0.0 target_qz:=0.0 target_qw:=0.0
```

Control gripper opening/closing from the same command while moving with `panda_arm_hand`:

```bash
# Keep gripper open while moving (default opening)
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py \
  gripper_command:=open

# Keep gripper closed while moving
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py \
  gripper_command:=close
```

Set an exact gripper opening (direct angle/position) together with the pose goal:

```bash
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py \
  target_x:=0.06 target_y:=0.50 target_z:=0.30 \
  target_qx:=1.0 target_qy:=0.0 target_qz:=0.0 target_qw:=0.0 \
  gripper_target:=0.020
```

Notes:

- `gripper_target` overrides `gripper_command` when both are provided.
- Typical Panda finger range is around `0.0` (closed) to `0.035` (open).
- Set `gripper_target:=-1.0` to disable direct target and use `gripper_command`.

Optional: you can still enable a finger joint planning constraint, but it is disabled by default for `panda_arm_hand`:

```bash
ros2 launch motion demo_07_moveit_arm_hand_pose_goal.launch.py \
  use_finger_joint_constraint:=true \
  finger_joint_target:=0.035
```
