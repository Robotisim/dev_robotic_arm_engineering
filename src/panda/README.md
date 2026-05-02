# panda

ROS 2 Jazzy package for Panda robot description, Gazebo simulation assets, controllers, and Panda launches.

## What This Package Owns

- Panda URDF/Xacro + meshes + worlds
- Panda ros2_control configs
- Panda launches (`panda_view`, `panda_sim`, `panda_sim_control`, `panda_pick_and_place`, `panda_pick_and_place_cubes`, `panda_ik_gazebo`, `panda_restricted_corridor`, `panda_restricted_pillars`, `panda_restricted_chicane`)
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

# Same launch but force a collision-free ready arm pose after controller load
ros2 launch panda panda_sim_control.launch.py seed_ready_pose:=true

# Pick-and-place world (table + objects) + external RGB-D + wrist RGB-D camera + MoveIt
ros2 launch panda panda_pick_and_place.launch.py

# IK + MoveIt RViz + Gazebo controllers (Plan and Execute drives Gazebo robot)
ros2 launch panda panda_ik_gazebo.launch.py

# Cube pick-and-place world: Panda + wrist RGB-D + table + cubes on table in front of robot (default 3, no world external camera bridge)
ros2 launch panda panda_pick_and_place_cubes.launch.py

# Same cube launch with custom cube count (1..5)
ros2 launch panda panda_pick_and_place_cubes.launch.py cube_count:=5

# Restricted corridor: narrow passage obstacle-avoidance demo
ros2 launch panda panda_multi_object_sequence.launch.py

# Restricted pillars: pillar field obstacle-avoidance demo
ros2 launch panda panda_restricted_pillars.launch.py

# Restricted chicane: zig-zag obstacle-avoidance demo
ros2 launch panda panda_restricted_chicane.launch.py
```

Use planning group `panda_arm` for wrist-reference arm motion or `panda_arm_hand` for fingertip TCP-reference arm motion (`panda_hand_tcp`) in MoveIt RViz. Use `hand` for gripper open/close, then click `Plan and Execute`.

If startup collision appears, increase bringup delays so the ready pose is applied before MoveIt starts:

```bash
ros2 launch panda panda_ik_gazebo.launch.py start_moveit_delay_sec:=12.0
```

## Clean Restart

If multiple Panda robots appear (leftover background processes), stop them before relaunching:

```bash
pkill -f "ros2 launch panda" || true
pkill -f "gz sim" || true
pkill -f "ros_gz_sim" || true
pkill -f "move_group" || true
pkill -f "controller_manager" || true
pkill -f "robot_state_publisher" || true
pkill -f "rviz2" || true
```

## Recent Package Changes

- Resource paths are now set directly in Panda Gazebo launches (`GZ_SIM_RESOURCE_PATH`, `IGN_GAZEBO_RESOURCE_PATH`), so manual exports are not required.
- Added eye-in-hand RGB-D sensor mounted before the gripper on `panda_link8`, oriented toward the grasp area.
- `panda_pick_and_place.launch.py` now launches MoveIt (`move_group`) with Panda controllers in the pick-and-place world.
- `panda_pick_and_place.launch.py` now supports reusable world and spawn arguments:
  - `world_file` for swapping the Gazebo world
  - `spawn_x`, `spawn_y`, `spawn_z`, `spawn_roll`, `spawn_pitch`, `spawn_yaw`
  - `bridge_external_camera` to enable/disable external RGB-D bridging
- `panda_pick_and_place.launch.py` also spawns a second external RGB-D camera from another angle (topics under `/external_camera/*`).
- Gripper controller and MoveIt hand controller mapping are enabled, so Panda hand can open/close from MoveIt RViz.
- Added configurable cube spawning to pick-and-place launch flow:
  - `cube_count` argument on `panda_pick_and_place.launch.py` (valid: `1..5`, or `0` to disable)
  - `panda_pick_and_place_cubes.launch.py` wrapper defaults to `cube_count:=3`
  - removed legacy split cube environment files (`pick_and_place_cubes_env_1.sdf`, `pick_and_place_cubes_env_2.sdf`) in favor of one base world + dynamic spawning
- Added three restricted-area obstacle worlds and launches:
  - `panda_restricted_corridor.launch.py`
  - `panda_restricted_pillars.launch.py`
  - `panda_restricted_chicane.launch.py`
- `view_rgbd.rviz` now shows external and wrist camera views together.

## Camera Topics

External RGB-D camera (from the world):
- `/camera/image_raw`
- `/camera/depth/image_raw`
- `/camera/depth/camera_info`

Spawned external RGB-D camera (second viewpoint):
- `/external_camera/image_raw`
- `/external_camera/depth/image_raw`
- `/external_camera/depth/camera_info`

Wrist RGB-D camera (from Panda URDF sensor):
- `/wrist_eye/image_raw`
- `/wrist_eye/depth/image_raw`
- `/wrist_eye/depth/camera_info`

Quick check:

```bash
ros2 topic list | rg "camera|wrist_eye"
```

For `panda_pick_and_place_cubes.launch.py` and restricted-area launches:
- world external camera bridge (`/camera/*`) is off by default
- spawned external camera (`/external_camera/*`) and wrist RGB-D camera are available

## Example Control Node

```bash
ros2 run panda example_position
```
