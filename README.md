# Robotic Arm Engineering Workspace

ROS 2 Jazzy learning workspace for robotic arm simulation, control, motion,
perception, manipulation, evaluation, and VLA-style task routing.

## Package Layout

- `robotic_arm_description`: stick-arm and Panda URDF/Xacro plus robot meshes
- `robotic_arm_sim`: Gazebo worlds, object models, tables, camera scenes
- `robotic_arm_control`: controller YAML and controller demo nodes
- `robotic_arm_hardware`: real-hardware skeleton package for later modules
- `robotic_arm_bringup`: top-level launch files students run
- `robotic_arm_moveit_config`: Panda MoveIt config
- `robotic_arm_motion`: numbered IK and trajectory demos
- `robotic_arm_vision`: RGB-D segmentation, calibration checks, point-cloud snapshots
- `robotic_arm_manipulation`: pick/place executors plus VLM pose pipeline scripts
- `robotic_arm_evaluation`: logging, trial metadata, metrics skeleton

Old package names (`arm_sim_bringup`, `panda`, `motion`, `cube_segmentation`,
`pick_place`, and `util`) are deprecated wrappers kept for temporary compatibility.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luqman/repos/dev_robotic_arm_engineering
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
```

## Teaching Commands

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=panda

ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda

ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup pick_place_world.launch.py world:=cubes
ros2 launch robotic_arm_bringup multi_object_world.launch.py
ros2 launch robotic_arm_bringup vla_world.launch.py

ros2 launch robotic_arm_vision segmentation.launch.py
ros2 launch robotic_arm_manipulation pick_place_multi_object.launch.py
```

Motion demos:

```bash
ros2 run robotic_arm_motion motion_01_point_to_point
ros2 run robotic_arm_motion motion_02_straight_line
ros2 run robotic_arm_motion motion_03_circle_bump
ros2 run robotic_arm_motion motion_04_rectangle_shape
ros2 run robotic_arm_motion motion_05_workspace_reject
```

## Module Map

- Module 1: `robotic_arm_description`, `robotic_arm_bringup`, `robotic_arm_sim`
- Module 2: `robotic_arm_control`, `robotic_arm_hardware`
- Module 3: `robotic_arm_motion`
- Module 4: `robotic_arm_moveit_config`, `robotic_arm_bringup`
- Module 5: `robotic_arm_manipulation`
- Module 6: `robotic_arm_vision`, `robotic_arm_manipulation`
- Module 7: `robotic_arm_evaluation`, `robotic_arm_manipulation`
- Module 8: `robotic_arm_manipulation`

See `docs/Plan.md` and the module notes in `docs/` for the teaching flow.
