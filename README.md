# Robotic Arm Engineering Workspace
### TODO
- Depth Camera not working
- Perform Depth Camera based pick and place using external camera and internal camera
- Execute a path from obstacles

### Introduction
ROS 2 Jazzy workspace for simulation and control demos with:
- `arm_sim_bringup`: custom 4DOF stick arm + Panda wrapper launch
- `panda`: Panda simulation/control package (Gazebo + ros2_control + MoveIt)
- `motion`: numbered Panda trajectory demos and MoveIt pose-goal node

## Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy

## Package Docs

- [arm_sim_bringup README](src/arm_sim_bringup/README.md): stick-arm and Panda bringup launch guidance
- [panda README](src/panda/README.md): Panda worlds, controllers, cameras, MoveIt, and environment launch files
- [motion README](src/motion/README.md): motion demos, launch arguments, and goal-sending node usage

## Dependencies (rosdep)

Dependencies are declared in package manifests (`package.xml`) and should be installed with `rosdep`:

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luqman/repos/dev_robotic_arm_engineering
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd /home/luqman/repos/dev_robotic_arm_engineering
colcon build --symlink-install
source install/setup.bash
```

## Launch Guides

### Stick Arm Bringup
Use the package guide: [arm_sim_bringup README](src/arm_sim_bringup/README.md)

### Panda Simulation and MoveIt Environments
Use the package guide: [panda README](src/panda/README.md)

### Motion Demos and Pose Goals using IK
Use the package guide: [motion README](src/motion/README.md)

## Troubleshooting

- Controller activation, camera topics, and cleanup are documented in [panda README](src/panda/README.md).
- If Conda Python interferes with ROS builds, rebuild with system Python (`/usr/bin/python3`).
