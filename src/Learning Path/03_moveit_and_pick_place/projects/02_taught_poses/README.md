# Module 3 · Project 02 — Save poses and run a taught pick/place sequence

Create four reproducible mock teaching poses, persist them to YAML, and exercise the complete open/approach/close/transfer/release recipe. These small-angle fixtures test sequencing; they are not physical pick coordinates.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 81, localhost discovery and the RoboFlex model in simulation. It never connects to real motors.

On real RoboFlex, replace these fixtures by appropriate commanded poses from your existing control workflow, and check the gripper's mechanical open/close mapping. No hand-guiding or encoder feedback is assumed. Do not copy these mock fixtures as workbench locations.

## Nodes and code to read

`task_client` teaches and recalls; `motion_server` serializes poses and dispatches the recipe. Read `arm_module_03/config/taught_pick_place.yaml`, `src/task_client.cpp`, then saved-pose/recipe branches in `src/motion_server.cpp`.

## 1. Set up each terminal

Paste this in both Terminal A and Terminal B. The helper file only waits for observations, checks rejection/results and resets the fixed Panda scene; robot behavior stays in C++.

<!-- runnable: setup -->
```bash
set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m3_02_taught_poses}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_03 module_03.launch.py backend:=mock rviz:=false allow_execution:=true pose_store:="$ARTIFACTS/poses.yaml"
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_wait_topic /learning/stage "$ARTIFACTS/stage.yaml" transient_local
course_wait_controllers arm_controller gripper_controller
# Pairs are saved-name and joint_1 angle; all remaining joints stay near Arm Up.
while read -r name angle; do
  ros2 run arm_module_03 task_client --ros-args -p operation:=joints \
    -p joints:="[$angle,-0.3839724,-0.9250245,-0.3665191]" -p execute:=true
  ros2 run arm_module_03 task_client --ros-args -p operation:=save -p target:="$name"
done <<'POSES'
pre_pick 0.04
pick 0.05
pre_place 0.07
place 0.08
POSES
cat "$ARTIFACTS/poses.yaml"
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=pre_pick
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=pre_pick -p execute:=true
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml" \
  2>&1 | tee "$ARTIFACTS/preview.txt"
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe -p execute:=true \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml" \
  2>&1 | tee "$ARTIFACTS/execute.txt"
course_expect_failure "$ARTIFACTS/missing_pose.txt" 'pose|Unknown|found' \
  ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=not_taught
```

## Expected result and evidence

poses.yaml contains the four names, robot identity and joint order. The 11-step recipe completes; an unknown saved name fails. This proves the teaching/script interface, not a physical grasp.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/03_moveit_and_pick_place/projects/02_taught_poses/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
