# Module 2 · Project 01 — DH, FK and XYZ motion without MoveIt

Derive this robot's DH table, compare geometric checks, and execute a small XYZ goal using KDL/Eigen and the existing controllers.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 81, localhost discovery and the RoboFlex model in simulation. It never connects to real motors.

Four joints solve position only here. Orientation follows the solution. DH approximates rounded nearly parallel CAD axes; runtime FK/IK uses the original URDF chain. IK checks joint bounds, not collision-free paths.

## Nodes and code to read

`kinematics_node` publishes FK and serves `/learning/move_xyz`; `xyz_client` sends one goal. Read `arm_module_02/launch/module_02.launch.py`, `src/kinematics_node.cpp`, `src/xyz_client.cpp` and the shared core.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m2_01_fk_dh_xyz}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_02 module_02.launch.py backend:=mock rviz:=false
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_wait_topic /learning/fk_pose "$ARTIFACTS/fk_before.yaml"
course_wait_controllers arm_controller gripper_controller
ros2 launch arm_module_02 module_02.launch.py start_backend:=false dh_report:=true \
  2>&1 | tee "$ARTIFACTS/dh.txt"
bash 'Learning Path/tools/checks.sh' 2>&1 | tee "$ARTIFACTS/checks.txt"
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]' \
  2>&1 | tee "$ARTIFACTS/preview.txt"
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]' -p execute:=true \
  2>&1 | tee "$ARTIFACTS/execute.txt"
course_wait_topic /learning/fk_pose "$ARTIFACTS/fk_after.yaml"
course_expect_failure "$ARTIFACTS/unreachable.txt" 'converge|unreachable|IK|residual' \
  ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[3.0,0.0,2.0]'
```

## Expected result and evidence

Geometry and native-learning checks pass. The small target has sub-millimetre solver residual; the unreachable target fails with no trajectory dispatch. Keep the base/tool matrices with the DH table.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/02_kinematics_and_control/projects/01_fk_dh_xyz/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
