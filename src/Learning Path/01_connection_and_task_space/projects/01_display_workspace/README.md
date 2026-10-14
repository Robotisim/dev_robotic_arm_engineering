# Module 1 · Project 01 — Display joint angles and sample the workspace

Inspect the exact RoboFlex model, update its displayed joint angles, and generate 2,000 FK workspace samples. This display has no controller or motor connection.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 81, localhost discovery and the RoboFlex model in simulation. It never connects to real motors.

For a visual lesson, change Terminal A to `rviz:=true` and add `/learning/workspace` as MarkerArray with Transient Local durability. The exercise saves the marker message before stopping its publisher; run the workspace launch separately to keep it visible. A sampled envelope does not prove collision-free reachability.

## Nodes and code to read

`joint_lab` publishes synthetic states or samples FK. `robot_state_publisher` converts joint states to TF. Read `arm_module_01/launch/module_01.launch.py`, `launch/lab.launch.py`, then `src/joint_lab.cpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m1_01_display_workspace}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_01 module_01.launch.py backend:=display rviz:=false
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_wait_topic /joint_states "$ARTIFACTS/joints_before.yaml"
ros2 topic pub --once /learning/display_target sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4], position: [0.1, -0.4, -0.9, -0.3]}"
ros2 launch arm_module_01 lab.launch.py operation:=inspect \
  2>&1 | tee "$ARTIFACTS/inspect.txt"
grep -q 'joint_1: 0.1' "$ARTIFACTS/inspect.txt"
ros2 topic pub --once /learning/display_target sensor_msgs/msg/JointState \
  "{name: [joint_1], position: [2.0]}"
ros2 launch arm_module_01 lab.launch.py operation:=inspect \
  2>&1 | tee "$ARTIFACTS/after_invalid.txt"
grep -q 'joint_1: 0.1' "$ARTIFACTS/after_invalid.txt"
setsid env --default-signal=INT ros2 launch arm_module_01 lab.launch.py operation:=workspace > "$ARTIFACTS/workspace_node.log" 2>&1 &
workspace_pid=$!
trap 'kill -INT -- "-$workspace_pid" 2>/dev/null || true; wait "$workspace_pid" || true' EXIT
course_wait_topic /learning/workspace "$ARTIFACTS/workspace.yaml" transient_local
test "$(grep -c '^  - x:' "$ARTIFACTS/workspace.yaml")" -eq 2000
kill -INT -- "-$workspace_pid"
wait "$workspace_pid" || true
trap - EXIT
```

## Expected result and evidence

Both inspections show joint_1 = 0.1 rad; the invalid 2.0 rad input leaves it unchanged. workspace.yaml contains 2,000 points in base_link.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/01_connection_and_task_space/projects/01_display_workspace/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
