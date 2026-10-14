# Module 1 · Project 02 — Mock controller angle commands

Preview and execute a bounded single-joint goal through the existing trajectory-controller contract.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 81, localhost discovery and the RoboFlex model in simulation. It never connects to real motors.

GenericSystem mirrors commanded state. This is a controller integration test; it does not model motor dynamics.

## Nodes and code to read

`joint_lab` owns one goal; `arm_controller` executes it. Read `arm_module_01/launch/module_01.launch.py`, `launch/lab.launch.py`, then `src/joint_lab.cpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m1_02_mock_jog}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_01 module_01.launch.py backend:=mock rviz:=false headless:=true
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_wait_topic /joint_states "$ARTIFACTS/joints_before.yaml"
course_wait_controllers arm_controller gripper_controller
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05 use_sim_time:=false \
  2>&1 | tee "$ARTIFACTS/preview.txt"
grep -q 'Validated jog preview' "$ARTIFACTS/preview.txt"
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05 execute:=true use_sim_time:=false \
  2>&1 | tee "$ARTIFACTS/execute.txt"
ros2 launch arm_module_01 lab.launch.py operation:=inspect use_sim_time:=false \
  2>&1 | tee "$ARTIFACTS/inspect.txt"
awk '{for (i=1; i<NF; i++) if ($i == "joint_1:") {found=1; if ($(i+1) < 0.045 || $(i+1) > 0.055) bad=1}} END {exit !(found && !bad)}' "$ARTIFACTS/inspect.txt"
```

## Expected result and evidence

Preview confirms the target without dispatching. The final joint_1 reading is within 0.005 rad of 0.05.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/01_connection_and_task_space/projects/02_mock_jog/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
