# Module 5 · Project 03 — Compare 0°, 45° and 90° grasps

Use fresh depth observations and the same task server for three measured transfers. Each colour uses a different yaw. The jaw span is side × (|cos yaw| + |sin yaw|); the diagonal grasp needs a wider jaw opening.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

Baseline: one trial, original cube positions, randomize=false. After completing it, optionally set `/scene_manager randomize true` and increase `trials` to study robustness. Randomized experiments may fail and must be reported separately from this baseline. Never reset while a task is active or while the arm is touching objects.

## Nodes and code to read

`project_runner` sequences colours/destinations/yaw and writes CSV; `scene_manager` owns reset/evaluation. Read `panda_learning_sim/src/project_runner.cpp`, `src/scene_manager.cpp`, `worlds/course.sdf`, and `arm_module_04/launch/module_05.launch.py` in the preceding module.

## 1. Set up each terminal

Paste this in both Terminal A and Terminal B. The helper file only waits for observations, checks rejection/results and resets the fixed Panda scene; robot behavior stays in C++.

<!-- runnable: setup -->
```bash
set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m5_03_grasp_yaw}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_04 module_05.launch.py headless:=true rviz:=false
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_panda_ready
ros2 run panda_learning_sim project_runner --ros-args -p use_sim_time:=true \
  -p project:=grasps -p execute:=true -p trials:=1 -p report:="$ARTIFACTS/grasps.csv"
cat "$ARTIFACTS/grasps.csv"
course_check_transfers "$ARTIFACTS/grasps.csv" 3
```

## Expected result and evidence

All three CSV rows have success=1 for the fixed-scene baseline. Planner success alone does not satisfy this check; the task verifies lift and destination.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

Earlier README verification encountered manipulation timeouts. The callback executor and planning-scene client lifetime were subsequently revised; the [execution report](../../../resources/README_RUN_REPORT.md) records the failures and retests. Keep failed logs in your experiment results, stop the entire workcell before restarting, and do not interpret one passing trial as a reliability estimate.

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/05_panda_depth_manipulation/projects/03_grasp_yaw/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
