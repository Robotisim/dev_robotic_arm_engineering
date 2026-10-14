# Module 6 · Project 01 — Ground a language command using rules

Establish an inspectable language baseline, reject ambiguous/unsupported requests, then execute a red/left command using depth geometry.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

Supported syntax is an affirmative pick/grab/move instruction containing exactly one known colour and an optional left/right destination. Rules do not understand unrestricted natural language.

## Nodes and code to read

`language_task` validates intent, resolves fresh colour detections and submits a Task action. Read `arm_module_06/launch/module_06.launch.py`, then the rules/validation/grounding branches of `src/language_task.cpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m6_01_rules}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_06 module_06.launch.py headless:=true rviz:=false
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_panda_ready
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=rules -p instruction:='pick the red cube and place it on the left' \
  2>&1 | tee "$ARTIFACTS/preview.txt"
course_expect_failure "$ARTIFACTS/ambiguous.txt" 'exactly one|Ambiguous|colour' \
  ros2 run arm_module_06 language_task --ros-args -p interpreter:=rules \
  -p interpret_only:=true -p instruction:='pick the red and blue cubes'
course_expect_failure "$ARTIFACTS/unsupported.txt" 'exactly one|colour' \
  ros2 run arm_module_06 language_task --ros-args -p interpreter:=rules \
  -p interpret_only:=true -p instruction:='pick the purple cube'
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=rules -p instruction:='pick the red cube and place it on the left' \
  -p execute:=true 2>&1 | tee "$ARTIFACTS/task.txt"
```

## Expected result and evidence

Interpretation selects red/left, preview plans the pregrasp, both invalid requests fail, and execution reports verified lift/place.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/06_language_guided_tasks/projects/01_rules/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)

