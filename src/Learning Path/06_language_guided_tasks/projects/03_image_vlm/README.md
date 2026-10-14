# Module 6 · Project 03 — Use the simulator image with a local VLM

Send the current RGB image and instruction to a locally installed vision model, validate its JSON, then use depth for the actual 3D target.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

Additional prerequisite: Ollama serving at 127.0.0.1:11434 with `gemma4:12b` already installed. Check with `ollama list`; if missing, install Ollama using its official instructions and run `ollama pull gemma4:12b` before this project. Model download/runtime is external to ROS and is not bundled. This 12B model needs substantially more memory than the text example; `ollama show` must list vision support. A text-only model cannot replace it.

## Nodes and code to read

`language_task` performs local HTTP/image serialization in C++, validates JSON and dispatches the depth-grounded Task. Read `arm_module_06/src/language_task.cpp` and the module's interpreter table.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m6_03_image_vlm}"
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
ollama show gemma4:12b 2>&1 | tee "$ARTIFACTS/model.txt"
grep -qi 'vision' "$ARTIFACTS/model.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=vlm -p model:=gemma4:12b -p interpret_only:=true \
  -p instruction:='pick the green cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/interpretation.txt"
grep -q '"colour":"green"' "$ARTIFACTS/interpretation.txt"
grep -q '"destination":"right"' "$ARTIFACTS/interpretation.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=vlm -p model:=gemma4:12b -p execute:=true \
  -p instruction:='pick the green cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/task.txt"
```

## Expected result and evidence

Interpretation explicitly selects green/right; the execution result confirms physical lift/place. Check the interpretation independently of task success.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/06_language_guided_tasks/projects/03_image_vlm/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
