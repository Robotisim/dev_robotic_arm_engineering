# Module 7 · Project 03 — Preview and dispatch a bounded learned policy

Use the included camera-trained checkpoint with live Panda observations: preview 50 predictions, then execute five bounded policy steps. Compare pipeline completion with actual manipulation success.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

This is an educational RGB + structured-language + state MLP, not pretrained SmolVLA/OpenVLA. After this baseline, substitute your checkpoint and explicitly increase steps to study full rollouts. Collision/contact rejection and failed picks are valid experimental outcomes that must be counted. Robust learned picking is not established by the two included demonstrations.

## Nodes and code to read

`policy_run` owns policy dispatch, validates fresh observations, bounds increments, queries MoveIt collisions and sends controller goals. Read `arm_module_07/launch/module_07.launch.py`, `src/policy_run.cpp` and `include/arm_module_07/policy.hpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_03_policy_rollout}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Terminal A — Start the backend

Keep this terminal running. The next section waits for the required observations; a launch banner alone does not mean the controllers/camera are ready.

<!-- runnable: launch -->
```bash
ros2 launch arm_module_04 module_04.launch.py headless:=true rviz:=false
```

## 3. Terminal B — Run the project

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
course_panda_ready
# Empty old reports so a failed launch cannot pass using a previous run's CSV.
: > "$ARTIFACTS/preview.csv"
: > "$ARTIFACTS/rollout.csv"
ros2 launch arm_module_07 module_07.launch.py mode:=run \
  checkpoint:="$COURSE_ROOT/Learning Path/resources/sample_policy.json" \
  colour:=red execute:=false steps:=50 report:="$ARTIFACTS/preview.csv" \
  2>&1 | tee "$ARTIFACTS/preview.txt"
awk -F, 'NR == 2 {ok=($1 == 0 && $2 == 50)} END {exit !ok}' "$ARTIFACTS/preview.csv"
ros2 launch arm_module_07 module_07.launch.py mode:=run \
  checkpoint:="$COURSE_ROOT/Learning Path/resources/sample_policy.json" \
  colour:=red execute:=true steps:=5 report:="$ARTIFACTS/rollout.csv" \
  2>&1 | tee "$ARTIFACTS/rollout.txt"
awk -F, 'NR == 2 {ok=($1 == 1 && $2 == 5)} END {exit !ok}' "$ARTIFACTS/rollout.csv"
cat "$ARTIFACTS/preview.csv" "$ARTIFACTS/rollout.csv"
```

## Expected result and evidence

Reports contain 50 preview steps and five dispatched steps with latency and physical metrics. manipulation_success is expected to be 0 for this short baseline; a completed policy loop is not a completed pick.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/07_demonstrations_and_vla/projects/03_policy_rollout/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
