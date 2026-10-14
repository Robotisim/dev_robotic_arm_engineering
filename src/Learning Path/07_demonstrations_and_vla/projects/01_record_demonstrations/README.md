# Module 7 · Project 01 — Record two successful expert demonstrations

Capture two independent red/left tasks as timestamped images, state, structured instruction and future controller targets. Verify the expert outcome before accepting each episode.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

Each run creates a fresh dataset directory. A nonzero expert result stops collection; preserve its failed episode separately rather than silently mixing it into successful behavior cloning. The two fixed-scene episodes verify the pipeline, not generalization. Change colour/destination consistently in recorder and expert for further data.

## Nodes and code to read

`episode_recorder` aligns RGB/state to controller targets about 0.2 s in the future; `policy_train` splits by whole episode. Read `arm_module_07/src/episode_recorder.cpp`, then `src/policy_train.cpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_01_record_demonstrations}"
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
DATASET="$(mktemp -d "$ARTIFACTS/dataset-XXXXXX")"
printf '%s\n' "$DATASET" 2>&1 | tee "$ARTIFACTS/dataset_path.txt"
for episode in episode_001 episode_002; do
  course_panda_ready
  setsid env --default-signal=INT ros2 run arm_module_07 episode_recorder --ros-args -p use_sim_time:=true \
    -p episode:="$DATASET/$episode" -p colour:=red \
    -p destination:='[0.45,0.28,0.025]' > "$ARTIFACTS/$episode.log" 2>&1 &
  recorder_pid=$!
  trap 'kill -INT -- "-$recorder_pid" 2>/dev/null || true; wait "$recorder_pid" || true' EXIT
  # Wait for the recorder's service; it must be alive before expert motion.
  for attempt in $(seq 1 30); do
    if grep -q 'Recording moving task stages' "$ARTIFACTS/$episode.log" && \
        timeout 10 ros2 service list > "$ARTIFACTS/services.txt" && \
        grep -qx /learning/finish_episode "$ARTIFACTS/services.txt"; then
      break
    fi
    kill -0 "$recorder_pid"
    sleep 1
  done
  grep -q 'Recording moving task stages' "$ARTIFACTS/$episode.log"
  grep -qx /learning/finish_episode "$ARTIFACTS/services.txt"
  ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
    -p colour:=red -p destination:='[0.45,0.28,0.025]' -p execute:=true \
    -p attempts:=1 -p report:="$ARTIFACTS/$episode.csv"
  ros2 service call /learning/finish_episode std_srvs/srv/Trigger '{}' \
    2>&1 | tee "$ARTIFACTS/$episode.finish.txt"
  grep -q 'success=True' "$ARTIFACTS/$episode.finish.txt"
  kill -INT -- "-$recorder_pid"
  wait "$recorder_pid"
  trap - EXIT
  test "$(wc -l < "$DATASET/$episode/episode.jsonl")" -ge 32
done
ros2 run arm_module_07 policy_train --ros-args -p dataset:="$DATASET" \
  -p epochs:=100 -p output:="$ARTIFACTS/recorded_policy.json" \
  2>&1 | tee "$ARTIFACTS/training.txt"
```

## Expected result and evidence

Both expert CSV files succeed; each episode has at least 32 labeled samples, and native training can load both episodes. dataset_path.txt identifies the new dataset and recorded_policy.json is its checkpoint.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/07_demonstrations_and_vla/projects/01_record_demonstrations/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)
