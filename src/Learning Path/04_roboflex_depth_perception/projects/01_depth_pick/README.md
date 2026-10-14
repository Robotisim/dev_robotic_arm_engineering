# Module 4 · Project 01 — Find a coloured cube with depth and pick it

Start the complete Panda workcell, inspect camera/object/evaluation streams, preview a depth-derived pregrasp, then physically lift and place the red cube.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

The camera pose/intrinsics are specified by simulation; physical calibration is unnecessary. The Harmonic packed cloud is explicitly interpreted in camera_link before TF to panda_link0. Segmentation supports saturated 4 cm cubes on this table, not arbitrary objects.

## Nodes and code to read

`depth_objects` transforms/filters/segments RGB point clouds and updates collision geometry. `perception_pick` resolves colour and requests a task. `scene_manager` supplies reset and ground truth only for metrics. Read `arm_module_04/launch/module_04.launch.py`, `src/depth_objects.cpp`, then `src/perception_pick.cpp`.

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
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m4_01_depth_pick}"
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
course_wait_topic /camera/camera_info "$ARTIFACTS/camera_info.yaml"
ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
  -p colour:=red -p attempts:=1 -p report:="$ARTIFACTS/preview.csv"
ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
  -p colour:=red -p attempts:=1 -p execute:=true \
  -p destination:='[0.45,0.28,0.025]' -p report:="$ARTIFACTS/pick.csv"
cat "$ARTIFACTS/pick.csv"
awk -F, 'NR > 1 {n++; if ($4 != 1) bad++} END {exit !(n == 1 && bad == 0)}' "$ARTIFACTS/pick.csv"
```

## Expected result and evidence

objects.yaml contains red/green/blue detections in panda_link0. pick.csv reports one successful physical lift/place. Preview means pregrasp planning only.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

After Terminal B finishes, press Ctrl+C in Terminal A and wait for shutdown before starting the next project. A fresh launch restores the initial simulated world. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/04_roboflex_depth_perception/projects/01_depth_pick/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)

