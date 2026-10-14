# Module 7 · Project 02 — Train and evaluate the C++ policy

Train the 206-input, 64-hidden-unit policy on the included successful demonstrations and evaluate a held-out episode without needing a simulator.

## Before starting

Complete the [course setup and build](../../../README.md#setup-and-build) once. Use Bash terminals in this checkout; if your checkout is elsewhere, change the `cd` path below. Stop other course launches before starting this project. This guide uses ROS domain 82, localhost discovery and Panda simulation. It never connects to real motors.

No backend terminal is needed. The last sorted episode is held out; normalization uses training data only. Synthetic self-test checkpoints are deliberately rejected for robot control. To use your own recording, replace DATASET with the path in the preceding project's dataset_path.txt.

## Nodes and code to read

`policy_train` loads episodes and evaluates; `policy.hpp` implements features, normalization, Eigen MLP, backpropagation and Adam. Read `arm_module_07/src/policy_train.cpp`, then `include/arm_module_07/policy.hpp`.

## 1. Set up each terminal

Paste this in your terminal. The helper file only waits for observations, checks rejection/results and resets the fixed Panda scene; robot behavior stays in C++.

<!-- runnable: setup -->
```bash
set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_02_native_training}"
mkdir -p "$ARTIFACTS"
```

Artifacts are retained in `$ARTIFACTS`; subsequent manual runs replace reports with the same names. Recorder/training datasets use new directories. Use a new `ARTIFACTS` path to retain separate trials.

## 2. Run the project

This project is offline; no simulator or launch terminal is required.

Run this block after its setup block. Commands stop on unexpected errors. Expected rejection tests are checked explicitly and do not interrupt a passing project.

<!-- runnable: run -->
```bash
DATASET="$(mktemp -d "$ARTIFACTS/sample-XXXXXX")"
tar -xzf "$COURSE_ROOT/Learning Path/resources/sample_demonstrations.tar.gz" -C "$DATASET"
ros2 run arm_module_07 policy_train --ros-args -p dataset:="$DATASET" \
  -p epochs:=100 -p output:="$ARTIFACTS/policy.json" 2>&1 | tee "$ARTIFACTS/training.txt"
test -s "$ARTIFACTS/policy.json"
ros2 run arm_module_07 policy_train --ros-args -p self_test:=true \
  -p output:="$ARTIFACTS/synthetic_selftest.json" 2>&1 | tee "$ARTIFACTS/selftest.txt"
test -s "$ARTIFACTS/synthetic_selftest.json"
```

## Expected result and evidence

Training prints separate train/held-out losses and writes policy.json. The deterministic included dataset previously gave approximately 0.01056/0.04834 normalized MSE. The synthetic learning/checkpoint self-test also passes.

Read the generated files in `$ARTIFACTS` and the terminal log. If a C++ child launched by ROS exits unsuccessfully, `ros2 launch` can still return zero; the assertions above check observations/report contents as well.

## Stop, rerun and diagnose

No persistent process is started by this project. If a readiness check times out, inspect Terminal A first, confirm both terminals use the same ROS domain and have sourced setup, and check that only one course backend is running. Missing packages require rerunning the root build. A failed physical pick is an experiment failure, not a reason to mark its CSV successful.

To run these exact README blocks automatically and retain backend/exercise logs:

```bash
bash 'Learning Path/tools/run_project.sh' 'Learning Path/07_demonstrations_and_vla/projects/02_native_training/README.md'
```

The runner extracts the three marked Bash blocks from this file, starts only its own backend process group, runs the same assertions, and stops that group at exit. It does not flash, arm or start the real robot.

[Module concepts and extensions](../../README.md) · [All runnable projects](../../../PROJECTS.md)

