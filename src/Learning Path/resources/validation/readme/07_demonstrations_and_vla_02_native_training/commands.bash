set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_02_native_training}"
mkdir -p "$ARTIFACTS"

DATASET="$(mktemp -d "$ARTIFACTS/sample-XXXXXX")"
tar -xzf "$COURSE_ROOT/Learning Path/resources/sample_demonstrations.tar.gz" -C "$DATASET"
ros2 run arm_module_07 policy_train --ros-args -p dataset:="$DATASET" \
  -p epochs:=100 -p output:="$ARTIFACTS/policy.json" 2>&1 | tee "$ARTIFACTS/training.txt"
test -s "$ARTIFACTS/policy.json"
ros2 run arm_module_07 policy_train --ros-args -p self_test:=true \
  -p output:="$ARTIFACTS/synthetic_selftest.json" 2>&1 | tee "$ARTIFACTS/selftest.txt"
test -s "$ARTIFACTS/synthetic_selftest.json"
