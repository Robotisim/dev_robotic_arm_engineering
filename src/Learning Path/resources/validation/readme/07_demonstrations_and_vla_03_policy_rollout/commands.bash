set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_03_policy_rollout}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_04 module_04.launch.py headless:=true rviz:=false
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
