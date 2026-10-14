set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m7_01_record_demonstrations}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_04 module_04.launch.py headless:=true rviz:=false
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
