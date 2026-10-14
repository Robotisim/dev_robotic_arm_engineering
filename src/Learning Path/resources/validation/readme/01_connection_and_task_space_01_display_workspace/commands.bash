set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m1_01_display_workspace}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_01 module_01.launch.py backend:=display rviz:=false
course_wait_topic /joint_states "$ARTIFACTS/joints_before.yaml"
ros2 topic pub --once /learning/display_target sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4], position: [0.1, -0.4, -0.9, -0.3]}"
ros2 launch arm_module_01 lab.launch.py operation:=inspect \
  2>&1 | tee "$ARTIFACTS/inspect.txt"
grep -q 'joint_1: 0.1' "$ARTIFACTS/inspect.txt"
ros2 topic pub --once /learning/display_target sensor_msgs/msg/JointState \
  "{name: [joint_1], position: [2.0]}"
ros2 launch arm_module_01 lab.launch.py operation:=inspect \
  2>&1 | tee "$ARTIFACTS/after_invalid.txt"
grep -q 'joint_1: 0.1' "$ARTIFACTS/after_invalid.txt"
setsid env --default-signal=INT ros2 launch arm_module_01 lab.launch.py operation:=workspace > "$ARTIFACTS/workspace_node.log" 2>&1 &
workspace_pid=$!
trap 'kill -INT -- "-$workspace_pid" 2>/dev/null || true; wait "$workspace_pid" || true' EXIT
course_wait_topic /learning/workspace "$ARTIFACTS/workspace.yaml" transient_local
test "$(grep -c '^  - x:' "$ARTIFACTS/workspace.yaml")" -eq 2000
kill -INT -- "-$workspace_pid"
wait "$workspace_pid" || true
trap - EXIT
