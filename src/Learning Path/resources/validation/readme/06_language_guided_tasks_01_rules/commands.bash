set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m6_01_rules}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_06 module_06.launch.py headless:=true rviz:=false
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
