set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m5_02_stack}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_04 module_05.launch.py headless:=true rviz:=false
course_panda_ready
ros2 run panda_learning_sim project_runner --ros-args -p use_sim_time:=true \
  -p project:=stack -p execute:=true -p trials:=1 -p report:="$ARTIFACTS/stack.csv"
cat "$ARTIFACTS/stack.csv"
course_check_transfers "$ARTIFACTS/stack.csv" 3
