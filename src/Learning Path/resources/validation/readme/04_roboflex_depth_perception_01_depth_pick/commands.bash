set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m4_01_depth_pick}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_04 module_04.launch.py headless:=true rviz:=false
course_panda_ready
course_wait_topic /camera/camera_info "$ARTIFACTS/camera_info.yaml"
ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
  -p colour:=red -p attempts:=1 -p report:="$ARTIFACTS/preview.csv"
ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
  -p colour:=red -p attempts:=1 -p execute:=true \
  -p destination:='[0.45,0.28,0.025]' -p report:="$ARTIFACTS/pick.csv"
cat "$ARTIFACTS/pick.csv"
awk -F, 'NR > 1 {n++; if ($4 != 1) bad++} END {exit !(n == 1 && bad == 0)}' "$ARTIFACTS/pick.csv"
