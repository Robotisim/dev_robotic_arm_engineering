set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m3_01_moveit_recipe}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_03 module_03.launch.py backend:=mock rviz:=false headless:=true allow_execution:=true pose_store:="$ARTIFACTS/poses.yaml"
course_wait_topic /learning/stage "$ARTIFACTS/stage.yaml" transient_local
course_wait_controllers arm_controller gripper_controller
ros2 run arm_module_03 task_client --ros-args -p use_sim_time:=false -p operation:=named -p target:='Arm Up'
ros2 run arm_module_03 task_client --ros-args -p use_sim_time:=false -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/real_motion_recipe.yaml" \
  2>&1 | tee "$ARTIFACTS/preview.txt"
ros2 run arm_module_03 task_client --ros-args -p use_sim_time:=false -p operation:=recipe -p execute:=true \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/real_motion_recipe.yaml" \
  2>&1 | tee "$ARTIFACTS/execute.txt"
course_expect_failure "$ARTIFACTS/out_of_range.txt" 'limit|bound|invalid' \
  ros2 run arm_module_03 task_client --ros-args -p use_sim_time:=false -p operation:=joints \
  -p joints:='[2.0,-0.3839724,-0.9250245,-0.3665191]' -p execute:=true
