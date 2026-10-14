set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m3_02_taught_poses}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_03 module_03.launch.py backend:=mock rviz:=false allow_execution:=true pose_store:="$ARTIFACTS/poses.yaml"
course_wait_topic /learning/stage "$ARTIFACTS/stage.yaml" transient_local
course_wait_controllers arm_controller gripper_controller
# Pairs are saved-name and joint_1 angle; all remaining joints stay near Arm Up.
while read -r name angle; do
  ros2 run arm_module_03 task_client --ros-args -p operation:=joints \
    -p joints:="[$angle,-0.3839724,-0.9250245,-0.3665191]" -p execute:=true
  ros2 run arm_module_03 task_client --ros-args -p operation:=save -p target:="$name"
done <<'POSES'
pre_pick 0.04
pick 0.05
pre_place 0.07
place 0.08
POSES
cat "$ARTIFACTS/poses.yaml"
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=pre_pick
ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=pre_pick -p execute:=true
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml" \
  2>&1 | tee "$ARTIFACTS/preview.txt"
ros2 run arm_module_03 task_client --ros-args -p operation:=recipe -p execute:=true \
  -p target:="$COURSE_ROOT/Learning Path/03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml" \
  2>&1 | tee "$ARTIFACTS/execute.txt"
course_expect_failure "$ARTIFACTS/missing_pose.txt" 'pose|Unknown|found' \
  ros2 run arm_module_03 task_client --ros-args -p operation:=load -p target:=not_taught
