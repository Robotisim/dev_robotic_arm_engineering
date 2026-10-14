set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m2_01_fk_dh_xyz}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_02 module_02.launch.py backend:=mock rviz:=false
course_wait_topic /learning/fk_pose "$ARTIFACTS/fk_before.yaml"
course_wait_controllers arm_controller gripper_controller
ros2 launch arm_module_02 module_02.launch.py start_backend:=false dh_report:=true \
  2>&1 | tee "$ARTIFACTS/dh.txt"
bash 'Learning Path/tools/checks.sh' 2>&1 | tee "$ARTIFACTS/checks.txt"
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]' \
  2>&1 | tee "$ARTIFACTS/preview.txt"
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]' -p execute:=true \
  2>&1 | tee "$ARTIFACTS/execute.txt"
course_wait_topic /learning/fk_pose "$ARTIFACTS/fk_after.yaml"
course_expect_failure "$ARTIFACTS/unreachable.txt" 'converge|unreachable|IK|residual' \
  ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[3.0,0.0,2.0]'
