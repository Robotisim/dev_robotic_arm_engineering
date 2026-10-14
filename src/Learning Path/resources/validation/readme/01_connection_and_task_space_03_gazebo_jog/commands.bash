set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=81
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m1_03_gazebo_jog}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_01 module_01.launch.py backend:=gazebo rviz:=false headless:=true
course_wait_clock
course_wait_topic /joint_states "$ARTIFACTS/joints_before.yaml"
course_wait_controllers arm_controller gripper_controller
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05 use_sim_time:=true \
  2>&1 | tee "$ARTIFACTS/preview.txt"
grep -q 'Validated jog preview' "$ARTIFACTS/preview.txt"
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05 execute:=true use_sim_time:=true \
  2>&1 | tee "$ARTIFACTS/execute.txt"
ros2 launch arm_module_01 lab.launch.py operation:=inspect use_sim_time:=true \
  2>&1 | tee "$ARTIFACTS/inspect.txt"
awk '{for (i=1; i<NF; i++) if ($i == "joint_1:") {found=1; if ($(i+1) < 0.045 || $(i+1) > 0.055) bad=1}} END {exit !(found && !bad)}' "$ARTIFACTS/inspect.txt"
