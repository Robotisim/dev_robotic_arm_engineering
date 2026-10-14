set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m6_03_image_vlm}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_06 module_06.launch.py headless:=true rviz:=false
course_panda_ready
ollama show gemma4:12b 2>&1 | tee "$ARTIFACTS/model.txt"
grep -qi 'vision' "$ARTIFACTS/model.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=vlm -p model:=gemma4:12b -p interpret_only:=true \
  -p instruction:='pick the green cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/interpretation.txt"
grep -q '"colour":"green"' "$ARTIFACTS/interpretation.txt"
grep -q '"destination":"right"' "$ARTIFACTS/interpretation.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=vlm -p model:=gemma4:12b -p execute:=true \
  -p instruction:='pick the green cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/task.txt"
