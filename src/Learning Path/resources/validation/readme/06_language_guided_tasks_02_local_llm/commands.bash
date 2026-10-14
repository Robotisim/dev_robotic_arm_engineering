set -eo pipefail
cd /home/luqman/repos/RoboFlexv2.1
export ROS_DOMAIN_ID=82
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source 'Learning Path/tools/setup.bash'
source 'Learning Path/tools/project_helpers.bash'
ARTIFACTS="${ARTIFACTS:-$COURSE_ROOT/Learning Path/artifacts/m6_02_local_llm}"
mkdir -p "$ARTIFACTS"
ros2 launch arm_module_06 module_06.launch.py headless:=true rviz:=false
course_panda_ready
ollama show qwen2.5:0.5b 2>&1 | tee "$ARTIFACTS/model.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=llm -p model:=qwen2.5:0.5b -p interpret_only:=true \
  -p instruction:='pick the blue cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/interpretation.txt"
grep -q '"colour":"blue"' "$ARTIFACTS/interpretation.txt"
grep -q '"destination":"right"' "$ARTIFACTS/interpretation.txt"
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p interpreter:=llm -p model:=qwen2.5:0.5b -p execute:=true \
  -p instruction:='pick the blue cube and put it on the right' \
  2>&1 | tee "$ARTIFACTS/task.txt"
