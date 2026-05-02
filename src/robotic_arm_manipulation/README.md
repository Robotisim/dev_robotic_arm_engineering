# robotic_arm_manipulation

Pick/place and task-level manipulation package for Modules 5-8.

This package owns:

- fixed and multi-object pick/place executors
- gripper behavior helpers
- object pose consumers that use `/segmentation/object_poses`
- VLM/PaliGemma pose pipeline scripts under `scripts/vlm_pose_pipeline/`
- LLM/VLM visual pick-and-place helpers

## Run

```bash
ros2 launch robotic_arm_bringup multi_object_world.launch.py

ros2 launch robotic_arm_vision segmentation.launch.py \
  use_sim_time:=true \
  publish_header_alias_tf:=true \
  publish_optical_tf:=true

ros2 launch robotic_arm_manipulation pick_place_multi_object.launch.py
```

Individual executables are available as:

```bash
ros2 run robotic_arm_manipulation object_detector_node
ros2 run robotic_arm_manipulation object_detector_pcl_node
ros2 run robotic_arm_manipulation pick_place_executor_node
ros2 run robotic_arm_manipulation pick_place_multi_executor_node
```

Deprecated wrappers exist in the old `pick_place` package during migration.
