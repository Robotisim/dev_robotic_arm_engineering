# Pick and Place

## Nodes to run

Change path to yaml file as needed.

```bash
# Setup for ros2 bag play arm_single_object_20260223_094215_0.mcap

ros2 run pick_place object_detector_pcl_node --ros-args --params-file ros2_ws/src/dev_robotic_arm_engineering/src/pick_place/config/params.yaml

# setup for simulation to detect object and publish pose 
ros2 run pick_place object_detector_node --ros-args --params-file ros2_ws/src/dev_robotic_arm_engineering/src/pick_place/config/params.yaml
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 panda_wrist_eye_link Panda/panda_link7/panda_wrist_eye_sensor


# Executes pick and place action server, which subscribes to the object pose topic and sends goals to the arm action server
ros2 run pick_place pick_place_executor_node --ros-args --params-file ros2_ws/src/dev_robotic_arm_engineering/src/pick_place/config/params.yaml


```
