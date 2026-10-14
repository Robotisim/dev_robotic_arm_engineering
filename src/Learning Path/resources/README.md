# Course resources

## Repository-specific references

- [DH numeric report](roboflex_dh.txt): generated from the canonical RoboFlex model, including base/tool transforms.
- [Validation record](VALIDATION.md): commands/results checked in this workspace and limitations.
- [README execution report](README_RUN_REPORT.md): results from running each standalone project guide, fixes and retained failed attempts.
- [Real-robot taught recipe](../03_moveit_and_pick_place/arm_module_03/config/taught_pick_place.yaml).
- [Panda world](../05_panda_depth_manipulation/panda_learning_sim/worlds/course.sdf) and [controller mapping](../05_panda_depth_manipulation/panda_learning_sim/config/controllers.yaml).

## Primary documentation, in course order

1. [ROS URDF tutorials](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html) — frames, joints and robot descriptions.
2. [Orocos KDL](https://github.com/orocos/orocos_kinematics_dynamics) and [ROS KDL parser](https://github.com/ros/kdl_parser) — geometry and solver foundations.
3. [ros2_control hardware components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html) — read/write responsibilities.
4. [MoveIt C++ MoveGroupInterface](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html) — planning, scene objects and execution.
5. [gz_ros2_control Jazzy](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html) — Gazebo/control integration.
6. [PCL tutorials](https://pcl.readthedocs.io/projects/tutorials/en/latest/) — voxel filtering, transforms and clustering.
7. [Ollama generate API](https://docs.ollama.com/api/generate) and [Qwen2.5-0.5B-Instruct model card](https://huggingface.co/Qwen/Qwen2.5-0.5B-Instruct) — local structured language inference.
8. [SmolVLA documentation](https://huggingface.co/docs/lerobot/smolvla) and [OpenVLA project](https://github.com/openvla/openvla) — optional next-stage study, not code implemented by the course MLP.

Installed ROS interfaces are the API authority for these examples; upstream main documentation may show a newer API. The course was built against Jazzy and Harmonic.

Simulator troubleshooting references: [Gazebo joint-limit recovery issue](https://github.com/gazebosim/gz-sim/issues/1684) and [gz_ros2_control joint-stop discussion](https://github.com/ros-controls/gz_ros2_control/issues/165). The course uses interior gripper targets and explicitly documents its early RoboFlex gravity simplification.
