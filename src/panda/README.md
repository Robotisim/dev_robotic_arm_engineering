# panda

Deprecated wrapper package.

Robot description files now live in `robotic_arm_description`, simulation assets in
`robotic_arm_sim`, controllers in `robotic_arm_control`, MoveIt config in
`robotic_arm_moveit_config`, and top-level launches in `robotic_arm_bringup`.

Use:

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda
```
