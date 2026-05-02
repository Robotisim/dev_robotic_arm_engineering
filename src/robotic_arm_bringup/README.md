# robotic_arm_bringup

Top-level launch package for students.

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup moveit_robot.launch.py robot:=panda
ros2 launch robotic_arm_bringup pick_place_world.launch.py world:=cubes
ros2 launch robotic_arm_bringup multi_object_world.launch.py
ros2 launch robotic_arm_bringup vla_world.launch.py
```
