# arm_sim_bringup

Deprecated wrapper package.

Use `robotic_arm_bringup` for new teaching commands:

```bash
ros2 launch robotic_arm_bringup view_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm
ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda
```

The old launch names in this package forward to the new bringup package for now.
