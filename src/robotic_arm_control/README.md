# robotic_arm_control

Owns controller configuration and control examples.

- `config/stick_arm/controllers.yaml`
- `config/panda/controllers_position.yaml`
- `config/panda/controllers_velocity.yaml`
- `config/panda/controllers_effort.yaml`
- `config/panda/joint_limits.yaml`

Top-level launch files in `robotic_arm_bringup` choose which controller file to
pass into each robot description.
