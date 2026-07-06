# Prompt: Implement `robot_arm_bringup`

You are implementing a ROS 2 package named `robot_arm_bringup`.

## Goal

Create one reusable bringup package that can display and control:

1. The existing Stick Arm model.
2. A Panda Arm model built in Xacro using STL meshes and named materials.
3. The same robot in RViz-only mode, Gazebo simulation mode, and later real-hardware mode.

Use `[ROS_DISTRO]` and modern Gazebo with `gz_ros2_control`. Do not use Gazebo Classic-only plugins.

## Important input

The Stick Arm URDF/Xacro already exists. Treat it as the source model. Do not redesign its mechanical geometry. First inspect and validate:

* Link names and parent-child hierarchy.
* Joint names, joint type, axis, origin, limits, and effort/velocity limits.
* `base_link`, flange/tool frame, and end-effector frame.
* Visual, collision, and inertial elements.
* TF tree correctness from `base_link` to `tool0`.

## Required package structure

```text
robot_arm_bringup/
├── description/
│   ├── stick_arm.urdf.xacro
│   ├── panda_arm.urdf.xacro
│   ├── common_materials.xacro
│   ├── stick_arm.ros2_control.xacro
│   └── panda_arm.ros2_control.xacro
├── meshes/
│   ├── panda/
│   │   ├── visual/
│   │   └── collision/
│   └── stick_arm/
├── config/
│   ├── stick_arm_controllers.yaml
│   ├── panda_arm_controllers.yaml
│   ├── stick_arm.rviz
│   └── panda_arm.rviz
├── launch/
│   ├── robot_stick_arm_rviz.launch.py
│   ├── robot_panda_rviz.launch.py
│   ├── gazebo.launch.py
│   └── robot_simulation.launch.py
├── worlds/
│   └── empty_arm_world.sdf
└── package.xml
```

## Stick Arm URDF validation work

Create a short markdown document named `docs/stick_arm_model_validation.md` containing:

* Full link list.
* Parent → child joint list.
* Joint type: revolute, continuous, prismatic, fixed.
* Joint axis and limits.
* Frame convention: `base_link`, `link_n`, `flange`, `tool0`.
* A rendered TF tree image or command output.
* Any missing collision, inertia, or limits found in the original model.

The arm must have a clean chain such as:

```text
base_link
  └── joint_1 → link_1
      └── joint_2 → link_2
          └── ...
              └── joint_n → tool0
```

## Panda Xacro requirements

Create `panda_arm.urdf.xacro` with:

* Separate Xacro macros for links, joints, materials, mesh paths, and ros2_control.
* STL visual meshes and simplified collision meshes.
* Named materials such as `panda_white`, `panda_black`, `panda_dark_gray`.
* Correct visual and collision origins.
* Inertial blocks for every movable link.
* Joint limits and safety limits.
* `base_link`, arm chain, flange, and `tool0`.
* Mesh paths resolved with `package://robot_arm_bringup/...`.

Use only assets that are already available in the workspace or legally obtained from the installed robot-description package.

## RViz-only launches

Implement:

```bash
ros2 launch robot_arm_bringup robot_stick_arm_rviz.launch.py
ros2 launch robot_arm_bringup robot_panda_rviz.launch.py
```

Each launch must start:

* `robot_state_publisher`
* `joint_state_publisher_gui`
* `rviz2`

Acceptance criteria:

* Moving a slider updates the robot in RViz.
* TF tree is complete.
* Robot has no broken mesh paths.
* Joint axes match expected physical motion.
* Collision geometry can be enabled in RViz for validation.

## ros2_control design

Use the same controller names for Stick, Panda, Gazebo, and real robot:

```text
/controller_manager
/joint_state_broadcaster
/arm_trajectory_controller
/arm_trajectory_controller/follow_joint_trajectory
```

Create controller YAML files with:

* `joint_state_broadcaster/JointStateBroadcaster`
* `joint_trajectory_controller/JointTrajectoryController`
* Position command interfaces.
* Position and velocity state interfaces.
* Explicit list of arm joints in correct kinematic order.

Three hardware modes must be supported:

```text
mode:=rviz
mode:=gazebo
mode:=hardware
```

* `rviz`: mock ros2_control hardware interface for controller testing without Gazebo.
* `gazebo`: `gz_ros2_control` hardware bridge attached to simulated joints.
* `hardware`: placeholder for a future real hardware interface plugin. Do not fake hardware communication; keep this mode disabled until the robot driver protocol is known.

## Gazebo requirements

Implement:

```bash
ros2 launch robot_arm_bringup gazebo.launch.py robot:=stick
ros2 launch robot_arm_bringup gazebo.launch.py robot:=panda
```

Gazebo launch must:

* Start an empty world.
* Spawn the selected robot.
* Load robot-specific ros2_control configuration.
* Spawn `joint_state_broadcaster`.
* Spawn `arm_trajectory_controller`.
* Publish simulated joint states back to ROS 2.
* Allow RViz to display the same simulated robot and TF tree.

Add Gazebo properties to every physical link:

* Collision geometry.
* Inertial values.
* Mass.
* Friction/contact settings where required.
* Gravity enabled.
* Damping where required.
* ros2_control joint mappings.

## Deliverables

1. Both robots visible in RViz.
2. Both robots spawned in Gazebo.
3. Controllers listed as active through `ros2 control list_controllers`.
4. A trajectory action endpoint available for each robot.
5. TF tree and joint-state checks documented.
6. One concise README with exact launch commands and expected topics/actions.

## Do not do

* Do not embed FK, IK, path planning, or shape drawing inside this package.
* Do not command Gazebo joints through separate Gazebo-native controllers when ros2_control owns the same joints.
* Do not hard-code robot-specific joint names inside generic launch logic.
