
# Module 1 Scope

## Module 1 title

ROS 2 Robotic Arm Model, URDF, TF, RViz, and Simulation Bringup

## Core module description

Module 1 teaches students how ROS 2 understands a robotic arm physically and visually. Students learn how a robotic arm is represented using links, joints, frames, URDF, Xacro, TF, and RViz. They first study a simple stick arm, then compare it with the Panda robot model, and finally bring the model into Gazebo.

The goal is not to control the robot deeply yet. The goal is to make students understand the robot model and validate that ROS 2 can correctly visualize and track the arm structure.

## Main story of Module 1

Before we can move, control, plan, or apply IK to a robotic arm, ROS 2 must first know:

```text
What is the robot made of?
Which links are connected?
Which joints can move?
Where are the frames?
What is the end-effector?
How does the robot appear in RViz and simulation?
```

So the main question of Module 1 is:

> How do we describe a robotic arm so ROS 2 understands its body?

## Module 1 in scope

Module 1 should cover:

* ROS 2 robotic arm workspace structure
* `robotic_arm_description`
* `robotic_arm_bringup`
* `robotic_arm_sim`
* simple stick arm model
* Panda model comparison
* URDF and Xacro basics
* links and joints
* parent-child structure
* joint types
* joint origins and axes
* joint limits
* visual, collision, and inertial tags
* robot frames
* TF tree
* `robot_description`
* `robot_state_publisher`
* `joint_state_publisher_gui`
* RViz visualization
* Gazebo model bringup
* basic validation and debugging

Module 1 should use the stick arm as the first teaching robot because it is simple and easier to debug, then use Panda as the realistic reference robot. Your teaching map also recommends this order: stick arm Xacro, stick arm RViz launch, Panda Xacro, Panda RViz launch, then Gazebo simulation.

## Module 1 out of scope

Do not teach these deeply in Module 1:

* inverse kinematics
* motion planning
* MoveIt
* detailed forward kinematics math
* DH parameters in depth
* controller tuning
* PID control
* torque/effort control
* full `ros2_control`
* real hardware motor communication
* task-space motion

These can be mentioned only as future motivation.

Example:

> Later, when we want the gripper to reach a target point, we will need FK, IK, task space, and motion planning. But all of those depend on having a correct robot model first.

## Module 1 deliverables

By the end of Module 1, students should have:

* a working ROS 2 workspace
* a stick arm robot description
* a Panda robot description available for comparison
* RViz bringup for stick arm
* RViz bringup for Panda
* Gazebo bringup for stick arm
* Gazebo bringup for Panda without deep controller focus
* a clean TF tree
* joint sliders moving the robot in RViz
* basic understanding of how URDF/Xacro becomes TF and RViz visualization

## Module 1 success criteria

Students should be able to:

* explain why robot description is required
* identify links and joints in a URDF/Xacro file
* explain parent-child link relationships
* explain what TF is doing
* launch the stick arm in RViz
* move joints using joint sliders
* launch Panda in RViz
* compare primitive geometry with mesh-based geometry
* bring the robot into Gazebo
* debug basic URDF/TF issues

## Module 1 final scope decision

Keep Module 1 focused on this pipeline:

```text
Robot structure → URDF/Xacro → robot_description → TF → RViz → Panda comparison → Gazebo bringup
```

Do not make hardware or controller implementation a required part of Module 1. Mention it as a preview only.

