
# Module 2 Scope

## Module 2 title

Joint Control, Safe Motion, Calibration, and Forward Kinematics

## Core module description

Module 2 teaches students how to safely command robotic arm joints after the model has been created and validated. Students move from visualization to controlled joint motion using `ros2_control`, controller manager, joint trajectory commands, `/joint_states`, joint limits, and safe movement practices.

This module also introduces forward kinematics as the connection between joint angles and end-effector pose. Students learn that when joints move, the end-effector moves because of the chain of transforms defined in the robot model.

## Main story of Module 2

In Module 1, we described the robot.

Now the question becomes:

> If ROS 2 knows the robot structure, how do we safely move its joints and verify where the arm actually goes?

This module connects:

```text
joint command → controller → joint state → TF → end-effector pose
```

## Module 2 in scope

Module 2 should cover:

* `ros2_control` basic architecture
* controller manager
* joint state broadcaster
* joint trajectory controller
* controller YAML files
* `/joint_states`
* position command interface
* velocity and effort interfaces as concept previews
* joint names and joint order
* joint limits
* safe joint commands
* single-joint movement
* multi-joint movement
* safe ready pose
* stick arm controller bringup
* Panda controller bringup
* calibration and homing concepts
* joint offsets
* repeatability
* forward kinematics
* FK vs TF validation
* debugging flipped joint axes or wrong joint order

The Module 2 teaching map identifies `stick_arm/controllers.yaml` as the main beginner controller config, and Panda controller configs as the realistic scaling example.

## Module 2 out of scope

Do not teach these deeply in Module 2:

* inverse kinematics
* Cartesian path planning
* MoveIt planning
* collision-aware planning
* full dynamics
* torque-level control
* advanced PID tuning
* real hardware driver implementation

Mention dynamics only lightly when explaining why real robots may behave differently.

## Module 2 deliverables

By the end of Module 2, students should have:

* stick arm running in Gazebo with controllers
* Panda running in Gazebo with controllers
* ability to echo `/joint_states`
* ability to publish a safe joint trajectory
* understanding of joint naming and joint order
* a safe move strategy
* basic calibration/homing runbook
* FK calculation or FK explanation connected to TF
* FK vs TF validation results

Your Module 2 plan also expects a safe move tool, calibration documentation, and FK verification results.  The current teaching map notes that some of these tools may still need to be implemented as Module 2 teacher/student work.

## Module 2 success criteria

Students should be able to:

* launch the stick arm with controllers
* identify loaded controllers
* command joints safely
* read `/joint_states`
* understand command vs measured state
* launch Panda with controllers
* send Panda to a safe pose
* explain why joint limits matter
* explain what calibration and offsets mean
* explain FK conceptually
* compare computed FK or expected pose with TF
* debug common joint control problems

## Module 2 final scope decision

Keep Module 2 focused on this pipeline:

```text
Controller bringup → joint command → joint state feedback → safe limits → FK → TF validation
```

This module should not yet ask students to command the gripper by Cartesian target. That belongs in Module 3.

---