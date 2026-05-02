
# Module 2 Scope

## Module 2 title

Joint Control, Safe Motion, Calibration, and Forward Kinematics

## Instructor Flow Design Answers

**Questions Based Instructor Flow Design?**

* Start with the question: once the model exists, how do joints move safely?
* Teach in the rhythm `controller config -> launch -> command -> joint state -> TF result`.
* Keep lessons focused on safe joint-space control before Cartesian targets or planning.

**What is this module about?**

* This module is about commanding robot joints and verifying the motion through feedback.
* Students learn `ros2_control`, controller manager, joint trajectory controllers, joint limits, and `/joint_states`.
* Forward kinematics is introduced as the connection between joint angles and end-effector pose.

**What final project outcome should students achieve?**

* Students should run stick arm and Panda controllers in Gazebo.
* Students should send a safe joint command and confirm the measured joint state changed.
* Students should explain how joint motion updates TF and the end-effector pose.

**What concepts must the instructor explain?**

* Explain controller manager, joint state broadcaster, trajectory controller, command interfaces, and state interfaces.
* Explain joint names, joint order, joint limits, calibration, homing, offsets, and safe ready poses.
* Explain FK as `joint angles -> transforms -> end-effector pose`, then compare it with TF.

**What exact code/package/files should be shown?**

* Show `robotic_arm_control/config/stick_arm/controllers.yaml`.
* Show `robotic_arm_control/config/panda/controllers_position.yaml`, `initial_joint_positions.yaml`, and `joint_limits.yaml`.
* Show `robotic_arm_bringup/launch/sim_robot.launch.py` and `robotic_arm_control/example_position.cpp`.

**What commands should be run?**

* Run `ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=stick_arm`.
* Run `ros2 control list_controllers` and `ros2 topic echo /joint_states`.
* Run `ros2 launch robotic_arm_bringup sim_robot.launch.py robot:=panda` for the realistic controller comparison.

**What visual/demo result should appear?**

* Gazebo should show the robot spawned with active controllers.
* RViz or TF should show the arm pose changing as joint states update.
* Terminal output should show loaded controllers and live joint positions.

**What mistakes should the instructor avoid?**

* Do not jump into IK or Cartesian planning before students understand joint commands.
* Do not ignore joint order, because correct names with wrong order still create wrong motion.
* Do not present calibration as optional trivia; connect offsets and homing to repeatable motion.

**What should the student understand by the end of each lesson?**

* Students should know which controller or topic caused the observed joint motion.
* Students should connect command, measured state, TF, and FK in one feedback loop.
* Students should be able to rerun the demo and debug a missing controller or wrong joint name.

**Package Names And Responsibilities?**

* `robotic_arm_control` owns controller YAML, joint limits, and controller examples.
* `robotic_arm_bringup` owns the student launch commands for controlled simulation.
* `robotic_arm_description` supplies the joint names and limits that control depends on.

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
