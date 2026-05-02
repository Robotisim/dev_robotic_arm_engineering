

# Module 3 Scope

## Module 3 title

Inverse Kinematics, Cartesian Trajectories, Workspace Validation, and Dynamics Basics

## Instructor Flow Design Answers

**Questions Based Instructor Flow Design?**

* Start with the question: how do we move the gripper to a Cartesian target?
* Teach in the rhythm `target -> workspace check -> IK -> waypoints -> marker preview -> trajectory`.
* Keep MoveIt as a comparison after students see the custom IK and trajectory pipeline.

**What is this module about?**

* This module is about moving from joint-space commands to task-space motion.
* Students learn IK, Cartesian targets, waypoints, trajectory timing, marker previews, and workspace rejection.
* Dynamics basics are included only as intuition for why real robot motion is harder than geometry alone.

**What final project outcome should students achieve?**

* Students should run Panda IK demos for point-to-point, straight line, circle, rectangle, and invalid targets.
* Students should preview paths with markers before sending a trajectory.
* Students should explain why unreachable or unsafe targets must be rejected before execution.

**What concepts must the instructor explain?**

* Explain task space, end-effector pose, IK seed, reachability, joint limits, and singularities at a visual level.
* Explain waypoint discretization, trajectory timing, `JointTrajectory`, and `/motion/status`.
* Explain how gravity, payload, and tuning affect real execution without deriving full dynamics.

**What exact code/package/files should be shown?**

* Show `robotic_arm_motion/motion/panda_kinematics.py` and `trajectory_utils.py`.
* Show `robotic_arm_motion/motion/demo_01_point_to_point.py` through `demo_05_workspace_reject.py`.
* Show `robotic_arm_motion/launch/demo_01_point_to_point.launch.py` through `demo_06_moveit_pose_goal.launch.py`.

**What commands should be run?**

* Run `ros2 launch robotic_arm_motion demo_01_point_to_point.launch.py`.
* Run `ros2 launch robotic_arm_motion demo_02_straight_line.launch.py` and `demo_04_rectangle_shape.launch.py`.
* Run `ros2 launch robotic_arm_motion demo_05_workspace_reject.launch.py`, then `demo_06_moveit_pose_goal.launch.py`.

**What visual/demo result should appear?**

* RViz should show marker previews for the intended Cartesian path.
* The Panda should follow the accepted trajectory in simulation or visualization.
* Invalid workspace targets should be rejected with clear status instead of unsafe motion.

**What mistakes should the instructor avoid?**

* Do not teach MoveIt first, because students need to see what MoveIt automates.
* Do not hide failed targets; workspace rejection is one of the main learning outcomes.
* Do not turn dynamics into a full math derivation in this module.

**What should the student understand by the end of each lesson?**

* Students should know how a Cartesian target becomes joint values.
* Students should connect each path shape to the code that generated its waypoints.
* Students should be able to rerun the demo and debug unreachable targets or missing markers.

**Package Names And Responsibilities?**

* `robotic_arm_motion` owns IK, trajectory demos, marker previews, and motion utilities.
* `robotic_arm_bringup` owns the Panda simulation and MoveIt-capable bringup used underneath.
* `robotic_arm_control` supplies the trajectory controllers that execute joint commands.

## Core module description

Module 3 teaches students how to move from joint-space control to task-space motion. Instead of saying “move joint 1 and joint 2,” students learn how to say “move the gripper to this point” and understand how IK converts that Cartesian target into joint values.

Students also learn that a trajectory is not a single target, but a sequence of waypoints with timing. They use Panda motion demos to run point-to-point movement, straight-line movement, circular paths, rectangle paths, and invalid workspace rejection.

## Main story of Module 3

In Module 2, students learned:

```text
joint angles → end-effector pose
```

Now Module 3 asks the reverse question:

> If I know where I want the gripper to go, how do I calculate the joint angles needed to reach it?

This is the transition from FK to IK:

```text
FK: joints → gripper pose
IK: gripper target → joints
```

## Module 3 in scope

Module 3 should cover:

* task space
* end-effector pose
* Cartesian target
* inverse kinematics
* IK seed
* joint limits
* reachability
* workspace limits
* singularities at a visual/conceptual level
* trajectory waypoints
* marker preview in RViz
* `JointTrajectory` publishing
* path discretization
* point-to-point path
* straight-line path
* circle/bump path
* rectangle path
* workspace rejection
* MoveIt pose goal as a higher-level comparison
* dynamics basics as intuition
* gravity/load effect as a real-world preview

The Module 3 file includes live demos in the `robotic_arm_motion` package for point-to-point, straight line, circle bump, rectangle shape, and workspace rejection.

## Module 3 out of scope

Do not go too deep into:

* full dynamics derivation
* Lagrangian equations in detail
* inverse dynamics implementation
* advanced MoveIt planning scenes
* collision-aware industrial planning
* force control
* torque control
* real payload compensation implementation

These can be introduced as future concepts.

## Module 3 deliverables

By the end of Module 3, students should have:

* Panda simulation running
* ability to run IK trajectory demos
* ability to preview path markers
* ability to monitor `/motion/status`
* understanding of Cartesian target to joint solution
* understanding of waypoint-based trajectories
* ability to identify invalid workspace targets
* basic understanding of why dynamics affects real motion
* MoveIt pose-goal demo as comparison with custom IK pipeline

## Module 3 success criteria

Students should be able to:

* explain IK in simple language
* explain task space vs joint space
* run point-to-point IK demo
* run straight-line trajectory demo
* run circle/bump trajectory demo
* run rectangle path demo
* run workspace rejection demo
* explain why invalid paths should be rejected before execution
* explain why markers are previewed before robot motion
* explain what MoveIt automates
* understand why gravity, payload, and tuning affect real robot motion

## Module 3 final scope decision

Keep Module 3 focused on this pipeline:

```text
Cartesian target → workspace check → IK solution → trajectory waypoints → marker preview → joint trajectory execution
```

MoveIt should be shown after the custom demos, not before. Students should first understand what MoveIt is automating.

---
