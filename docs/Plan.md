# Progression Plan

## Module Files (docs/)
\# [Notes Index (Page Map)](<Notes Index.md>)
\# [Module 1: ROS2 Workspace, Simulation, and Arm Model](<Module 1 _ ROS2 Workspace, Simulation, and Arm Model.md>)
\# [Module 2: Joint Control, Calibration, and Forward Kinematics](<Module 2 _ Joint Control, Calibration, and Forward Kinematics.md>)
\# [Module 3: Inverse Kinematics, Trajectories, and Dynamics Basics](<Module 3 _ Inverse Kinematics, Trajectories, and Dynamics Basics.md>)
\# [Module 4: MoveIt 2 Planning and Collision Avoidance](<Module 4 _ MoveIt 2 Planning and Collision Avoidance.md>)
\# [Module 5: Pick and Place with Fixed Targets](<Module 5 _ Pick and Place with Fixed Targets.md>)
\# [Module 6: RGB-D Perception, Calibration, and Scene Mapping](<Module 6 _ RGB-D Perception, Calibration, and Scene Mapping.md>)
\# [Module 7: Failure Recovery, Logging, and Evaluation](<Module 7 _ Failure Recovery, Logging, and Evaluation.md>)
\# [Module 8: Robot Learning and Language-to-Action Tasking](<Module 8 _ Robot Learning and Language-to-Action Tasking.md>)

## Teaching Approach
\# Toy arm (basic URDF) first, then Panda side-by-side for the same concept.

## Module 1 — ROS2 Setup + Arm Modeling
\# __Lecture 0)__ ROS2 + Arm Bringup (Simulation First) — Create a ROS2 workspace, bring up the arm in RViz/Gazebo with a clean launch pipeline.
\# __Lecture 1)__ URDF/Xacro Modeling of Your Arm — Build the arm model (links/joints, limits, inertias) and validate it in RViz with TF.
\# __Lecture 2)__ ros2_control Hardware Interface (Real Arm) — Implement the driver layer (joint states + joint commands) so ROS2 can read/command the physical motors.

## Module 2 — Control + Calibration + FK
\# __Lecture 3)__ Basic Joint Control (Position/Velocity Control) — Move single joints and then multi-joint trajectories with safety limits and smooth interpolation.
\# __Lecture 4)__ Calibration + Zeroing + Repeatability Checks — Establish homing, joint offsets, and repeatability tests so the arm behaves predictably every run.
\# __Lecture 5)__ Forward Kinematics (FK) + TF Verification — Compute end-effector pose from joint angles and verify it matches RViz/TF numerically.

## Module 3 — IK + Trajectories + Dynamics
\# __Lecture 6)__ Inverse Kinematics (IK) for Target Poses — Solve “move end-effector to (x,y,z,orientation)” using a practical IK approach for your arm.
\# __Lecture 7)__ Trajectory Generation (Time-Parameterized Motion) — Generate smooth joint trajectories (limits-aware) and execute them reliably on real hardware.
\# __Lecture 8)__ Dynamics Basics (Why Motion Feels Different in Reality) — Introduce torque/load, gravity effects, and tuning considerations that impact real motion quality.

## Module 4 — MoveIt 2 + Motion Planning
\# __Lecture 9)__ MoveIt 2 Integration (Planning Pipeline) — Use MoveIt 2 for planning + execution, including planning scene, collision objects, and constraints.
\# __Lecture 10)__ Motion Planning with Obstacles (Collision Avoidance) — Plan around simple obstacles in simulation, then mirror the same planning scene on the real arm.

## Module 5 — Manipulation Basics
\# __Lecture 11)__ Pick/Place “Fixed Targets” (First Manipulation) — Execute repeatable pick-and-place using pre-defined object poses and approach/retreat motions.

## Module 6 — Perception + Scene Understanding
\# __Lecture 12)__ Perception Setup (RGB-D Camera in ROS2) — Integrate a depth camera, publish point clouds, and align camera-to-arm calibration (extrinsics).
\# __Lecture 13)__ Scene Mapping for Manipulation (Voxel/Octomap) — Build a 3D scene representation and feed it to planning for runtime collision awareness.
\# __Lecture 14)__ Runtime Object Picking (Pose from Depth) — Detect an object, estimate its 6D pose from depth, plan grasp, and execute pick-and-place online.
\# __Lecture 15)__ QR Code-Based Pick/Place (Structured Perception) — Use QR tags to get robust IDs + poses for teaching structured automation workflows.

## Module 7 — Robustness + Evaluation
\# __Lecture 16)__ Failure Handling + Recovery Behaviors — Add “retry grasp,” “re-localize,” “safe retreat,” and “home” behaviors for real-world robustness.
\# __Lecture 17)__ Data Logging + Evaluation Harness — Record ROS bags, define success metrics (time, accuracy, retries), and build a repeatable test suite.

## Module 8 — Learning + High-Level Tasking
\# __Lecture 18)__ RL for Motion Primitives (Simulation to Real Concepts) — Train a small policy for a constrained skill (e.g., insertion/approach) in sim, then discuss safe transfer.
\# __Lecture 19)__ VLA / “Language-to-Action” Task Layer — Add a high-level instruction interface that selects skills (“pick red block,” “place in bin”) while ROS2 executes safely.
