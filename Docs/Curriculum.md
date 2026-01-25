# Modules for Robotic Arm Learning Path

Follow each subsection top-to-bottom; each item builds on the previous one.
Complete Simulation before moving to Real Robotic Arm in each module.

## Module 1: Robotic Arm Bringup
- Introduction video
    - 1) What are Robotic Arm and what are there applications, what we will learn in this learning path
    - 2) Industrial robotic arms
    - 3) Tech jargons : Workspace , joint space , transforms , kinematics, Ik , Qr based robot motion , depth camera based auto pick and place, VLA robotic arm actions and explanation with real robot videos
- Simulation
    - 1) ROS2 installation and workspace setup
    - 2) Concept of URDF for robotic arm , DOF , transforms , components etc.
    - 3) Creating a robotic arm from urdf
    - 4) Robotic arm bringup and visualize in rviz
    - 5) Bringup example robotic arm like panda and understand its transform
    - 6) Running popular robotic arm Panda in simplified way launching in gazebo
    - 7) Shift to our robot in simulation even in next lecture - leave panda
    - 8) Simulation checklist: URDF loads, TF tree clean, joints move in RViz/Gazebo
- Real Robotic Arm
    - 1) creating actual robotic arm using stls ( blender to position etc)
    - 2) Concept of making a robotic arm ( motor selection , electronics, types of motor)
    - 3) Exploring opensource robotic arms : how they are constructed etc.
    - 4) Motor Selection and calculations for each joint
    - 5) Electronics for motor driving and firmware
    - 6) Building real robotic arm and not drive simple construction
    - 7) Power-on safety checklist, E-stop, and joint direction test at low speed

## Module 2: Robotic Arm Basic Control
- Introduction video
    - 1) ROS2 control and motion for robotic arm and options of control
- Simulation
    - 1) Understanding ROS2 Control
    - 2) Controller Types and installation of them in urdf
    - 3) Driving Different joints and observing effects in transforms
    - 4) Use RQT Plot to check responses
    - 5) Checkpoint: controllers load/unload cleanly and responses stay within limits
- Real Robotic Arm
    - 1) Updating firmware with joint controlls and command motors to move
    - 2) Finding limits of joints and hard coding them
    - 3) Tuning Position Controlling for most smooth behaviours
    - 4) Explain real robotic arm driving options ( simple ros2 topic etc or hardware interface)
    - 5) Writting Hardware Interface for Robotic Arm and drive using ROS2
    - 6) Build the digital Twin of it.
    - 7) Verify encoder zero/direction and soft limits before FK/IK

## Module 3: Kinematics and Calibration
- Introduction video
    - 1) Moving where we want to make it go , alot of mathematics
- Simulation
    - 1) Explaning Kinematics forward
    - 2) Drivign Equation for it basic and using python or library to sovle it for us
    - 3) Apply in a node to perform fk for panda
    - 4) Explain why Ik is desired and not the fk
    - 5) Solve ik using library
    - 6) Perform Ik solution with a node to perform the solution
    - 7) Checkpoint: FK/IK outputs match RViz within tolerance
- Real Robot
    - 1) Solve ik for our real robot using library and write node to send command too hardware interface
    - 2) Perform very basic Pick and place with saved co-ordinates
    - 3) Calibrate joint offsets and validate end-effector pose against measurement

## Module 4: Trjectory Execution
- Introduction Video
    - 1) Moving robot on a path for smooth output
    - 2) moveit motions  of Panda
- Simulation
    - 1) Explaning Trajectory concept and how it will be executed
    - 2) Explaning Trajectory for panda arm
    - 3) Introduction to why Moveit , direct run and installation of moveit with panda
    - 4) Setup and generate for panda configs
    - 5) Fix robot big rotations problem
    - 6) Checkpoint: trajectories execute smoothly in sim and respect limits
- Real Robot
    - 1) Concept of Trajectory Execution on real robot
    - 2) Real Robot trajectory execution Code with digital twin
    - 3) Setup Moveit for robotic arm and save configs
    - 4) Drive with moveit system gui ball
    - 5) problem that you dont know about availble area
    - 6) Fix robot big rotations problem
    - 7) Verify tracking error and stop behavior on real arm

## Module 5: Perception and Collision aware motion planning
- Introduction video
    - 1) having collision with hand and robot doesnot have understanding of scene
    - 2) 3D path generation in point cloud scnescene and avoidning  obstacles
- Simulation
    - 1) Basic moveit application with hard coded object addition example in robot goal path
    - 2) Point cloud data understnading
    - 3) Inserting camera in urdf and visualizing data in urdf and gazebo
    - 4) Depth camera addition and scene building
    - 5) Moveit importing point cloud data and planning with these obstacles
    - 6) Checkpoint: obstacles update planning scene and affect plans
- Real Robot
    - 1) Setup Intel realsense camera and processing
    - 2) Performing 3d Re construction of the environment , SLAM
    - 3) Moveit obtain map and perform motion planning
    - 4) Hand-eye calibration (camera to robot base) and frame alignment check

## Module 6: Pick and Place with Motion Awareness
- Introduction video
    - 1) We will develop simulation for the application and shift to real
    - 2) Robotic Arm in simulatio nand real : Picking and placcing objects in different bins
- Simulation
    - 1) Build Environment and record ros2 bag for data pipelines
    - 2) Add up moveit of goals and make things work
    - 3) Checkpoint: pick/place loop works in sim with repeatable success
- Real Robot
    - 1) Train data on objects basedon color
    - 2) stack up with moveit collision aware motion planning
    - 3) Gripper/tooling setup and approach/retreat tuning

## Module 7: VLA for robotic Arms
- Introduction Video
    - 1) Instructing robotic arm to produce actions
- Simulation and real
    - 1) Understanding model
    - 2) Explaning the workflow of model calls and the architecture
    - 3) Understanding the segment of model utilization
    - 4) Generated commands to our controllers.
    - 5) Live demo
    - 6) Safety gate + manual override for generated commands
