# Prompt: Implement `motion_control`

You are implementing a ROS 2 Python package named `motion_control`.

## Goal

Build the mathematics and command layer above `robot_arm_bringup`.

This package must:

1. Define DH tables and coordinate conventions for Stick Arm and Panda.
2. Compute forward kinematics.
3. Extend FK into numerical inverse kinematics.
4. Send safe joint trajectories to the `arm_trajectory_controller` from `robot_arm_bringup`.
5. Generate motion between points.
6. Draw circles and squares by converting Cartesian samples into valid joint trajectories.
7. Run a hard-coded Gazebo pick-and-place demonstration.
8. Prepare the controller interface used later by MoveIt.

This package must not implement low-level Gazebo plugins or hardware interfaces. It is a client of the controller action.

## Required package structure

```text
motion_control/
├── motion_control/
│   ├── dh_models.py
│   ├── kinematics_types.py
│   ├── forward_kinematics.py
│   ├── inverse_kinematics.py
│   ├── trajectory_generation.py
│   ├── shape_paths.py
│   ├── controller_client.py
│   ├── fk_cli_node.py
│   ├── ik_cli_node.py
│   ├── joint_command_cli_node.py
│   ├── trajectory_demo_node.py
│   ├── shape_demo_node.py
│   └── pick_place_demo_node.py
├── launch/
│   ├── robot_arm_controller.launch.py
│   └── robot_panda_controller.launch.py
├── config/
│   ├── stick_arm_kinematics.yaml
│   └── panda_kinematics.yaml
├── test/
│   ├── test_fk.py
│   ├── test_ik.py
│   └── test_trajectory_generation.py
└── package.xml
```

## Dependencies and libraries

Use:

* `rclpy` for ROS 2 nodes.
* `numpy` for matrices, vectors, Jacobians, and trajectory arrays.
* `sensor_msgs/msg/JointState` for feedback.
* `trajectory_msgs/msg/JointTrajectory` and `JointTrajectoryPoint`.
* `control_msgs/action/FollowJointTrajectory` for controller action goals.
* `geometry_msgs/msg/Pose` and `PoseStamped` for Cartesian targets.
* `tf2_ros` and `tf_transformations` for transforms and quaternion conversion.
* `argparse` for command-line inputs.
* Optional: `scipy.optimize` only for bounded numerical IK, but keep a NumPy damped-least-squares solver as the core implementation.

## Coordinate-frame rules

Document and enforce:

```text
world → base_link → link_1 → ... → flange → tool0
```

Use:

* Metres for translation.
* Radians for joint angles.
* Right-handed coordinate system.
* DH frame `i` attached according to the documented DH convention.
* End-effector pose always reported as `base_link → tool0`.

Never guess a DH table from robot appearance. Derive it from the validated URDF joint origins, axes, and link transforms.

## FK implementation

Create reusable pure-Python FK functions:

```python
forward_kinematics(robot_name, joint_positions) -> TransformMatrix
```

The result must include:

* Position: x, y, z.
* Rotation matrix.
* Quaternion.
* Roll, pitch, yaw.
* Full homogeneous transform matrix.

Create `fk_cli_node.py`:

```bash
ros2 run motion_control fk_cli_node --robot stick --joints 0.0,0.2,-0.4,...
```

It must print:

* Input joint vector.
* DH transform for every joint.
* Final `base_link → tool0` matrix.
* Position and orientation.

Validate FK by sending the same joint state to RViz and comparing the calculated tool pose against TF.

## IK implementation

Extend the FK module with numerical IK:

```python
inverse_kinematics(
    robot_name,
    target_pose,
    seed_joint_positions,
    joint_limits,
    tolerance,
    max_iterations
) -> IKResult
```

Use damped least squares:

* Calculate current end-effector error.
* Build the Jacobian.
* Apply damping near singularities.
* Respect joint limits.
* Use the prior solution as the next seed for continuous motion.
* Return structured failure instead of unsafe random joint values.

Create `ik_cli_node.py`:

```bash
ros2 run motion_control ik_cli_node \
  --robot stick \
  --xyz 0.35,0.10,0.25 \
  --rpy 0.0,1.57,0.0
```

The node must print:

* Target pose.
* Solution vector.
* FK verification of the solved pose.
* Position/orientation error.
* Failure reason when unreachable or singular.

## Controller action client

Create `controller_client.py` and `joint_command_cli_node.py`.

The action endpoint must be configurable, defaulting to:

```text
/arm_trajectory_controller/follow_joint_trajectory
```

Example command:

```bash
ros2 run motion_control joint_command_cli_node \
  --robot stick \
  --joints 0.0,0.3,-0.6,0.2,... \
  --duration 3.0
```

Before sending:

* Verify joint count.
* Validate joint limits.
* Validate velocity and acceleration limits.
* Ensure `time_from_start` is strictly increasing.
* Obtain the current joint state and begin trajectory from the current pose.
* Reject commands if controller action server is unavailable.

## Motion between points

Create `trajectory_generation.py` with:

* Joint-space interpolation.
* Cubic or quintic time scaling.
* Velocity and acceleration limits.
* Segment timing based on maximum joint movement.
* Conversion to `JointTrajectory`.

Implement `trajectory_demo_node.py`:

* Move from current state to point A.
* Move A → B.
* Move B → C.
* Return to home.
* Log every target, solved joint vector, duration, and action result.

## Circle and square demonstrations

Create `shape_paths.py`:

* `generate_circle(center, radius, plane, samples)`
* `generate_square(center, side_length, plane, samples_per_edge)`

Implement `shape_demo_node.py` pipeline:

```text
Cartesian shape points
→ continuous IK using previous result as seed
→ reject unreachable points
→ joint-space smoothing
→ velocity/acceleration validation
→ JointTrajectory
→ FollowJointTrajectory action
```

Requirements:

* Start with small shapes near the workspace centre.
* Use at least 50 samples for a circle.
* Do not jump between IK branches.
* Stop safely when IK fails.
* Publish sampled Cartesian points for RViz visualization.

## Gazebo pick-and-place demo

Create a new Gazebo world with:

* Table.
* Three colored cubes.
* Drop zones.
* Fixed object poses.
* Robot spawned at a known base pose.

Create `pick_place_demo_node.py`:

1. Move to a safe home pose.
2. Move above a hard-coded object pose.
3. Descend to grasp pose.
4. Trigger placeholder gripper command or simulated attachment.
5. Lift vertically.
6. Move above drop zone.
7. Descend and release.
8. Return home.

Keep this first version deterministic and hard-coded. Do not claim perception, collision avoidance, or dynamic grasp planning in this stage.

## Required launches

```bash
ros2 launch motion_control robot_arm_controller.launch.py
ros2 launch motion_control robot_panda_controller.launch.py
```

Each launch must:

* Include the matching bringup/controller launch.
* Start the applicable command/diagnostic nodes.
* Expose launch arguments:

  * `robot:=stick|panda`
  * `mode:=rviz|gazebo|hardware`
  * `use_sim_time:=true|false`
  * `controller_name:=arm_trajectory_controller`

## Acceptance criteria

1. FK matches TF at at least five tested joint configurations.
2. IK reaches valid reachable targets within tolerance.
3. Invalid, singular, and unreachable targets fail safely.
4. CLI node moves the arm through `FollowJointTrajectory`.
5. Circle and square execute in Gazebo without discontinuous joint jumps.
6. Pick-and-place completes for a fixed object arrangement.
7. Unit tests cover FK, IK convergence, limits, trajectory timestamps, and invalid input.
