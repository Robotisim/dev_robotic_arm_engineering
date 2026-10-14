# Module 4 — Simulated depth perception and picking

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [Find a coloured cube with depth and pick it](projects/01_depth_pick/README.md)

Switch to Panda in Gazebo and build a camera-to-object-to-grasp pipeline. Real-robot execution ends at Module 3; this module never starts RoboFlex transport.

## Features and concepts

RGBD, camera intrinsics/extrinsics supplied by simulation, TF, point-cloud conventions, ROI, table exclusion, voxel filtering, Euclidean colour clusters, box estimates, scene collision objects, stale observations, grasp stages and outcome verification.

The supported scene contains red, green and blue 4 cm cubes on a known horizontal table. Detection uses coloured top patches; it is not general object recognition. No physical calibration is needed, but correct transforms and timestamps remain essential.

## Node responsibilities

| Node | Responsibility |
|---|---|
| `panda_learning_sim` workcell launch | Gazebo, Panda, controllers, camera, ROS bridges, MoveIt and task server |
| `depth_objects` | Transform/filter RGB point cloud; publish objects, markers and planning-scene updates |
| `perception_pick` | Select a fresh unambiguous colour, submit a pick, bounded retry with rotated grasp, CSV report |
| `motion_server` | Open, pregrasp, approach, close, lift, transfer, lower, release, retreat |
| `scene_manager` | Simulator ground truth for outcome evaluation and deterministic scene resets |

Ground-truth poses are used for evaluation, not object selection or grasp pose generation. Those come from the camera.

## Lab 1 — Start the workcell

The bridge maps Gazebo `/world/course/clock` to ROS `/clock`. The project readiness helper checks that time is positive and advancing before motion or recording. Camera messages alone do not establish clock readiness. Using the world-qualified topic avoids depending on Gazebo's global `/clock`, which may be suppressed when another publisher is detected; see the [official bridge documentation](https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md).

Stop the RoboFlex launch. In a new domain or the now-empty course domain:

```bash
ros2 launch arm_module_04 module_04.launch.py headless:=true rviz:=false
```

Omit the GUI arguments for Gazebo/RViz. This starts the complete stack. If the bare Panda workcell is already running, use `start_sim:=false` to add only perception; do not start two copies of `depth_objects`.

Inspect:

```bash
ros2 topic echo /camera/camera_info --once
ros2 topic echo /learning/objects --once
ros2 topic echo /learning/ground_truth --once
```

RViz displays: `/camera/image_raw`, `/learning/filtered_cloud`, `/learning/object_markers`, TF and the robot. The camera is at `(0.48, 0, 1.15)` in world, looking down. `panda_link0` coincides with world.

**Harmonic frame detail:** the RGBD packed cloud stores X-forward camera-link coordinates even when the sensor header uses `camera_optical_frame`. `depth_objects` explicitly uses `cloud_frame:=camera_link`. Image projection uses optical conventions. A lesson should compare the raw centre pixel's approximately 1.15 m X value to its transformed table Z near zero. Set `cloud_frame` to an empty string only for an input that correctly labels its own cloud frame.

## Lab 2 — Inspect the C++ processing pipeline

Read `arm_module_04/src/depth_objects.cpp` in this order:

1. Transform each valid RGB point into `panda_link0` at the image timestamp.
2. Apply the workspace ROI and exclude the known table plane and neutral-coloured arm.
3. Split saturated red/green/blue points; downsample with 4 mm voxels.
4. Cluster points and require a visible top patch so a thin side-face fragment is not another object.
5. Estimate XY extent and top height; use the known 4 cm cube height to recover centre Z, including cubes on a stack. Attach a frame and timestamp, and publish typed objects.
6. Publish markers and collision objects while the task stage is idle; freeze scene geometry during grasp execution. Retain last-seen geometry for up to 20 idle frames through short camera occlusions.

Change ROI/voxel parameters, compare segmentation and centroid error against evaluator ground truth, and deliberately hide a cube. Multiple matches for one requested colour must be treated as ambiguous.

## Lab 3 — Preview a camera-derived grasp

Second terminal:

```bash
ros2 run arm_module_04 perception_pick --ros-args \
  -p use_sim_time:=true -p colour:=red -p attempts:=1
```

A pick preview plans **only the pregrasp**. The full manipulation needs contact and object state changes, so this preview is not represented as full-task success evidence.

## Lab 4 — Pick, lift and place

```bash
ros2 run arm_module_04 perception_pick --ros-args \
  -p use_sim_time:=true -p colour:=red -p execute:=true \
  -p destination:='[0.45,-0.28,0.025]' -p attempts:=2 \
  -p report:=/tmp/perception_trials.csv
```

The server permits finger contact with the target, approaches in Cartesian segments, closes the fingers, lifts and verifies a physical Gazebo lift of at least 4 cm. MoveIt attachment is collision bookkeeping; no invisible simulator weld is used. Placement is checked against the requested destination. Friction, controller tracking, geometry and object alignment therefore matter.

On failure read the stage, controller result and CSV. Retries are bounded; they do not claim success because a plan alone completed. For repeated independent trials reset the scene between tasks:

```bash
ros2 service call /learning/reset_scene std_srvs/srv/Trigger '{}'
```

Reset is intended when the arm is idle and clear of objects. The scene manager rejects a reset during an active task.

## Completion evidence

Show the three detected cubes, compare one centroid to ground truth, demonstrate a missing/ambiguous target rejection, and inspect the pick stage/result. Progress to Module 5 to evaluate whole projects rather than individual plans.
