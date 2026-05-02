
# Module 6 Scope

## Module 6 title

**RGB-D Perception, Camera Calibration, Scene Mapping, and Perception-Driven Manipulation**

## Core module description

Module 6 teaches students how to replace fixed target poses with **runtime perception**. Students learn how RGB-D cameras produce image, depth, and point cloud data; how camera frames connect to the robot base; how color/depth segmentation estimates object poses; how those poses are transformed into the robot frame; and how the robot uses those detected poses for planning and pick/place.

This module should not become a general computer vision course. It should stay focused on:

```text
camera data → object pose → robot frame → planning scene / pick-place
```

Your Module 6 plan defines the goal as running an RGB-D pipeline, visualizing point clouds, feeding a 3D map into planning, calibrating camera-to-arm extrinsics, and performing runtime object pose estimation for pick/place.

## Main story of Module 6

In Module 5, we knew the object pose manually.

In Module 6, the question becomes:

> How does the robot see the object, estimate where it is, and use that pose for planning?

The main pipeline is:

```text
RGB image + depth image
→ segmentation
→ pixel/depth object center
→ 3D point in camera frame
→ transform to robot base frame
→ publish object pose
→ MoveIt planning
→ pick/place execution
```

## Module 6 in scope

Module 6 should cover:

* RGB-D camera bringup
* RGB image topic
* depth image topic
* camera info topic
* camera frame
* optical frame
* intrinsics
* extrinsics
* camera-to-base transform
* TF direction
* RGB/depth alignment
* point cloud visualization
* HSV/color segmentation
* contour detection
* depth at centroid
* 3D object pose estimation
* transform pose into `panda_link0` / robot base
* publish detections
* publish markers
* publish object poses
* PCD snapshot for debugging
* calibration check
* perception pose to MoveIt pose goal
* perception-driven pick/place integration
* scene mapping / Octomap as an advanced extension

## Module 6 out of scope

Do not make these the main focus:

* deep learning object detection
* neural segmentation
* VLA policies
* full SLAM
* advanced point cloud registration
* industrial hand-eye calibration derivation
* advanced grasp synthesis
* multi-camera fusion
* visual servoing
* force-based grasp correction

These can be future modules or bonus sections.

## Module 6 required deliverables

By the end of Module 6, students should have:

* `robotic_arm_vision` launch working
* camera topics visible
* segmentation output visible
* annotated image visible
* RViz markers visible
* object poses published
* PCD snapshot captured
* calibration check performed
* saved camera-to-base extrinsics file
* perception pose used by MoveIt
* perception-driven pick/place demo
* debugging report for frame or timestamp issue

Your uploaded plan asks for camera/point-cloud launch files, saved extrinsics such as `camera_to_base.yaml`, a calibration procedure, scene mapping into MoveIt, and runtime picking based on detected poses.

## Module 6 cleanup before recording

Before recording Module 6, fix or clearly mark these issues:

* Docs say `src/perception_bringup`, but actual package is `robotic_arm_vision`.
* Add saved `camera_to_base.yaml`.
* Add a clean calibration procedure doc like `docs/m6_calibration.md`.
* Octomap/voxel integration into MoveIt is not implemented yet.
* `pcd_snapshot.py` imports Open3D, but package dependencies need to declare Open3D.
* The end-to-end detect → estimate pose → pick flow exists in pieces, but should be wrapped into one clean launch flow.

These cleanup items are identified in your Module 6 teaching map.

## Module 6 success criteria

Students should be able to:

* Launch the RGB-D perception pipeline.
* Explain intrinsics vs extrinsics.
* Explain camera frame vs robot base frame.
* Visualize camera output.
* Visualize point cloud data.
* Run segmentation.
* Inspect object poses.
* Explain pixel + depth to 3D point conversion.
* Validate camera-to-base transform.
* Use detected object pose for planning.
* Debug frame direction errors.
* Debug depth instability.
* Debug timestamp/QoS issues.

## Module 6 final scope decision

Keep Module 6 focused on:

```text
RGB-D data → calibration → object pose estimation → robot-frame pose → planning/pick-place
```

Treat Octomap/voxel mapping as either:

```text
Core only if implemented before recording
```

or:

```text
Advanced/bonus if not implemented yet
```

Do not make scene mapping the main blocker for Module 6 unless the pipeline is ready.

---
