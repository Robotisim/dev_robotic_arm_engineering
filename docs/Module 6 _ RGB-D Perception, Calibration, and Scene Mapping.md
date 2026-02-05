# Module 6: RGB-D Perception, Calibration, and Scene Mapping

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: run an RGB-D pipeline, visualize point clouds, and feed a 3D map into planning for collision awareness.
  2. On __real system/hardware__: calibrate camera-to-arm extrinsics and perform runtime object pose estimation for pick/place (toy arm for transform intuition, Panda for end-to-end demos).

- __Prerequisites:__
  - Module 4–5 MoveIt + pick/place baseline
  - A depth camera (RealSense/ZED/etc.) or simulated depth topics

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.2-9 (frames + homogeneous transforms; required for extrinsics)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy arm: teach coordinate transforms and “what extrinsics mean” without too many frames.
  - Panda: validate extrinsics by overlaying point clouds on a realistic robot model and running pick/place.

- __Deliverables (Must Produce):__
  - `src/perception_bringup/` launch files for camera + point cloud visualization
  - Saved extrinsics (e.g., `camera_to_base.yaml`) + a calibration procedure doc
  - Scene mapping pipeline (Octomap/voxel) integrated into MoveIt planning scene
  - Runtime picking demo (sim or real): detect → estimate pose → plan → execute

- __Key Steps (Execution Path):__
  - Bring up camera topics and point cloud visualization
  - Calibrate camera-to-arm extrinsics and publish TF
  - Build/update a 3D occupancy representation and inject into planning scene
  - Implement object pose estimation (simple depth pipeline or tags)
  - Execute pick/place using measured poses

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ topic interfaces, TF frames, point cloud processing stages, planning scene updates.
  - __What changes in real:__ noise, missing depth, lighting, lens distortion, timestamp sync, vibration.
  - __Minimum validation:__ stable `camera_link → base_link` TF; scene updates at consistent rate; planning avoids mapped obstacles.
  - __Failure modes + checks (at least 3):__
    - Misaligned extrinsics → verify calibration target placement and frame conventions; check TF direction.
    - Point cloud unstable → check timestamps/QoS; reduce depth range; filter outliers.
    - Planning collides with real obstacles → ensure map frame matches `world/base`; tune padding and update rate.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - Depth to point clouds; intrinsics vs extrinsics
   - Mapping for manipulation: occupancy vs collision geometry
   - Perception-to-planning data flow
   - Toy vs Panda: same extrinsics math; more frames and more failure modes on Panda
2. __Implementation__
   - Launch RGB-D pipeline and visualize in RViz
   - Build Octomap/voxel map and push it to MoveIt planning scene
3. __Validation__
   - Scene updates visibly block plans that would collide with obstacles

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Extrinsics calibration workflow; publish TF reliably
2. __Runtime Testing__
   - Pose estimation repeatability on a known object/tag
3. __Debugging__
   - Timestamp sync, frame direction mistakes, depth holes, false detections

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run camera launch + visualize `/points` and `/image`
- Run calibration routine and save extrinsics
- Run scene mapping and confirm collision avoidance changes

### Pictorial (Visual / Intuition)
- RViz: point cloud overlay on robot model
- Planning scene: obstacles appearing/disappearing from the map

### Abstract (Math / Architecture / Theory)
- Transform chain: pixel → ray → 3D point → robot base frame
- Update rates, filtering, and planning safety margins

---

## Updated Video List

> Naming convention:
> `M6_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M6_V01_Outcome_Perception_to_Planning_[F]__
   - __Content__: Camera bringup, extrinsics, mapping, runtime pose → pick.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (frames/transforms; perception uses the same math)

2. __M6_V02_Rigid_Transforms_and_Camera_Extrinsics_Theory_[F]__
   - __Content__: Rigid transforms, relative poses, and the exact meaning of “camera-to-base extrinsics”; how small transform errors become missed grasps.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (relative poses + homogeneous transforms)

3. __M6_V03_Extrinsics_Calibration_Mental_Model_[A]__
   - __Content__:
     - __Context__: Without correct extrinsics, your robot “believes” the world is somewhere else.
     - __Analogy Start__: “Like wearing glasses with the wrong prescription—the world is shifted.”
     - __Explanation__: Intrinsics vs extrinsics; `camera_link` relative to `base_link`; why small errors become missed grasps.
     - __Animation Style__: 2D camera frustum on a table; coordinate axes; calibration board.
     - __Process__:
       1. Show true board pose vs shifted estimated pose.
       2. Apply extrinsics correction → alignment improves.
       3. Visualize impact at gripper (miss vs hit).
       4. Emphasize frame direction: `base → camera` vs `camera → base`.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (relative poses + homogeneous transforms)

4. __M6_V04_RGBD_Bringup_and_PointCloud_Visualization_[S]__
   - __Content__: Launch camera driver, visualize point clouds, sanity-check frame IDs and timestamps (toy + Panda overlays).
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (frame mismatch debugging)

5. __M6_V05_Octomap_or_Voxel_Mapping_into_MoveIt_[S]__
   - __Content__: Generate a 3D map and integrate into planning scene; demonstrate collision-aware replanning.
   - __Notes__: (Mapping/Octomap is not covered in `Notes/`)

#### Part B: Real System / Hardware
6. __M6_V06_Camera_to_Arm_Calibration_Runbook_[S]__
   - __Content__: Execute calibration, save extrinsics, verify by overlaying point cloud on robot geometry.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (rigid transform intuition)

7. __M6_V07_Runtime_Object_Pose_and_Tag_Based_Picking_[S]__
   - __Content__: Detect object pose from depth (or QR/AprilTag), plan grasp, execute pick.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (pose/frames) + p.17-22 (IK) as background

---

## Assignments & Practice Tasks

1. __M6_A01_PointCloud_Sanity_Check__
   - __Goal__: bring up RGB-D and confirm point cloud frame alignment.
   - __Submission__: RViz screenshot showing point cloud + robot model aligned.
   - __Pass Criteria__: points appear in correct location relative to `base_link`.

2. __M6_A02_Extrinsics_Calibration_Artifact__
   - __Goal__: produce a saved extrinsics file + short procedure doc.
   - __Submission__: `camera_to_base.yaml` + `docs/m6_calibration.md`.
   - __Pass Criteria__: TF published consistently; includes a 3-trial repeatability note.

3. __M6_A03_Scene_Mapping_Blocks_Collisions__
   - __Goal__: show that a mapped obstacle prevents a colliding plan.
   - __Submission__: 60–90s demo video + relevant config.
   - __Pass Criteria__: planner reroutes or fails with a clear collision reason.

4. __M6_A04_Debugging_Drill_TimeSync_or_FrameFlip__
   - __Goal__: reproduce a time sync or frame flip bug and fix it.
   - __Submission__: writeup + screenshot before/after.
   - __Pass Criteria__: stable point cloud + correct transform restored.

---

## Final Summary

- You can bring up RGB-D perception, visualize point clouds, and calibrate extrinsics.
- You can build a scene map and feed it into planning for runtime collision awareness.
- You can estimate object pose (depth or tags) and run pick/place online.
- Next module adds recovery behaviors, logging, and an evaluation harness for reliability.
