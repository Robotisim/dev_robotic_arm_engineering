* **Tabletop Bin-Picking Cell**: Detect table + segment multiple objects, generate grasps, pick-and-place into bins with retries and live PlanningScene updates.
* **“Clear the Table” Assistant**: Continuous perception loop + dynamic collision scene, picks all detected items and sorts them by size/zone (left/right) with failure recovery.
* **Smart Sorting Station (by type/pose)**: 6D pose for known objects + top-down grasps for unknowns, then sort into labeled bins using marker-defined drop zones.
* **Click-to-Pick Operator Station**: UI click on camera feed selects target; system does segmentation, grasp planning, constrained wrist approach, executes and reports success/fail.
* **Block Stacking Demo Cell**: Depth-based cube detection + pose refinement, constrained upright wrist, smooth trajectories, stack 2–3 blocks with regrasp if misaligned.
* **Pick-and-Place Packaging Tray**: Detect tray slots (AprilTag/geometry), pick objects, place into precise slots, enforce collision constraints and path smoothness.
* **Vision-Guided Precision Landing for Parts**: Use markers to define exact placement pose; approach under constraints, align using depth feedback, place gently with force threshold.
* **Regrasp-and-Orient Pipeline**: If object pose is bad for final placement, do intermediate place → re-detect → regrasp to achieve required orientation.
* **Wrist-Inspection + Best-Grasp Selection**: Move wrist through viewpoints, fuse depth observations, choose highest-confidence grasp, then execute with retry policy.
* **Safety-Aware Collaborative Table Cell**: Real-time scene updates from depth, enforce workspace limits, stop/replan on obstacle intrusion, log failures for dataset building.

