# Module 5: Pick and Place with Fixed Targets

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: execute a repeatable pick-and-place routine using pre-defined target poses and approach/retreat motions (toy arm for explanation, Panda for the full workflow).
  2. On __real system/hardware__: execute the same routine with conservative speeds and simple success checks.

- __Prerequisites:__
  - Module 4 MoveIt 2 pipeline working
  - A gripper (real or simulated) or a placeholder “attach/detach” interface

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.2-9 (poses, frames, homogeneous transforms)
  - `Notes/part_1_kinematics.pdf` p.17-22 (IK)
  - `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning)
  - `Notes/part_3_motion.pdf` p.12-18 (control/tuning intuition for reliable execution)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy arm: teach frames (`base`, `tool`, `pregrasp`, `grasp`) and target definitions.
  - Panda: run the full pick/place pipeline with MoveIt execution and collision checking.

- __Deliverables (Must Produce):__
  - A `pick_place` package/script that runs:
    - approach → grasp/attach → lift → place → retreat
  - A set of fixed targets stored in YAML (object + place + pre-grasp)
  - Demo video showing 3 consecutive successful cycles (sim; real if available)

- __Key Steps (Execution Path):__
  - Define object + place targets in a fixed frame (`base_link` or `world`)
  - Add approach/retreat waypoints and constraints (orientation, clearance)
  - Integrate gripper open/close (or attach/detach in sim)
  - Add basic checks: reachability, plan success, post-place retreat

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ targets/frames, approach/retreat logic, planning requests, gripper command interface.
  - __What changes in real:__ grasp tolerance, contact uncertainty, compliance, object slip, safety speeds.
  - __Minimum validation:__ 3 cycles with zero collisions and consistent final placement pose.
  - __Failure modes + checks (at least 3):__
    - Grasp misses object → adjust pre-grasp offsets; verify frame and object pose.
    - Collisions near table → increase clearance; tune collision padding.
    - Object drops on lift → slow acceleration; verify gripper force/close timing.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - Task decomposition: approach/grasp/lift/place/retreat
   - Fixed frames + stored targets for repeatability
   - Toy vs Panda: grasp frames stay the same; execution stack complexity changes
2. __Implementation__
   - Implement a deterministic pick/place pipeline using MoveIt plans
   - Store targets in YAML and load at runtime
3. __Validation__
   - Cycle test: repeat N times and record success/failure + timings

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Gripper calibration (open/close positions or force)
2. __Runtime Testing__
   - Bench test with conservative speeds; add “pause/continue” safety gating
3. __Debugging__
   - Diagnose object pose offsets, gripper timing, and contact variability

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run pick/place script with fixed targets
- Record repeated cycles and capture failure reasons

### Pictorial (Visual / Intuition)
- RViz markers for target poses + approach waypoints
- Success timeline: plan → execute → gripper → attach/detach

### Abstract (Math / Architecture / Theory)
- Motion constraints (orientation constraints, path constraints)
- Why structured approach/retreat reduces failure probability

---

## Updated Video List

> Naming convention:
> `M5_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M5_V01_Outcome_First_Manipulation_[F]__
   - __Content__: The pick/place pipeline, target definitions, and what “success” means.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (pose/frames) + p.17-22 (IK)

2. __M5_V02_Grasp_Frames_and_Target_Poses_Theory_[F]__
   - __Content__: How to define `pregrasp`, `grasp`, and `place` frames, how those relate to the tool frame, and why frame mistakes cause “perfect plans that miss”.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (poses/frames) + p.17-22 (IK framing)

3. __M5_V03_Why_Approach_and_Retreat_Work_[A]__
   - __Content__:
     - __Context__: Most failures happen near contact; structured motion reduces uncertainty.
     - __Analogy Start__: “Like parking a car: you line up, move slowly into the spot, then back out cleanly.”
     - __Explanation__: Pre-grasp alignment, straight approach corridor, lift clearance, and retreat.
     - __Animation Style__: 2D top/side views showing safe corridor volumes and waypoints.
     - __Process__:
       1. Show direct-to-grasp path colliding/missing.
       2. Add pre-grasp waypoint → alignment improves.
       3. Add straight approach corridor → contact reliability improves.
       4. Add lift/retreat clearances → fewer collisions.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning mindset; approach/retreat pattern not explicit in `Notes/`)

4. __M5_V04_Implementing_PickPlace_with_MoveIt_[S]__
   - __Content__: Load targets from YAML, plan approach/grasp/lift/place/retreat, execute, and log outcomes (toy arm for visualization; Panda for the real demo).
   - __Notes__: `Notes/part_1_kinematics.pdf` p.17-22 (IK) + p.39-46 (trajectory planning) as background

5. __M5_V05_Validation_Cycle_Test_and_Failure_Logging_[S]__
   - __Content__: Run N cycles, log timings/failures, compute success rate.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (interpreting oscillation/overshoot symptoms) as background

#### Part B: Real System / Hardware
6. __M5_V06_Gripper_Bringup_and_Timing_[S]__
   - __Content__: Real gripper interface, open/close calibration, and safe testing.
   - __Notes__: (Gripper hardware bringup; not covered in `Notes/`)

7. __M5_V07_Debugging_Grasp_Offsets_and_Slip_[S]__
   - __Content__: Diagnose missed grasps and object slip; tune offsets, speeds, and gripper parameters.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (tuning symptoms) + `Notes/part_3_motion.pdf` p.17-18 (gravity compensation concept) as background

---

## Assignments & Practice Tasks

1. __M5_A01_Fixed_Targets_YAML__
   - __Goal__: define object + place targets in YAML and visualize them in RViz.
   - __Submission__: YAML + RViz screenshot.
   - __Pass Criteria__: targets load correctly and are in the expected frame.

2. __M5_A02_PickPlace_Demo__
   - __Goal__: complete one full pick and one full place.
   - __Submission__: 60–90s demo video + code link.
   - __Pass Criteria__: no collisions; object ends at the place target.

3. __M5_A03_Cycle_Test_Report__
   - __Goal__: run 10 cycles (sim) and summarize success rate + mean cycle time.
   - __Submission__: CSV + 5–10 line report.
   - __Pass Criteria__: includes top 3 failure reasons (if any) and one improvement attempt.

4. __M5_A04_Debugging_Drill_Wrong_Target_Frame__
   - __Goal__: reproduce a bug where targets are in the wrong frame and fix it.
   - __Submission__: writeup + screenshot before/after.
   - __Pass Criteria__: corrected frame usage; routine works again.

---

## Final Summary

- You can perform deterministic pick/place with fixed targets and structured approach/retreat.
- You can measure and improve reliability via cycle tests and failure logs.
- Next module adds perception to replace fixed targets with runtime-detected poses.
