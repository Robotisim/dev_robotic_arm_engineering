# Module 2: Joint Control, Calibration, and Forward Kinematics

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: command joints on the toy arm first, then on Panda, using position/velocity control, enforcing limits, and executing safe multi-joint moves.
  2. On __real system/hardware__: perform homing/zeroing, apply joint offsets, and verify repeatable motion on your physical arm (Panda hardware is optional).

- __Prerequisites:__
  - Module 1 workspace + URDF/Xacro + bringup launch
  - A basic `ros2_control` setup (sim controller manager or hardware skeleton)

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.10-16 (FK + DH)
  - `Notes/part_3_motion.pdf` p.2-3 (control goal + feedback control overview)
  - `Notes/part_3_motion.pdf` p.12-18 (PD/PID + gravity compensation intuition)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy arm: derive FK with DH and validate everything with simple joint sets.
  - Panda: apply the same command/limits concepts on a full 7-DOF arm with standard tooling.

- __Deliverables (Must Produce):__
  - Controller config(s) for toy arm + Panda (position/velocity controllers + limits)
  - `src/arm_tools/` (or `scripts/`) containing:
    - a single-joint command tool
    - a multi-joint “safe move” tool (limits + rate limiting)
  - `docs/calibration.md` describing homing + offsets procedure
  - FK verification results (CSV or text summary)

- __Key Steps (Execution Path):__
  - Bring up controller manager + load joint controllers
  - Command one joint, then multiple joints with a simple interpolation (ramp) on toy → then Panda
  - Implement homing/zeroing flow (switches/encoders/markers or manual index)
  - Apply offsets in software and store calibration parameters
  - Implement FK calculation and compare with TF in RViz

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ joint naming, command APIs, joint limits, FK math, target definitions.
  - __What changes in real:__ backlash, stiction, gravity sag, offset drift, controller tuning, safety constraints.
  - __Minimum validation:__ command a 3-point pose set and return “home” with max error below your threshold (define it).
  - __Failure modes + checks (at least 3):__
    - Joints oscillate → reduce gains/velocity/accel; confirm units and limits.
    - Wrong zero offsets → re-run homing; verify sign conventions per joint axis.
    - FK mismatch → confirm joint order, axis directions, and `origin` transforms in URDF.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - Position vs velocity control; limits (pos/vel/acc/effort)
   - Homing/zeroing concepts (reference index vs encoder zero)
   - FK as a chain of transforms
   - Toy vs Panda: FK is the same math; controllers must respect different joint limits
2. __Implementation__
   - Configure controllers and expose a “safe move” command path
   - Add calibration parameter storage (YAML) and apply at startup
   - Compute FK end-effector pose from joint states
3. __Validation__
   - Plot commanded vs measured joint states
   - Compare FK pose to TF pose numerically for several joint sets

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Execute homing safely; set offsets; confirm limit switch/hard stop behavior
2. __Runtime Testing__
   - Repeatability test: move to 3 poses, return home, repeat N times
3. __Debugging__
   - Identify joint direction flips, encoder sign errors, or loose couplers

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run controller bringup and command joints via CLI/tooling
- Run a repeatability script that logs joint states across cycles

### Pictorial (Visual / Intuition)
- RViz: joint motion + end-effector marker
- Plots: commanded vs measured joint positions over time

### Abstract (Math / Architecture / Theory)
- FK pipeline: joint angles → transforms → end-effector pose
- Where calibration offsets enter the chain (joint position offsets)

---

## Updated Video List

> Naming convention:
> `M2_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M2_V01_Outcome_and_Safety_Model_[F]__
   - __Content__: What “safe joint control” means, limits we enforce, and what you’ll submit.
   - __Notes__: `Notes/part_3_motion.pdf` p.2-3 (what control tries to achieve)

2. __M2_V02_FK_and_Control_Loops_Under_the_Hood_[F]__
   - __Content__: DH/FK as the “math layer” under URDF/TF, plus how a joint controller closes the loop (PD/PID intuition, limits, and why tuning matters).
   - __Notes__: `Notes/part_1_kinematics.pdf` p.10-16 (FK + DH) + `Notes/part_3_motion.pdf` p.2-3 and p.12-16 (feedback + PD/PID)

3. __M2_V03_Homing_Offsets_and_Repeatability_[A]__
   - __Content__:
     - __Context__: Real arms need a consistent “zero” to make planning and picking repeatable.
     - __Analogy Start__: “Like tuning a guitar: each string has to be set to a reference pitch before songs sound right.”
     - __Explanation__: Homing methods (switch, hard-stop + current spike, absolute encoder), offsets, and why repeatability matters.
     - __Animation Style__: 2D arm with a “zero marker” per joint; show repeated cycles tightening around a target.
     - __Process__:
       1. Show unknown start position → homing move.
       2. Detect reference → set zero.
       3. Apply offsets → consistent coordinates.
       4. Repeat test loop → visualize drift vs stability.
   - __Notes__: (Homing/zeroing procedures are hardware-specific; not covered in `Notes/`)

4. __M2_V04_Controller_Manager_and_Joint_Commanding_[S]__
   - __Content__: Bring up controller manager, load controllers, command one joint and then multiple joints on toy → then Panda.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-16 (PD/PID regulation intuition)

5. __M2_V05_FK_Checks_TF_vs_Computed_Pose_[S]__
   - __Content__: Compute FK from joint states and compare numerically to TF; capture a CSV of pose deltas (toy + Panda).
   - __Notes__: `Notes/part_1_kinematics.pdf` p.10-16 (FK + DH steps)

#### Part B: Real System / Hardware
6. __M2_V06_Real_Homing_Runbook_and_Safety_Checks_[S]__
   - __Content__: Step-by-step homing flow, emergency stop, limit tests, and storing offsets.
   - __Notes__: (Homing/zeroing is implementation-specific; not covered in `Notes/`)

7. __M2_V07_Repeatability_Test_and_Debugging_Slippage_[S]__
   - __Content__: Run repeatability cycles, interpret logs, and diagnose common mechanical errors (slip, backlash).
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (symptoms of under/over-tuning) as background

---

## Assignments & Practice Tasks

1. __M2_A01_Safe_Move_Tool__
   - __Goal__: create a tool that moves joints to targets with rate limiting.
   - __Steps__: read current state; compute ramp; enforce limits; command controller; run on toy → then Panda using the same tool/config switch.
   - __Submission__: PR + 30–60s demo video showing toy + Panda.
   - __Pass Criteria__: no joint exceeds configured limits; smooth interpolation (no jumps) on both robots.

2. __M2_A02_Calibration_Runbook__
   - __Goal__: document your homing/zeroing procedure.
   - __Steps__: list safety checks; steps; failure handling; where offsets are stored.
   - __Submission__: `docs/calibration.md`.
   - __Pass Criteria__: includes steps, safety checks, and storage/loading details.

3. __M2_A03_FK_vs_TF_Numeric_Check__
   - __Goal__: compare computed FK to TF for 10 random joint sets (toy + Panda).
   - __Steps__: sample joint sets; compute FK; read TF; compute deltas.
   - __Submission__: CSV + summary of max/mean position and orientation error.
   - __Pass Criteria__: errors are explainable; if large, you identify which joint/frame is wrong.

4. __M2_A04_Debugging_Drill_Flipped_Joint_Axis__
   - __Goal__: fix a sign mistake in one joint axis (toy), then sanity-check the same symptom on Panda.
   - __Steps__: reproduce mismatch; diagnose; correct URDF axis/origin; re-verify.
   - __Submission__: writeup + before/after end-effector error metric.
   - __Pass Criteria__: after fix, FK vs TF error drops significantly.

---

## Final Summary

- You can command joints safely in sim (and on real hardware if available).
- You have a repeatable zeroing approach and stored offsets.
- You can compute FK and verify it against TF numerically.
- Next module adds IK + time-parameterized trajectories and the “why” of dynamics.
