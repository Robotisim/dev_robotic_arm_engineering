# Module 3: Inverse Kinematics, Trajectories, and Dynamics Basics

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: move the end-effector to target poses using IK + smooth trajectories on the toy arm first, then on Panda.
  2. On __real system/hardware__: execute the same trajectories within safe limits while accounting for gravity/load and tuning.

- __Prerequisites:__
  - Module 2 joint control + FK verification
  - End-effector frame defined (e.g., `tool0`)

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.17-22 (IK: geometric + numerical)
  - `Notes/part_1_kinematics.pdf` p.23-38 (Jacobian + velocity/acceleration kinematics + singularities)
  - `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning + 5th-order polynomial)
  - `Notes/part_2_dynamics.pdf` p.2-8 and p.18-27 (Lagrangian + D/C/G form)
  - `Notes/part_3_motion.pdf` p.19-21 (tracking/inverse dynamics control intuition)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy arm: implement and visualize IK branches and singularities with minimal DOF.
  - Panda: run the same “go to pose” idea using a library kinematics pipeline (compare behavior and failure modes).

- __Deliverables (Must Produce):__
  - An IK solver node with a clear API:
    - input: target pose + seed
    - output: joint solution + status
    - include both: toy-arm IK (your implementation) and Panda IK (library-backed, wrapped behind the same API)
  - A time-parameterized trajectory generator (limits-aware)
  - Validation logs: success rate, pose error, and trajectory constraint checks

- __Key Steps (Execution Path):__
  - Define pose targets in a consistent frame (`base_link` or `world`)
  - Implement IK with joint limits + seed handling + timeout
  - Validate IK on toy → then run the same targets on Panda and compare convergence/failure modes
  - Generate trajectories (vel/acc limited) from current → goal joints
  - Execute and measure: final pose error + constraint adherence
  - Introduce dynamics intuition: gravity/load affects required torque and tuning

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ IK math, frames, targets, joint limits, trajectory constraints.
  - __What changes in real:__ gravity sag, motor torque limits, friction/backlash, thermal throttling, payload shifts.
  - __Minimum validation:__ 20 random reachable poses: IK success rate ≥ threshold you set; final pose error reported.
  - __Failure modes + checks (at least 3):__
    - IK converges to wrong branch → change seed strategy; add joint-weighting/constraints.
    - Trajectory violates limits → verify units and limit config; clamp and re-time.
    - Real arm stalls or chatters → reduce accel/vel; check torque/current limits; add gravity compensation if available.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - IK: multiple solutions, singularities, and reachability
   - Time-parameterization: smoothness for actuators
   - Dynamics basics: gravity + load change required torque
   - Toy vs Panda: what the library automates, and what assumptions it still relies on (frames, limits, seeds)
2. __Implementation__
   - Build an IK service/action for “pose → joints”
   - Build a trajectory generator (trapezoidal / S-curve) with joint limits
3. __Validation__
   - Report IK success rate + pose error distribution
   - Assert trajectory never exceeds limits (pos/vel/acc)

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Safe velocity/acc caps; emergency stop path
2. __Runtime Testing__
   - Run a small pose suite with payload/no-payload comparison (optional)
3. __Debugging__
   - Identify singularity symptoms, oscillation, or missed steps/encoder lag

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Command “go to pose” requests and watch the arm move
- Run a batch test: random pose targets → IK solve → execute → log error

### Pictorial (Visual / Intuition)
- RViz: target marker vs achieved end-effector marker
- Plots: pose error histogram; joint velocity profiles over time

### Abstract (Math / Architecture / Theory)
- Jacobian intuition; singularities; multiple IK branches
- Trajectory constraints and time-scaling

---

## Updated Video List

> Naming convention:
> `M3_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M3_V01_Outcome_IK_plus_Trajectories_[F]__
   - __Content__: End-effector goals, expected success metrics, and how we’ll validate.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.17-22 (IK overview) + p.39-46 (trajectory planning)

2. __M3_V02_Dynamics_and_Control_Under_the_Hood_[F]__
   - __Content__: What dynamics terms (`D`, `C`, `G`) mean, why gravity matters, and how tracking/inverse-dynamics control relates to smooth execution in real arms.
   - __Notes__: `Notes/part_2_dynamics.pdf` p.2-8 and p.18-27 (D/C/G) + `Notes/part_3_motion.pdf` p.17-21 (gravity comp + tracking control)

3. __M3_V03_IK_Branches_and_Singularities_[A]__
   - __Content__:
     - __Context__: IK failure is often geometry: singularities and multiple solutions.
     - __Analogy Start__: “Reaching a shelf: you can use an overhand or underhand motion—both work, until you hit a cramped corner.”
     - __Explanation__: Two-solution IK, workspace boundary, and how seeds pick a branch.
     - __Animation Style__: Minimal 2D arm + reachable workspace; show “elbow-up” and “elbow-down” in different colors.
     - __Process__:
       1. Pick a target inside workspace → show two solutions.
       2. Move target toward boundary → solutions collapse.
       3. Move target near singularity → tiny pose change causes huge joint change.
       4. Show seed choosing the branch; joint limits rejecting invalid solutions.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.17-22 (IK) + p.23-36 (Jacobian + singularities)

4. __M3_V04_Implementing_an_IK_Service_[S]__
   - __Content__: Build `pose_to_joints` service/action, include limits, seeds, and timeouts; demo on toy → then Panda.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.17-22 (IK methods) + p.23-27 (Jacobian setup)

5. __M3_V05_Trajectory_Time_Parameterization_and_Limit_Checks_[S]__
   - __Content__: Implement trapezoidal/S-curve timing, validate vel/acc bounds, and log joint profiles.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning + 5th-order polynomial)

#### Part B: Real System / Hardware
6. __M3_V06_Real_Execution_Tuning_for_Smooth_Motion_[S]__
   - __Content__: Practical tuning knobs (max vel/acc, gains), and what “too aggressive” looks like.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (tuning symptoms) + p.19-21 (tracking control intuition)

7. __M3_V07_Payload_and_Gravity_Effects_Debugging_[S]__
   - __Content__: Demonstrate the same motion with/without payload (or simulated gravity), interpret symptoms, and mitigate.
   - __Notes__: `Notes/part_2_dynamics.pdf` p.2-8 (energy/dynamics basics) + `Notes/part_3_motion.pdf` p.17-18 (gravity compensation)

---

## Assignments & Practice Tasks

1. __M3_A01_IK_Solver_API__
   - __Goal__: implement a pose-to-joints solver with success/failure status (toy IK + Panda IK behind the same API).
   - __Steps__: implement toy IK; add limits/seeds/timeouts; call Panda IK via library plugin; normalize outputs and error codes.
   - __Submission__: PR + demo video moving to 5 target poses in RViz on toy + Panda.
   - __Pass Criteria__: unreachable poses return failure; joint limits always respected; consistent behavior across both robots.

2. __M3_A02_Trajectory_Profile_Logs__
   - __Goal__: generate a smooth trajectory and plot joint velocities.
   - __Steps__: generate path; time-parameterize; log velocity/acc; plot.
   - __Submission__: plot image + CSV.
   - __Pass Criteria__: no velocity/acc limit violations in logs.

3. __M3_A03_Random_Pose_Benchmark__
   - __Goal__: benchmark 20 random reachable pose targets on toy + Panda.
   - __Steps__: sample targets; solve IK; execute; compute pose error; compare success rate and failure reasons.
   - __Submission__: summary table (success rate, mean/max error, worst-case target) for each robot.
   - __Pass Criteria__: documented thresholds + explanation for failures and differences.

4. __M3_A04_Debugging_Drill_Singularity_Case__
   - __Goal__: reproduce an IK instability near a singularity and mitigate it.
   - __Steps__: pick target; observe instability; adjust seed/constraints; re-test.
   - __Submission__: short writeup + before/after metric.
   - __Pass Criteria__: improved convergence stability (fewer failures or lower error).

---

## Final Summary

- You can command end-effector poses using IK with limits-aware behavior.
- You can generate smooth, time-parameterized trajectories that respect constraints.
- You can explain why real motion differs (gravity/load/tuning) and adjust safely.
- Next module integrates MoveIt 2 for full planning and collision constraints.
