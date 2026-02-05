# Module 4: MoveIt 2 Planning and Collision Avoidance

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: plan and execute collision-free motions with MoveIt 2 (Panda as the primary reference arm), while using the toy arm as a simplified model to explain the concepts.
  2. On __real system/hardware__: run the same planning pipeline with conservative execution settings and a mirrored collision scene.

- __Prerequisites:__
  - Module 1–3: arm model + controllers + IK/trajectories baseline
  - End-effector frame + planning group definition expectations

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning background)
  - `Notes/part_1_kinematics.pdf` p.23-36 (Jacobian/singularities background)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy arm: explain “what planning is” without 7-DOF complexity.
  - Panda: implement the full MoveIt pipeline (SRDF, planners, planning scene, execution).

- __Deliverables (Must Produce):__
  - `src/arm_moveit_config/` generated and edited for your arm
  - A demo node/script that:
    - adds at least 2 collision objects
    - plans to 3 targets and executes
  - Benchmark CSV: planning success rate + planning time for 10 trials

- __Key Steps (Execution Path):__
  - Generate MoveIt config (SRDF, planning groups, controllers)
  - Launch MoveIt 2 + RViz planning scene
  - Add collision objects and constraints
  - Execute planned trajectories in sim; mirror scene for real

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ SRDF groups, frames, collision geometry, constraints, planning requests.
  - __What changes in real:__ execution scaling, controller timing, safety margins, collision padding.
  - __Minimum validation:__ 10 planned motions with 2 obstacles: ≥ target success rate and no collisions reported.
  - __Failure modes + checks (at least 3):__
    - Planning fails unexpectedly → check joint limits, start state validity, collision geometry.
    - Trajectory executes but deviates → lower scaling; check controller latency/jitter.
    - Scene mismatch real vs sim → confirm obstacle frames/poses; tune padding and update rate.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - MoveIt 2 components: planning scene, planners, kinematics plugin, controllers
   - Collision objects and constraints as “rules for motion”
   - Toy vs Panda: planning ideas stay the same; configuration size and constraints change
2. __Implementation__
   - Create `arm_moveit_config` and tune group/limits/controllers
   - Publish collision objects (boxes/cylinders) into the planning scene
3. __Validation__
   - Repeatable planning benchmark: plan time + success rate + collision-free paths

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Conservative execution scaling; safety stop; validate real joint state feedback stability
2. __Runtime Testing__
   - Mirror obstacles (measured fixtures) and validate on bench
3. __Debugging__
   - Frame alignment mistakes; collision padding; controller timing issues

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Launch MoveIt and plan to targets in RViz
- Add/remove obstacles and confirm planner behavior changes

### Pictorial (Visual / Intuition)
- RViz planning scene: collision geometry + planned path
- “With obstacle” vs “without obstacle” comparison

### Abstract (Math / Architecture / Theory)
- Configuration space intuition (high level)
- Planning scene as the shared world model

---

## Updated Video List

> Naming convention:
> `M4_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M4_V01_Outcome_MoveIt_Pipeline_[F]__
   - __Content__: What MoveIt provides, what must be configured, and success criteria.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning concepts; MoveIt internals not covered in `Notes/`)

2. __M4_V02_MoveIt_Pipeline_Under_the_Hood_[F]__
   - __Content__: What MoveIt actually computes (IK → planning request → collision checking → time parameterization → controller execution), and where failures typically come from.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.39-46 (trajectory planning background; MoveIt planners/collision checking not covered in `Notes/`)

3. __M4_V03_PlanningScene_and_Collision_Intuition_[A]__
   - __Content__:
     - __Context__: Collision checking is the guardrail that makes planning safe.
     - __Analogy Start__: “Like navigating a room: you need a map of furniture, not just a destination.”
     - __Explanation__: Planning scene holds robot + world geometry; planners search for collision-free paths; padding creates safety margins.
     - __Animation Style__: 2D “room” with obstacles; planner search visualized as expanding wavefronts.
     - __Process__:
       1. Show goal-only motion hitting an obstacle.
       2. Add obstacle → collision check blocks direct path.
       3. Planner explores alternate route → valid path appears.
       4. Increase padding → path becomes more conservative.
   - __Notes__: (Collision checking + sampling-based planning are not covered in `Notes/`)

4. __M4_V04_Generating_MoveIt_Config_and_Launching_RViz_[S]__
   - __Content__: Use setup assistant, validate groups, and launch MoveIt RViz plugin (Panda-first; toy arm optional).
   - __Notes__: (MoveIt setup assistant workflow; not covered in `Notes/`)

5. __M4_V05_Adding_Collision_Objects_and_Benchmarking_[S]__
   - __Content__: Add obstacles via code, measure plan time/success rate, and write a CSV.
   - __Notes__: (MoveIt planning scene APIs; not covered in `Notes/`)

#### Part B: Real System / Hardware
6. __M4_V06_Real_Execution_Scaling_and_Safety_Margins_[S]__
   - __Content__: Set conservative scaling; validate start state; run first real plans safely.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (why conservative gains/speeds matter) as background

7. __M4_V07_Debugging_Frames_and_Scene_Mismatch_[S]__
   - __Content__: Diagnose wrong object poses, wrong frames, and collision padding issues on real setups.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (frame/transform debugging mindset)

---

## Assignments & Practice Tasks

1. __M4_A01_MoveIt_Config_Package__
   - __Goal__: create and commit `arm_moveit_config`.
   - __Submission__: PR + screenshot of RViz MoveIt panel with your planning group.
   - __Pass Criteria__: planning group loads; robot state updates; no red errors.

2. __M4_A02_Collision_Object_Demo__
   - __Goal__: add two collision objects and plan around them.
   - __Submission__: 60–90s demo video + code link.
   - __Pass Criteria__: planner finds a path avoiding collisions; objects visible in RViz.

3. __M4_A03_Planning_Benchmark__
   - __Goal__: run 10 plans and record timing/success.
   - __Submission__: CSV + summary (mean/95th percentile plan time).
   - __Pass Criteria__: results reproducible; notes on failures included.

4. __M4_A04_Debugging_Drill_Bad_Collision_Frame__
   - __Goal__: publish an obstacle in the wrong frame and fix it.
   - __Submission__: writeup + screenshot before/after.
   - __Pass Criteria__: corrected obstacle pose; planning behavior matches expectation.

---

## Final Summary

- You have a working MoveIt 2 configuration for your arm.
- You can plan and execute collision-free motions with obstacles in the planning scene.
- You understand real-vs-sim differences (padding, scaling, frame alignment).
- Next module builds a repeatable pick/place routine using structured motion.
