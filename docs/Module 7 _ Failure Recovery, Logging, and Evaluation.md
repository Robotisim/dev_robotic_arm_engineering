# Module 7: Failure Recovery, Logging, and Evaluation

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: wrap your manipulation pipeline with recovery behaviors and produce repeatable evaluation metrics.
  2. On __real system/hardware__: record bags, diagnose failures, and improve reliability with structured retries and safe fallback states.

- __Prerequisites:__
  - Module 4–6: planning + pick/place + (optional) perception

- __Notes (Optional Theory Refresh):__
  - `Notes/part_3_motion.pdf` p.2-3 (feedback control overview)
  - `Notes/part_3_motion.pdf` p.12-18 (PD/PID tuning intuition for “why motion got unstable”)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Panda pipeline: collect realistic failures (planning, collisions, perception jitter).
  - Toy arm: reproduce and isolate “one variable at a time” when debugging.

- __Deliverables (Must Produce):__
  - A recovery-enabled task executor (FSM or Behavior Tree) with states:
    - `home`, `pregrasp`, `grasp`, `lift`, `place`, `retreat`, `recover`
  - A run script that records bags + metadata per trial (e.g., `scripts/record_run.sh`)
  - A metrics generator that outputs:
    - success rate, mean/95th percentile cycle time, retry counts, top failure codes

- __Key Steps (Execution Path):__
  - Define failure codes and a minimal FSM/BT
  - Add recovery actions: retry grasp, re-localize pose, safe retreat, home
  - Record bags and metadata for each run
  - Run batch evaluations and generate a metrics report

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ executor logic, failure codes, metrics computation, bag pipeline.
  - __What changes in real:__ more failure variety (slip, occlusion, latency), safety triggers, operator intervention.
  - __Minimum validation:__ run N trials (define N) and generate a metrics report with at least 4 metrics.
  - __Failure modes + checks (at least 3):__
    - Perception drops frames → log timestamps; check QoS; add re-localize step.
    - Grasp fails intermittently → log pregrasp pose; add retry with offset pattern.
    - Planner fails mid-run → safe retreat, clear scene/map, retry from known state.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - Robust autonomy: recovery beats perfection
   - Observability: logs/bags/metrics make iteration possible
2. __Implementation__
   - Implement FSM/BT around existing pick/place pipeline
   - Standardize failure codes + bounded retries
   - Record bags + metadata per run
3. __Validation__
   - Batch trials with metrics report and top failure reasons

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Safe “pause/abort” mechanisms; operator checklist
2. __Runtime Testing__
   - Run repeated trials on bench; compare revisions via metrics
3. __Debugging__
   - Use bag replay + RViz to reproduce and fix failures

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run 20 trials with logging enabled
- Reproduce a failure from bag replay and confirm the fix

### Pictorial (Visual / Intuition)
- State machine timeline (state transitions over time)
- Metrics dashboard: success rate + retries + cycle time histogram

### Abstract (Math / Architecture / Theory)
- Why structured recovery increases long-run success probability
- Metrics design: what to measure and why it matters

---

## Updated Video List

> Naming convention:
> `M7_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M7_V01_Outcome_Robustness_Loop_[F]__
   - __Content__: “Build → run → log → measure → improve” loop and what gets submitted.
   - __Notes__: (Robustness/metrics topics are not covered in `Notes/`; see `docs/Notes Index.md` for math refresh topics)

2. __M7_V02_Metrics_and_Evaluation_Theory_[F]__
   - __Content__: What to measure (success rate, time, retries, pose error), how to design experiments, and how to turn logs into actionable engineering decisions.
   - __Notes__: (Metrics/experiment design is not covered in `Notes/`)

3. __M7_V03_StateMachines_and_Recovery_Patterns_[A]__
   - __Content__:
     - __Context__: Manipulation tasks fail for many reasons; recovery structures keep the system safe and productive.
     - __Analogy Start__: “Like a pilot checklist: when something goes wrong, you follow a known procedure before continuing.”
     - __Explanation__: FSM/BT nodes, failure codes, retries with bounded attempts, safe retreat and home.
     - __Animation Style__: Flowchart with token movement; failures branch to recovery then rejoin.
     - __Process__:
       1. Show a normal successful run path.
       2. Inject “grasp failed” → recovery branch → retry.
       3. Inject “planning failed” → retreat → home.
       4. Show max-retry reached → abort safely.
   - __Notes__: (FSM/BT patterns are not covered in `Notes/`)

4. __M7_V04_Implementing_TaskExecutor_FSM_or_BT_[S]__
   - __Content__: Add states, transitions, and failure codes around your pick/place pipeline.
   - __Notes__: (FSM/BT patterns are not covered in `Notes/`)

5. __M7_V05_Bagging_and_Metrics_Report_Generation_[S]__
   - __Content__: Record bags + metadata, generate metrics report, interpret results.
   - __Notes__: (Logging/metrics are not covered in `Notes/`)

#### Part B: Real System / Hardware
6. __M7_V06_Real_Run_Safety_Checklist_and_Abort_Paths_[S]__
   - __Content__: Operator checklist, safe pause/abort, and what to do when something looks wrong.
   - __Notes__: (Safety process is not covered in `Notes/`)

7. __M7_V07_Debugging_From_Bags_Replay_to_Fix_[S]__
   - __Content__: Use bag replay + RViz to reproduce a failure and validate the fix.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (control instability symptoms) as background when diagnosing motion issues

---

## Assignments & Practice Tasks

1. __M7_A01_TaskExecutor_Skeleton__
   - __Goal__: implement FSM/BT with at least 6 states and 3 failure codes.
   - __Submission__: PR + state diagram screenshot.
   - __Pass Criteria__: state transitions occur as expected on success and failure.

2. __M7_A02_Logging_and_Bag_Runbook__
   - __Goal__: create a one-command run that records bags + metadata.
   - __Submission__: `scripts/record_run.sh` + `docs/runbook.md`.
   - __Pass Criteria__: bag includes required topics; metadata includes config hash + run ID.

3. __M7_A03_Metrics_Report__
   - __Goal__: run 20 trials and generate a metrics report.
   - __Submission__: report file + plots (cycle time, retries).
   - __Pass Criteria__: includes success rate, mean/95th percentile cycle time, and top 3 failure codes.

4. __M7_A04_Debugging_Drill_Forced_Grasp_Failure__
   - __Goal__: force a grasp failure (wrong offset) and verify recovery works.
   - __Submission__: before/after metrics + short writeup.
   - __Pass Criteria__: recovery triggers and returns to a safe state reliably.

---

## Final Summary

- You have a recovery-aware execution layer (FSM/BT) for manipulation.
- You can record data consistently and generate metrics that guide improvements.
- You can reproduce failures via bag replay and fix them systematically.
- Next module explores learning-based motion primitives and language tasking (optional/advanced).
