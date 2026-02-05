# Module 8: Robot Learning and Language-to-Action Tasking

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: train/evaluate a small policy for a constrained motion primitive (e.g., approach or insertion) and wrap it with safety checks.
  2. On __real system/hardware__: optionally deploy only with strict constraints, low speed, and a “human-in-the-loop” kill switch.

- __Prerequisites:__
  - Module 7 evaluation harness (metrics + safe recovery)
  - A stable simulation environment for repeatable training/evaluation

- __Notes (Optional Theory Refresh):__
  - `Notes/part_3_motion.pdf` p.19-21 (inverse dynamics / tracking control background)
  - `Notes/part_2_dynamics.pdf` p.18-27 (D/C/G form background for “model-based baselines”)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Toy environment: train/debug quickly with minimal state and fewer failure modes.
  - Panda environment: evaluate realism (more DOF, collisions, perception noise, timing issues).

- __Deliverables (Must Produce):__
  - `learning/` folder containing:
    - training script or notebook
    - evaluation script producing success rate + plots
  - A “skill library” interface with consistent inputs/outputs:
    - `pick_fixed`, `place_fixed`, `approach_policy`, `home`, `retreat`
  - A language-to-action router that maps text → skill call with parameters

- __Key Steps (Execution Path):__
  - Define a constrained skill task + observation/action spaces
  - Train policy in sim and evaluate with your metrics harness
  - Wrap policy with safety: workspace bounds, speed caps, collision checks, fallback
  - Implement language router: parse instruction → select skill → execute safely

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ skill API, safety wrappers, metrics, high-level routing logic.
  - __What changes in real:__ sim-to-real gaps, unmodeled dynamics, perception noise, safety risk.
  - __Minimum validation:__ evaluation report + ablation comparing policy vs scripted baseline.
  - __Failure modes + checks (at least 3):__
    - Policy exploits sim artifacts → randomize sim; add domain constraints.
    - Unsafe outputs → clamp actions; add collision checks and emergency abort.
    - Language routing ambiguity → require confirmations and strict parameter schemas.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - Why learning helps: contact-rich or hard-to-model skills
   - Safety wrappers as non-negotiable guardrails
   - Skill-based tasking: language selects skills, not raw motor commands
   - Toy vs Panda: prototype the skill, then validate it under realistic constraints
2. __Implementation__
   - Train a small policy for one primitive (approach/insertion)
   - Implement a skill registry + executor interface
   - Implement language routing with constrained schema (allowed skills + parameters)
3. __Validation__
   - Compare policy vs baseline; report success rate and failure types

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Conservative speeds; physical safety perimeter; clear stop procedures
2. __Runtime Testing__
   - Start with “shadow mode” (policy proposes, human approves)
3. __Debugging__
   - Log policy actions + observations; replay in sim; iterate with metrics

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run training + evaluation scripts; generate plots and metrics
- Run language commands that trigger safe skills (no freeform execution)

### Pictorial (Visual / Intuition)
- Learning curves (reward/success vs training steps)
- Skill graph: router → skill → safety wrapper → execution

### Abstract (Math / Architecture / Theory)
- Reward shaping basics; observation/action constraints
- Why “language-to-skill” is safer than “language-to-joint-torques”

---

## Updated Video List

> Naming convention:
> `M8_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M8_V01_Outcome_Learning_plus_Language_Layer_[F]__
   - __Content__: Optional/advanced module; safety-first; skill-based tasking.
   - __Notes__: (RL/VLA topics are not covered in `Notes/`; listed notes are for control/dynamics refresh)

2. __M8_V02_Skills_vs_LowLevel_Control_Theory_[F]__
   - __Content__: Why we route language to skills (not torques), what a “safety wrapper” checks, and how the low-level controller still matters even with a learned policy.
   - __Notes__: `Notes/part_3_motion.pdf` p.2-3 (control overview) + p.19-21 (tracking control background)

3. __M8_V03_Skills_Safety_and_Policies_[A]__
   - __Content__:
     - __Context__: Learning outputs must be boxed in by safety; language should only select pre-validated skills.
     - __Analogy Start__: “Like a menu: you choose dishes, not the stove temperature and raw ingredients.”
     - __Explanation__: Skill registry, parameter validation, safety wrapper, fallback behaviors, and human approval mode.
     - __Animation Style__: Router feeding into skill boxes; safety gate as a barrier; failures route to recovery.
     - __Process__:
       1. Text instruction arrives → parser extracts intent.
       2. Router selects skill + parameters.
       3. Safety gate validates bounds/collisions/limits.
       4. Execute skill; on failure → recovery; log metrics.
   - __Notes__: (Skill routing concepts are not covered in `Notes/`; see `Notes/part_3_motion.pdf` p.2-3 for “why safety wrappers exist”)

4. __M8_V04_Training_a_Constrained_Primitive_in_Sim_[S]__
   - __Content__: Define task, run training, save checkpoints.
   - __Notes__: (RL training is not covered in `Notes/`)

5. __M8_V05_Evaluation_Report_and_Ablation_[S]__
   - __Content__: Evaluate policy vs scripted baseline; generate plots + failure breakdown.
   - __Notes__: (RL evaluation is not covered in `Notes/`)

#### Part B: Real System / Hardware (Optional)
6. __M8_V06_Shadow_Mode_and_Human_Approval_Execution_[S]__
   - __Content__: Policy proposes, human confirms; strict speed and workspace limits.
   - __Notes__: `Notes/part_3_motion.pdf` p.12-18 (why conservative tuning matters) as background

7. __M8_V07_Debugging_Policy_Failures_with_Replay_[S]__
   - __Content__: Capture observations/actions, replay in sim, iterate with metrics.
   - __Notes__: (RL debugging is not covered in `Notes/`; use the same metrics discipline as Module 7)

---

## Assignments & Practice Tasks

1. __M8_A01_Skill_Library_Interface__
   - __Goal__: implement a skill registry with at least 5 skills (including `home` and `retreat`).
   - __Submission__: PR + demo script.
   - __Pass Criteria__: consistent inputs/outputs; failure codes are standardized.

2. __M8_A02_Policy_Training_and_Checkpoint__
   - __Goal__: train a policy for one constrained primitive.
   - __Submission__: checkpoint file + training curve plot.
   - __Pass Criteria__: reproducible run parameters; curve is included.

3. __M8_A03_Evaluation_vs_Baseline__
   - __Goal__: compare policy to scripted baseline on N trials.
   - __Submission__: report (success rate, mean time, top failure types).
   - __Pass Criteria__: clear comparison and interpretation of results.

4. __M8_A04_Language_Router_with_Safety_Schema__
   - __Goal__: route text commands to skills with strict parameter validation.
   - __Submission__: demo video + example instruction set + logs.
   - __Pass Criteria__: rejects ambiguous/unsafe requests; executes only allowed skills.

---

## Final Summary

- You can build a safe, skill-based task layer where language selects validated skills.
- You can train and evaluate a constrained learning policy in simulation with metrics.
- You can deploy cautiously using shadow mode and strict safety wrappers (optional).
- This closes the learning path with an extensible framework for more autonomy.
