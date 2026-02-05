# Universal Robotics Curriculum Markdown Authoring Guide

This file defines how content agents must create **learning path modules** as Markdown files for **any robotics curriculum** (Mobile Robots, Robotic Arms, Drones, Perception, SLAM, RL, Embedded, Controls, etc.).
It is optimized for the workflow: **Founder gives lecture/thoughts → agents produce structured modules + lecture videos + assignments**.

---

## 0) Output Rules (Non-Negotiable)

1. **One module per file**, named:
   - `Module X _ <Module Title>.md`
2. Use **this exact section ordering** in every module file:
   - `# Module X: <Title>`
   - `## 1. Purpose & Scope`
   - `## 2. Curriculum Focus`
   - `## 3. Concrete–Pictorial–Abstract (CPA) Breakdown`
   - `## Updated Video List`
   - `## Assignments & Practice Tasks`
   - `## Final Summary`
3. Each module must explicitly include:
   - **Simulation/Emulation track** (even if minimal)
   - **Real system track** (hardware/field/bench testing), or clearly marked “optional”
4. Keep language **practical and execution-focused**:
   - what learner builds, runs, measures, and submits
5. Keep sections **short and structured** (bullets > paragraphs).

---

## 1) How to Convert “Founder Lecture/Thoughts” into Modules

### Inputs you may receive
- A voice note, rough lecture notes, brainstorm text, or a big outline.
- Possibly messy ordering, repeated ideas, or jumps between topics.

### Your job
1. Extract the **capability progression** (what student can do after each module).
2. Split into modules where each module:
   - has a single clear outcome
   - contains a small set of concepts that support that outcome
3. For each module:
   - define **deliverables**
   - define **videos**
   - define **assignments** with measurable outputs

### If the lecture contains too much material
- Create **multiple modules** rather than an overloaded module.
- Prefer 60–120 minutes total video per module (guideline, not strict).

---

## 2) Standard Module Skeleton (Copy/Paste Template)

# Module <N>: <Module Title>

---

## 1. Purpose & Scope

- **Overall Goal:**
  1. In **simulation/emulation**: <what learner achieves>
  2. On **real system/hardware**: <what learner validates>

- **Prerequisites:**
  - <module/skills/tools required>

- **Deliverables (Must Produce):**
  - <repo folder / notebook / package / config>
  - <plots/logs/bag/metrics>
  - <short demo video link criteria>

- **Key Steps (Execution Path):**
  - <Step 1>
  - <Step 2>
  - <Step 3>
  - <Step 4>

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. **Concepts**
   - <what concepts matter>
2. **Implementation**
   - <what learner implements>
3. **Validation**
   - <how learner verifies it works>

### Part B: Real System / Hardware Track (or Optional)

1. **Bringup**
   - <drivers, interfaces, calibration>
2. **Runtime Testing**
   - <tests, limits, safety>
3. **Debugging**
   - <common failures + checks>

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- <commands they run, nodes they launch, physical tests, experiments>

### Pictorial (Visual / Intuition)
- <RViz/Gazebo plots, diagrams, animations, dashboards>

### Abstract (Math / Architecture / Theory)
- <equations, transforms, control loops, state machines, data flow>

---

## Updated Video List

> Naming convention:
> `M<N>_V<NN>_<Title>_[F|S|A]`
> - `[F]` face recording (teaching/explaining)
> - `[S]` screen recording (demo/implementation)
> - `[A]` animation (editor instructions included)
> - Optional `[U]` prefix for “update later” items

#### Part A: Simulation / Emulation
1. **M<N>_V01_<Intro & Outcome>_[F]**
   - **Content**: <1–4 lines, goal + what will be built>

2. **M<N>_V02_<Core Concept Animation>_[A]**
   - **Content**:
     - **Context**: <where this appears in robotics>
     - **Analogy Start**: "<simple analogy>"
     - **Explanation**: <key mental model>
     - **Animation Style**: <2D/3D, elements, vibe>
     - **Process**: <step-by-step storyboard>

3. **M<N>_V03_<Implementation Walkthrough>_[S]**
   - **Content**: <files/components shown + what is implemented>

4. **M<N>_V04_<Validation + Metrics>_[S]**
   - **Content**: <how to test + what plots/logs prove success>

#### Part B: Real System / Hardware (continue numbering)
5. **M<N>_V05_<Real Bringup & Calibration>_[S]**
   - **Content**: <setup steps + checks + safety>

6. **M<N>_V06_<Real Test + Failure Debug>_[S]**
   - **Content**: <common failures + how to diagnose>

---

## Assignments & Practice Tasks

> Each assignment must specify a measurable submission (plot/log/bag/video/PR).

1. **M<N>_A01_<TaskName>**
   - **Goal**: <what to achieve>
   - **Steps**: <brief>
   - **Submission**: <required artifacts>
   - **Pass Criteria**: <threshold/expected behavior>

2. **M<N>_A02_<TaskName>**
   - ...

3. **M<N>_A03_<TaskName>**
   - ...

4. **M<N>_A04_<TaskName>**
   - ...

---

## Final Summary

- <3–6 bullets: what learner can now do>
- <how this connects to next module>

---

## 3) Universal “Robotics Curriculum Types” Mapping

Agents must categorize each module as one (or more) of:

- **Foundations**: frames, units, timing, tooling, debugging mindset
- **Modeling**: URDF, kinematics, dynamics, sensors as models
- **Control**: PID/MPC basics, stability, tuning, constraints
- **Perception**: camera/lidar pipelines, filtering, detection, tracking
- **Mapping/SLAM**: occupancy, point clouds, loop closure concepts
- **Planning**: global/local planning, trajectory generation, cost functions
- **Autonomy Execution**: FSM/BT, mission logic, recovery behaviors
- **Robot Learning (RL/IL)**: rewards, observations/actions, training loop, eval
- **Embedded/Hardware Interface**: MCU comms, drivers, latency, safety
- **System Integration**: ROS2 graph, QoS, performance, logging, bagging

This mapping helps you split the founder lecture into modules correctly.

---

## 4) Consistency Checklist (Must Answer in Every Module)

Every module must explicitly state:

- **What stays the same** in sim vs real (interfaces, messages, math, logic)
- **What changes** in real (noise, drift, latency, calibration, safety)
- **Minimum validation** required (test + metric)
- **Failure modes** + debugging checks (at least 3)

---

## 5) Assignment Quality Bar

Each module must include 3–5 assignments spanning:

- **Simulation**: implement + verify
- **Configuration**: parameters/QoS/frames/tuning
- **Debugging**: injected error or broken setup to fix
- Optional: **comparison** (A/B) to build engineering judgment

---

## 6) Recommended Folder Layout (Conceptual)

learning-paths/
  <curriculum-name>/
    Module 1 _ <...>.md
    Module 2 _ <...>.md
    ...

Do not assume hidden context. Each module must stand alone.

---

## 7) Pitfalls to Avoid

- Vague goals like “understand concept”
- Long unstructured paragraphs
- Video list without clear “what will be shown/built”
- No measurable criteria (no plots/logs/bags/videos)
- Skipping CPA
- Skipping real-system consideration (even if optional)

---

## 8) Definition of Done (DoD)

A module `.md` is complete when it has:
- correct headings/order
- sim + real track coverage
- at least:
  - 1 intro `[F]`
  - 1 animation `[A]` with full storyboard
  - 2+ validation/debug `[S]`
- 3–5 assignments with measurable submissions
- a clear final summary linking to the next module
