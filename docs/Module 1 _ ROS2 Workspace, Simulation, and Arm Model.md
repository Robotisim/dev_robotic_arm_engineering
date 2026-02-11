# Module 1: ROS2 Workspace, Simulation, and Arm Model

---

## 1. Purpose & Scope

- __Overall Goal:__
  1. In __simulation/emulation__: bring up two arms side-by-side:
     - a minimal __toy arm URDF__ (2–3 DOF) you build from scratch to teach the fundamentals
     - the __Franka Panda__ model as the reference arm used in later modules
     Validate TF, joint naming, and joint limits in RViz (and optionally Gazebo).
  2. On __real system/hardware__: scaffold a `ros2_control` hardware interface package for your physical arm (Panda hardware is optional).

- __Prerequisites:__
  - ROS2 installed (Humble/Iron/Jazzy) + `colcon` basics
  - Basic Linux CLI + Git usage

- __Notes (Primary):__
  - `Notes/part_1_kinematics.pdf` p.2-9 (frames, rotation matrices, homogeneous transforms)
  - `Notes/part_1_kinematics.pdf` p.10-16 (FK + DH, for checking joint/frame conventions)
  - `docs/Notes Index.md`

- __Teaching Setup (Toy vs Panda):__
  - Teach every new concept on the __toy arm__ first (simple chain, easy math, easy debugging).
  - Then show the same concept on the __Panda__ (realistic joint count + standard tooling support).

- __Deliverables (Must Produce):__
  - `src/toy_arm_description/` (minimal URDF/Xacro you author from scratch)
  - Panda reference bringup (dependency notes + a launch that displays Panda in RViz)
  - `src/arm_bringup/` (launch files for RViz + sim, for both toy arm and Panda)
  - `src/arm_hardware/` (skeleton `ros2_control` hardware interface plugin)
  - One short demo video:
    - RViz showing toy arm and Panda with TF tree + joint sliders moving without TF errors

- __Key Steps (Execution Path):__
  - Create ROS2 workspace + packages (`toy_arm_description`, `arm_bringup`, `arm_hardware`)
  - Model the toy arm links/joints + limits + inertias (good-enough first pass)
  - Launch RViz with `robot_state_publisher` + `joint_state_publisher_gui`
  - Bring up Panda in RViz and compare TF tree + joint naming vs the toy arm
  - (Optional) Add Gazebo/ignition sim launch and confirm the model loads
  - Implement `ros2_control` hardware interface skeleton (compile + loadable plugin)

- __Sim vs Real (Consistency Checklist):__
  - __What stays the same:__ URDF frames, joint names, joint limits, controller interfaces, TF conventions.
  - __What changes in real:__ latency, encoder noise, offsets, hard stops, safety limits, wiring/power issues.
  - __Minimum validation:__ `check_urdf` passes + RViz shows a stable TF tree (no frame spam) + `colcon build` succeeds for `arm_hardware`.
  - __Failure modes + checks (at least 3):__
    - TF tree broken → verify `robot_state_publisher` is running and `robot_description` is set.
    - Joints not moving in RViz → verify joint names match URDF and `joint_state_publisher_gui` publishes.
    - `arm_hardware` plugin not found → verify `pluginlib` export, install targets, and `ament` index registration.

---

## 2. Curriculum Focus

### Part A: Simulation / Emulation Track

1. __Concepts__
   - URDF vs Xacro; links, joints, limits, inertias
   - TF tree expectations for an arm (base → joints → end-effector)
   - Toy vs Panda: same frame math, different complexity
2. __Implementation__
   - Author a toy arm URDF/Xacro and launch RViz with sliders
   - Bring up Panda in RViz using an existing description package (reference robot)
   - Add clean bringup launches for both robots (parameters, remaps, namespaces if needed)
3. __Validation__
   - `check_urdf` + RViz shows correct frames and joint motion without warnings (toy + Panda)

### Part B: Real System / Hardware Track (or Optional)

1. __Bringup__
   - Create `ros2_control` hardware plugin skeleton implementing state + command interfaces
2. __Runtime Testing__
   - Load plugin in a minimal `ros2_control` config (no real comms yet)
3. __Debugging__
   - Plugin load errors, interface mismatch, joint name mismatch against URDF

---

## 3. Concrete–Pictorial–Abstract (CPA) Breakdown

### Concrete (Hands-On / Doing)
- Run `colcon build` + `ros2 launch arm_bringup view_arm.launch.py`
- Move joints in `joint_state_publisher_gui` and observe end-effector motion

### Pictorial (Visual / Intuition)
- RViz TF tree view + robot model overlay
- One diagram: “URDF → robot_state_publisher → TF → RViz”

### Abstract (Math / Architecture / Theory)
- Frame naming conventions (`base_link`, tool frame)
- Why limits/inertias matter later (controllers + simulation stability)

---

## Updated Video List

> Naming convention:
> `M1_V<NN>_<Title>_[F|S|A]`

#### Part A: Simulation / Emulation
1. __M1_V01_Outcome_and_Workspace_Structure_[F]__
   - __Content__: What gets built in this module, package layout, and how we’ll validate (RViz + TF + build success).
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2 (why kinematics/dynamics) + p.2-9 (frames/transforms)

2. __M1_V02_Coordinate_Frames_and_Homogeneous_Transforms_Theory_[F]__
   - __Content__: Rotation matrices, relative poses, and homogeneous transforms, then how URDF/TF encode the same math for a robot arm.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (rotation + homogeneous transforms)

3. __M1_V03_URDF_Xacro_and_TF_Mental_Model_[A]__
   - __Content__:
     - __Context__: URDF is the single source of truth for frames/joints that everything else (controllers, planning, perception) depends on.
     - __Analogy Start__: “A robotic arm is a stack of rulers connected by hinges; TF is the set of instructions that tells you where each ruler is.”
     - __Explanation__: Show base frame, joint axis, link frames, and how a joint angle changes downstream frames; highlight joint naming consistency.
     - __Animation Style__: Clean 2D hinge-chain with labels; show a “TF tree” expanding as joints are added.
     - __Process__:
       1. Start with a single link at `base_link`.
       2. Add one revolute joint + link; animate rotation about axis.
       3. Add 3–6 joints; show how end-effector pose changes.
       4. Overlay: “URDF → robot_state_publisher → TF → RViz”.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (rotation + homogeneous transforms)

4. __M1_V04_Building_Toy_Arm_and_Bringing_Up_Panda_in_RViz_[S]__
   - __Content__: Create `toy_arm_description`, write the toy URDF/Xacro, launch RViz, then bring up Panda and compare joint names + frames side-by-side.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (frames/transforms) + p.10 (FK concept)

5. __M1_V05_Validation_check_urdf_TF_and_Common_Fixes_[S]__
   - __Content__: Run `check_urdf`, inspect TF tree in RViz, fix typical errors (wrong joint names, missing inertias, bad origins) on toy + Panda.
   - __Notes__: `Notes/part_1_kinematics.pdf` p.2-9 (homogeneous transforms and relative poses) + p.10-16 (frame conventions via FK/DH)

#### Part B: Real System / Hardware
6. __M1_V06_ros2_control_Hardware_Plugin_Skeleton_[S]__
   - __Content__: Generate `arm_hardware` package, implement minimal interfaces, and confirm it compiles.
   - __Notes__: (ROS2/`ros2_control` architecture; not covered in `Notes/`)

7. __M1_V07_Loading_the_Plugin_and_Debugging_Interfaces_[S]__
   - __Content__: Minimal controller manager config, verify interface names match URDF joints, troubleshoot pluginlib/load errors.
   - __Notes__: (ROS2/`pluginlib` debugging; not covered in `Notes/`)

---

## Assignments & Practice Tasks

> Each assignment must include a measurable submission (plot/log/bag/video/PR).

1. __M1_A01_Toy_Arm_URDF_and_TF_Validation__
   - __Goal__: RViz shows a clean TF tree for the toy arm.
   - __Steps__: build + launch; inspect TF; move joints with GUI.
   - __Submission__: screenshot of RViz TF tree + `check_urdf` output pasted into `docs/m1_a01.txt`.
   - __Pass Criteria__: no TF frame errors in RViz; `check_urdf` completes without fatal errors.

2. __M1_A02_Bringup_Launches_Toy_and_Panda__
   - __Goal__: one command launches RViz view for each robot (toy + Panda).
   - __Steps__: create `arm_bringup` launches; set params; document usage.
   - __Submission__: PR + 30–60s screen recording showing both launches working.
   - __Pass Criteria__: both `ros2 launch ...` commands work on a clean rebuild.

3. __M1_A03_controllers_install__
   - __Goal__: `arm_hardware` builds as a plugin.
   - __Steps__: implement skeleton; export plugin; build.
   - __Submission__: PR + build log snippet.
   - __Pass Criteria__: `colcon build` succeeds; plugin export is present.

4. __M1_A04_Create Panda Robotic arm with stls and controllers__
   - __Goal__: diagnose and fix a deliberate joint-name mismatch.
   - __Steps__: rename one joint in URDF, reproduce failure, then fix it.
   - __Submission__: short writeup: “symptom → root cause → fix”.
   - __Pass Criteria__: RViz motion + TF tree restored after fix.

---

## Final Summary

- You have a ROS2 workspace with `arm_description` + `arm_bringup`.
- Your arm model visualizes correctly in RViz with a stable TF tree and joint motion.
- You have a compiling `ros2_control` hardware plugin skeleton ready for real comms.
- Next module adds real joint control paths (position/velocity) with safety limits.
