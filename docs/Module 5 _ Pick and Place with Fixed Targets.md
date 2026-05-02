
# Module 5 Scope

## Module 5 title

**Pick and Place with Fixed Targets**

## Instructor Flow Design Answers

**Questions Based Instructor Flow Design?**

* Start with the question: how does a robot pick a known object when all target poses are fixed?
* Teach in the rhythm `fixed target -> pre-grasp -> grasp -> lift -> place -> retreat`.
* Keep the lesson structured around manipulation sequencing, not perception.

**What is this module about?**

* This module is about turning MoveIt planning into a repeatable pick/place procedure.
* Students learn fixed object poses, pre-grasp poses, approach vectors, retreat vectors, gripper timing, and cycle testing.
* The target is known ahead of time, so perception should remain a preview for Module 6.

**What final project outcome should students achieve?**

* Students should launch a Panda pick/place world with fixed cubes or objects.
* Students should run a structured pick/place executor through approach, grasp, lift, place, and retreat.
* Students should repeat the cycle and record whether the routine succeeds consistently.

**What concepts must the instructor explain?**

* Explain object frame, place frame, tool frame, pre-grasp, grasp, lift clearance, and table clearance.
* Explain MoveIt planning for each phase and why direct-to-grasp motion is risky.
* Explain simulated gripper open/close, attach/detach behavior, parameter tuning, and failure logging.

**What exact code/package/files should be shown?**

* Show `robotic_arm_bringup/launch/pick_place_world.launch.py`.
* Show `robotic_arm_manipulation/config/params.yaml` and `robotic_arm_manipulation/src/pick_place_executor.cpp`.
* Show `robotic_arm_sim/worlds/pick_and_place_cubes_base.sdf` and related Panda pick/place launch files.

**What commands should be run?**

* Run `ros2 launch robotic_arm_bringup pick_place_world.launch.py world:=cubes`.
* Run `ros2 run robotic_arm_manipulation pick_place_executor_node`.
* Run repeated cycles with the same fixed targets and record success, timing, and failure reason.

**What visual/demo result should appear?**

* Gazebo should show Panda, a table scene, and known target objects.
* RViz or MoveIt should show planned phase motions for approach, pick, lift, place, and retreat.
* The object should appear to move from the source target to the place target during the routine.

**What mistakes should the instructor avoid?**

* Do not introduce RGB-D perception or runtime object detection as the core Module 5 workflow.
* Do not skip pre-grasp and retreat, because they are the main safety structure of manipulation.
* Do not call a one-off success reliable without repeated cycle testing.

**What should the student understand by the end of each lesson?**

* Students should know which phase of pick/place the lesson just demonstrated.
* Students should connect each phase to a target pose, parameter, planner call, or gripper action.
* Students should be able to rerun the demo and debug wrong frames, missed grasps, or poor clearances.

**Package Names And Responsibilities?**

* `robotic_arm_manipulation` owns pick/place executors, parameters, gripper behavior, and task sequencing.
* `robotic_arm_bringup` owns the public pick/place world launch.
* `robotic_arm_sim` owns the table, cubes, and fixed-target world assets.

---

## Module 5 in scope

Module 5 should cover:

* What pick and place means in robotics
* Task decomposition
* Fixed target poses
* Object frame
* Place frame
* Tool frame
* Pre-grasp frame
* Grasp frame
* Approach vector
* Retreat vector
* Lift clearance
* Table clearance
* Orientation constraints
* MoveIt planning for each phase
* Gripper open/close
* Simulated attach/detach behavior
* Parameter tuning
* YAML-based target storage
* Cycle testing
* Failure logging
* Debugging grasp misses and slips

## Module 5 out of scope

Do not make these the focus of Module 5:

* RGB-D camera perception
* color segmentation
* runtime object detection
* camera calibration
* point clouds
* Octomap mapping
* advanced grasp synthesis
* deep learning-based grasping
* visual servoing

These belong in Module 6 or later.

## Module 5 required deliverables

By the end of Module 5, students should have:

* A `robotic_arm_manipulation` pick/place routine
* Fixed object/place/pre-grasp targets
* A clean fixed target YAML file
* A working Panda pick/place world
* A single-object pick/place executor
* Gripper open/close or attach/detach behavior
* A 3-cycle successful demo
* A 10-cycle reliability report
* A CSV with timing and success/failure
* A short debugging writeup

Your plan asks for a `robotic_arm_manipulation` package/script that runs approach → grasp/attach → lift → place → retreat, plus fixed targets in YAML and a demo showing 3 consecutive successful cycles.

## Module 5 cleanup before recording

Before recording Module 5, fix or clearly mark these issues:

* The docs ask for fixed targets in YAML, but current target values are inside `params.yaml`; create a cleaner `fixed_targets.yaml`.
* Add benchmark/log CSV for 3 or 10 repeated cycles.
* Formalize simulated attach/detach object handling.
* Resolve `static_env_sdf_path` from the `robotic_arm_sim` package share instead of a hardcoded source-style path.

These cleanup items are already noted in your teaching map.

## Module 5 success criteria

Students should be able to:

* Define fixed target poses in a known frame.
* Explain pre-grasp, grasp, lift, place, and retreat.
* Launch the pick/place world.
* Run a pick/place routine.
* Tune approach height and clearances.
* Explain why direct-to-grasp motion is risky.
* Debug wrong target frames.
* Debug missed grasps.
* Run repeated cycles and calculate success rate.

## Module 5 final scope decision

Keep Module 5 focused on:

```text
fixed targets → structured manipulation sequence → MoveIt plans → gripper timing → cycle validation
```

Do not introduce perception yet. Module 6 will replace fixed targets with detected targets.

---
