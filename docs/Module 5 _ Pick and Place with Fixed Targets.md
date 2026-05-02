
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