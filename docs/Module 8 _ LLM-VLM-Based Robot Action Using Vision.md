# Module 8 Scope

## Module 8 title

**LLM/VLM-Based Robot Action Using Vision**

## Instructor Flow Design Answers

**Questions Based Instructor Flow Design?**

* Start with the question: how can language or image input select a safe robot skill?
* Teach in the rhythm `language/image input -> object grounding -> validated skill -> robot procedure`.
* Keep the model responsible for target selection, not raw joint, torque, or open-ended robot control.

**What is this module about?**

* This module is about using LLM/VLM-style perception to choose visual pick/place targets.
* Students learn prompts, visual grounding, bounding boxes, depth-to-3D conversion, pose publishing, and skill validation.
* The robot still executes a bounded ROS skill rather than arbitrary model-generated motion.

**What final project outcome should students achieve?**

* Students should launch the VLA-style world and run the classical segmentation baseline.
* Students should run the VLM pose pipeline, trigger inference, and publish `/pick_place/object_pose`.
* Students should connect a high-level instruction like "pick the red sphere" to a validated pick/place procedure.

**What concepts must the instructor explain?**

* Explain LLM, VLM, and VLA roles at a high level without turning the lesson into model training.
* Explain prompts, visual target grounding, PaliGemma-style location tokens, bounding boxes, and depth lookup.
* Explain why robot skills must validate frames, reachability, object identity, and allowed actions before execution.

**What exact code/package/files should be shown?**

* Show `robotic_arm_bringup/launch/vla_world.launch.py` and `robotic_arm_sim/worlds/vla_pipeline_env.sdf`.
* Show `robotic_arm_vision/launch/segmentation.launch.py` as the classical baseline.
* Show `robotic_arm_manipulation/scripts/vlm_pose_pipeline/vlm_ros2.py` and `paligemma_pose_from_image.py`.

**What commands should be run?**

* Run `ros2 launch robotic_arm_bringup vla_world.launch.py`.
* Run `ros2 launch robotic_arm_vision segmentation.launch.py` for the baseline.
* Run the VLM pipeline scripts, then `ros2 service call /vlm_pose_pipeline/trigger std_srvs/srv/Trigger {}`.

**What visual/demo result should appear?**

* Gazebo should show the VLA-style object sorting scene with colored objects and baskets.
* The baseline should show classical segmentation output for comparison.
* The VLM pipeline should publish detections and a target pose that the pick/place executor can consume.

**What mistakes should the instructor avoid?**

* Do not claim the model controls joints directly.
* Do not make training, fine-tuning, or foundation-model research the core deliverable.
* Do not skip safety validation between model output and robot procedure.

**What should the student understand by the end of each lesson?**

* Students should know which part is language/image reasoning and which part is deterministic robot execution.
* Students should connect a prompt to a detection, depth point, object pose, and skill call.
* Students should be able to rerun the demo and debug hardcoded prompts, missing dependencies, or wrong target frames.

**Package Names And Responsibilities?**

* `robotic_arm_bringup` owns the VLA world launch.
* `robotic_arm_vision` owns the classical segmentation baseline used for comparison.
* `robotic_arm_manipulation` owns VLM scripts, pose publication, and pick/place skill execution.

## Core module description

Module 8 teaches students how a language or vision-language model can help a robot choose and execute a visual pick-and-place action.

The focus is not training a learned policy. The focus is:

```text
text instruction
→ visual target request
→ VLM detects the target in an RGB image
→ depth converts the detection to a 3D pose
→ ROS publishes the target pose
→ the pick/place executor uses that pose
```

## Main story of Module 8

In Module 6, the robot detected objects using classical RGB-D perception.

Now the question becomes:

> Can we give the robot a high-level instruction like “pick the red sphere,” use vision to locate that object, and send the resulting pose into the robot pick/place procedure?

The main pipeline is:

```text
user instruction
→ object selection prompt
→ VLM detection
→ bounding box
→ depth lookup
→ 3D pose in camera frame
→ object pose topic
→ visual pick/place execution
```

## Module 8 in scope

Module 8 should cover:

* What LLMs, VLMs, and VLAs are at a high level
* Why this module uses models for visual target selection, not low-level robot control
* VLA-style simulation world
* Object semantics:

  * red sphere
  * blue cylinder
  * green brick
  * orange puck
  * colored baskets
* Classical segmentation baseline
* VLM-based object detection
* PaliGemma-style location tokens
* RGB-D snapshot collection
* Depth-based 3D pose estimation
* Camera intrinsics
* Detection bounding box to depth lookup
* Object pose publishing
* `/vlm_pose_pipeline/detections`
* `/pick_place/object_pose`
* Pick/place executor consuming the visual target pose
* Prompt parameterization
* Visual pick-and-place demo
* Comparison between classical segmentation and VLM detection

## Module 8 out of scope

Do not make these the core of Module 8:

* training policies
* fine-tuning PaliGemma
* training a VLA model
* raw joint control from text
* raw torque control from text
* open-ended household planning
* advanced foundation-model research
* deep grasp synthesis
* visual servoing
* production cloud deployment

These may be future advanced topics.

## Module 8 teaching command flow

Use this as the core student-facing flow.

### 1. Bring up VLA world

```bash
ros2 launch robotic_arm_bringup vla_world.launch.py
```

This brings up the VLA-style sort/place world with Panda, MoveIt, cameras, objects, baskets, and the external RGB-D camera.

### 2. Run classical segmentation baseline

```bash
ros2 launch robotic_arm_vision segmentation.launch.py
```

Use this as the comparison baseline:

```text
classical color/depth segmentation
vs
VLM-based object detection
```

### 3. Run VLM scripts from source for now

Terminal A:

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering/src/robotic_arm_manipulation/scripts/vlm_pose_pipeline
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python vlm_ros2.py
```

Terminal B:

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering/src/robotic_arm_manipulation/scripts/vlm_pose_pipeline
source .venv/bin/activate
python paligemma_pose_from_image.py
```

### 4. Trigger VLM inference

```bash
ros2 service call /vlm_pose_pipeline/trigger std_srvs/srv/Trigger {}
```

Expected flow:

```text
VLM detection
→ /vlm_pose_pipeline/detections
→ location tokens + depth
→ /pick_place/object_pose
→ pick/place executor consumes pose
```

## Module 8 required deliverables

By the end of Module 8, students should have:

* VLA world launch working
* Classical segmentation baseline running
* VLM pose pipeline running
* VLM inference trigger working
* Detection output visible
* Depth-to-3D pose conversion working
* `/pick_place/object_pose` published
* Pick/place executor consuming the visual target pose
* Prompt can be changed for different visual targets
* Comparison notes:

  * classical segmentation vs VLM detection
* Final demo:

  * “pick the red sphere”
  * “pick the blue cylinder”
  * “pick the green brick”

## Module 8 success criteria

Students should be able to:

* Explain why the model is used for visual target selection, not low-level robot control.
* Explain the difference between LLM, VLM, and VLA.
* Launch the VLA world.
* Run the classical segmentation baseline.
* Run VLM detection.
* Trigger VLM inference through a ROS 2 service.
* Explain how location tokens become a bounding box.
* Explain how depth converts a 2D detection into a 3D point.
* Publish object pose for pick/place.
* Connect a visual model output to the robot pick/place executor.

---

# Module 8 Cleanup Before Recording

These are implementation cleanup tasks before final recording:

```text
VLM scripts are not installed as ros2 run executables.
requirements.txt appears malformed.
vlm_ros2.py uses a hardcoded prompt.
README does not fully match current ROS 2 node behavior.
paligemma_pose_from_image.py is described like CLI, but behaves as a ROS 2 node.
```

Recommended fixes:

```text
install VLM scripts as package executables
make the VLM prompt configurable
update the VLM README for ROS 2 node usage
show terminal A / terminal B startup clearly
add a small demo instruction file with example prompts
```
