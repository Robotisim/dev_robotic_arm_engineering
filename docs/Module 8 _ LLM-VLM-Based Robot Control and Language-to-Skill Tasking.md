

# Module 8 Scope

## Module 8 title

**LLM/VLM-Based Robot Control and Language-to-Skill Tasking**


## Core module description

Module 8 teaches students how to use language and vision-language models as a high-level tasking layer for a robotic arm.

The key idea is:

```text
language/text instruction
→ object/task intent
→ VLM detects object in image
→ depth converts detection to 3D pose
→ robot skill consumes pose
→ safe pick/place executes
```


## Main story of Module 8

In Module 6, the robot detected objects using classical RGB-D perception.



Now the question becomes:

> Can we give the robot a high-level instruction like “put the red object in the red basket,” and have the system ground that instruction into perception, pose estimation, and safe robot skills?

The main pipeline is:

```text
user instruction
→ language/task parsing
→ object selection
→ VLM detection
→ RGB-D pose estimation
→ safety validation
→ skill execution
→ logging/evaluation
```

## Module 8 in scope

Module 8 should cover:

* What LLMs, VLMs, and VLAs are
* Why this module is not RL/policy training
* Language-to-skill architecture
* Skill registry concept
* Safety schema for language commands
* VLA-style simulation world
* Object semantics:

  * red sphere
  * blue cylinder
  * green brick
  * orange puck
  * colored baskets
* Classical segmentation baseline
* VLM-based detection
* PaliGemma-style location tokens
* RGB-D snapshot collection
* Depth-based 3D pose estimation
* Camera intrinsics
* Detection bounding box to depth lookup
* Object pose publishing
* `/vlm_pose_pipeline/detections`
* `/pick_place/object_pose`
* Pick/place executor consuming VLM pose
* Prompt parameterization
* Ambiguity rejection
* Human confirmation mode
* Comparison between classical segmentation and VLM detection
* Final language-to-action demo

## Module 8 out of scope

Do not make these the core of Module 8:

* RL training
* Policy learning
* Fine-tuning PaliGemma
* Training a VLA model
* Raw joint control from language
* Raw torque control from language
* Free-form unsafe autonomy
* Multi-step open-ended household planning
* Advanced foundation model research
* Deep grasp synthesis
* Visual servoing
* Production-grade cloud deployment

These may be future advanced topics.

## Module 8 teaching command flow

Use this as the core student-facing flow.

### 1. Bring up VLA world

```bash
ros2 launch robotic_arm_bringup vla_world.launch.py
```

This should bring up the VLA-style sort/place world with Panda, MoveIt, cameras, objects, baskets, and the external RGB-D camera.

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

```bash
cd /home/luqman/repos/dev_robotic_arm_engineering/src/robotic_arm_manipulation/scripts/vlm_pose_pipeline
python3.12 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
python vlm_ros2.py
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
→ pick_place_executor consumes pose
```

## Module 8 required deliverables

By the end of Module 8, students should have:

* Updated Module 8 documentation with the correct title
* VLA world launch working
* Classical segmentation baseline running
* VLM pose pipeline running
* VLM inference trigger working
* Detection output visible
* Depth-to-3D pose conversion working
* `/pick_place/object_pose` published
* Pick/place executor consuming VLM pose
* A simple language-to-skill router
* A skill registry
* A safety schema
* Ambiguity rejection behavior
* Human confirmation mode
* Comparison report:

  * classical segmentation vs VLM
* Final demo:

  * “put the red object in the red basket”
  * “put the blue object in the blue basket”
  * ambiguous command rejected safely

## Module 8 success criteria

Students should be able to:

* Explain why Module 8 is language-to-skill, not robot learning.
* Explain the difference between LLM, VLM, and VLA.
* Launch the VLA world.
* Run the classical segmentation baseline.
* Run VLM detection.
* Trigger VLM inference through ROS 2 service.
* Explain how location tokens become a bounding box.
* Explain how depth converts a 2D detection into a 3D pose.
* Publish object pose for pick/place.
* Connect language intent to a safe skill.
* Reject ambiguous or unsafe language commands.

---

# Module 8 Required Cleanup Before Recording

These are important and should be treated as implementation tasks before Youssef records the final version.

## Documentation cleanup

Current docs should be renamed from:

```text
Robot Learning and Language-to-Action Tasking
```

To:

```text
LLM/VLM-Based Robot Control and Language-to-Skill Tasking
```

Remove or move RL/policy training sections to a future optional module.

## Code/package cleanup

Fix these:

```text
VLM scripts are not installed as ros2 run executables.
requirements.txt appears malformed.
vlm_ros2.py uses a hardcoded prompt.
README does not fully match current ROS 2 node behavior.
paligemma_pose_from_image.py is described like CLI, but behaves as ROS 2 node.
```

## Missing features to add

Add these for a strong Module 8:

```text
language-to-skill router
skill registry
strict command schema
ambiguity rejection
human confirmation mode
prompt parameter support
task/object/basket mapping
safety validation before execution
```

## Recommended skill registry

Minimum skills:

```text
home
retreat
detect_object
estimate_object_pose
pick_object
place_object
pick_and_place_object
sort_object_to_basket
open_gripper
close_gripper
```

## Recommended command schema

Example safe command object:

```json
{
  "intent": "sort_object",
  "object": {
    "color": "red",
    "shape": "sphere"
  },
  "destination": {
    "type": "basket",
    "color": "red"
  },
  "constraints": {
    "confirm_before_execution": true,
    "max_retries": 1,
    "safe_speed": true
  }
}
```

## Ambiguity examples

Reject or ask confirmation for:

```text
"pick the object"
"move that thing"
"put it there"
"grab everything"
"move fast"
"ignore the obstacle"
"pick the red one" when multiple red objects exist
```

Accepted examples:

```text
"put the red sphere in the red basket"
"place the blue cylinder in the blue basket"
"move the green brick to the green basket"
"go home"
"retreat from the current object"
```

---
