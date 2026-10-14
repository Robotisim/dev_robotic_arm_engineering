# Module 6 — Language and vision grounded in depth observations

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [Ground a language command using rules](projects/01_rules/README.md)
- [Use a small local LLM for task intent](projects/02_local_llm/README.md)
- [Use the simulator image with a local VLM](projects/03_image_vlm/README.md)

Translate a short instruction into a validated task, resolve it against detected cubes and execute the existing C++ pick pipeline. A local model interprets intent; the motion server remains responsible for robot motion.

## Features and topics

Structured output, local inference, colour/object references, ambiguity, schema validation, stale observations, deterministic task dispatch and the difference between LLM/VLM planning and learned robot actions.

## Implementation

`arm_module_06/src/language_task.cpp` contains three interpreters:

| Interpreter | Input | Output |
|---|---|---|
| `rules` | Short affirmative pick/grab/move instruction | Deterministic colour + left/right destination |
| `llm` | Instruction + constrained schema prompt | Local Ollama JSON response |
| `vlm` | Same prompt + current RGB image | Local vision-capable Ollama JSON response |

All ROS/client code and HTTP/image serialization are C++. Ollama is an external native inference service, not a Python ROS node. Exact JSON fields are `action`, `colour`, `destination`; accepted values are pick, red/green/blue, left/right. The model supplies neither executable code nor robot coordinates. Unknown fields, unsupported values, missing or ambiguous objects and stale observations are rejected.

## Lab 1 — Rules baseline

Keep Module 4/5 running, or start:

```bash
ros2 launch arm_module_06 module_06.launch.py headless:=true rviz:=false
```

Run a preview:

```bash
ros2 run arm_module_06 language_task --ros-args -p use_sim_time:=true \
  -p instruction:='pick the red cube and place it on the left'
```

Add `-p execute:=true` for the simulated transfer. Test `pick the purple cube`, a negated command and a request with two colours; inspect rejection rather than accepting a guessed object.

## Lab 2 — Small local language model

With Ollama installed and serving on localhost:

```bash
ollama pull qwen2.5:0.5b
ros2 run arm_module_06 language_task --ros-args \
  -p interpreter:=llm -p model:=qwen2.5:0.5b -p interpret_only:=true \
  -p instruction:='pick the red cube and place it on the left'
```

`interpret_only` validates language output without requiring a simulator. Compare paraphrases against the rules baseline. For depth-grounded execution:

```bash
ros2 run arm_module_06 language_task --ros-args \
  -p use_sim_time:=true -p interpreter:=llm -p model:=qwen2.5:0.5b \
  -p instruction:='move the blue cube to the right' -p execute:=true
```

The endpoint defaults to `http://127.0.0.1:11434/api/generate`. A small LLM can misinterpret language despite returning valid JSON; record the interpretation and task outcome independently.

## Lab 3 — Optional image-conditioned interpretation

Use a **vision-capable model installed in your local Ollama server** and set `interpreter:=vlm -p model:=YOUR_INSTALLED_VISION_MODEL`. Unlike the 0.5B text model, it must support image input. The client encodes the latest simulator RGB image and sends it with the instruction. Depth still supplies the 3D target. No vision-model weights are bundled, and the text-only checkpoint must not be presented as a VLM.

Test the image path with your chosen model and include its exact model/version in the lab report. The image path was also exercised with the locally installed `gemma4:12b`, which advertised vision support. It returned a valid green/right instruction. It is larger than the text baseline; availability and behavior of another optional model depend on the local server.

## Completion evidence

Compare rules and LLM on clear, ambiguous and unsupported commands; show the validated JSON, resolved depth target and resulting simulation task. Explain why this is language-guided MoveIt, while Module 7 learns joint actions from demonstrations.
