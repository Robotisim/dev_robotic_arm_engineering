# Module 7 — Demonstrations and native C++ VLA foundations

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [Record two successful expert demonstrations](projects/01_record_demonstrations/README.md)
- [Train and evaluate the C++ policy](projects/02_native_training/README.md)
- [Preview and dispatch a bounded learned policy](projects/03_policy_rollout/README.md)

Record Panda observations/actions, train a small conditioned neural policy in C++, evaluate held-out episodes and run bounded predictions in simulation. This makes the entire learning loop visible before studying a large pretrained VLA.

## What is implemented

- `episode_recorder.cpp`: simulator RGB PNGs, joint state, language condition, destination, task stage and future controller reference saved as JSONL.
- `include/arm_module_07/policy.hpp`: Eigen network, normalization, backpropagation, minibatch Adam, serialization and inference.
- `policy_train.cpp`: independent-episode split, training, held-out loss and checkpoint metadata.
- `policy_run.cpp`: fresh simulated observation checks, native inference latency, bounded joint increments, MoveIt collision queries and controller dispatch.
- `launch/module_07.launch.py`: `record`, `train`, `run` modes.

Architecture: 8×8 RGB (192 values) + seven arm angles + one finger position + three-way colour condition + XYZ destination = **206 inputs**; 64 tanh hidden units; eight continuous joint targets. Finger position is half the total jaw opening. Action horizon is 0.2 seconds. Instructions are a small structured vocabulary; there is no tokenizer or pretrained semantic language encoder.

This is an educational vision-language-action/behavior-cloning baseline, not SmolVLA, OpenVLA or an equivalent foundation model. It teaches the same observation/action/data/evaluation concerns without introducing Python nodes or a hidden training service. Performance on unseen scenes must be measured, not assumed.

## Lab 1 — Collect an episode

Keep the Module 4 Panda stack running. Start a recorder in a second terminal:

```bash
ros2 launch arm_module_07 module_07.launch.py mode:=record \
  episode:=/tmp/arm_course_dataset/episode_001 colour:=red
```

In a third terminal, run the expert task with the **same destination as the recorder**:

```bash
ros2 run arm_module_04 perception_pick --ros-args -p use_sim_time:=true \
  -p colour:=red -p destination:='[0.45,0.28,0.025]' -p execute:=true
ros2 service call /learning/finish_episode std_srvs/srv/Trigger '{}'
```

Stop the recorder after finishing. It rejects an existing episode directory to avoid silently overwriting demonstrations. Recording requires simulated time and fresh RGB, state and both controller-reference streams. It records non-idle task stages only and pairs an observation with the controller reference about 0.2 seconds later. Final samples without a future label are dropped.

The standalone recording guide checks advancing `/clock` before collecting. The recorder rejects samples whose sensor/controller timestamps are ahead of its clock and reports a clock-alignment warning. `finish_episode` must return success and a nonzero sample count; PNG files alone do not mean that training labels were recorded.

For a custom destination, run the recorder executable directly with `-p use_sim_time:=true -p episode:=... -p colour:=... -p destination:='[x,y,z]'`. Keep labels consistent with the demonstrated task. Failed expert attempts should be inspected and moved out of the training dataset if the objective is successful behavior cloning; task success is not inferred from the existence of a JSONL file.

## Lab 2 — Collect varied, independent episodes

Reset the scene while idle, return the arm to a repeatable observation pose and record `episode_002` and further episodes using new paths. Vary cube positions, colours and left/right destinations while keeping correct labels. At least two nonempty episodes and 32 training samples are required; two episodes only exercise the pipeline and do not establish generalization.

The dataset contains one PNG per observation and an `episode.jsonl` with schema, robot, state source, observation/action times, colour, destination, image path, features and target action. Inspect timing gaps, missing frames and action magnitudes before training.

## Lab 3 — Native training and held-out loss

```bash
ros2 run arm_module_07 policy_train --ros-args \
  -p dataset:=/tmp/arm_course_dataset -p epochs:=100 \
  -p output:=/tmp/course_policy.json
```

Whole episodes are split 80/20 in sorted path order; the last episode is held out even with only two. Normalization uses training samples. Checkpoint metadata records episode paths, sample counts, epochs and normalized training/test MSE. Rename/shuffle episode ordering deliberately when studying distribution shifts; adjacent frames from one episode must not be split across train and test.

The native learning self-test is separate:

```bash
ros2 run arm_module_07 policy_train --ros-args -p self_test:=true \
  -p output:=/tmp/course_policy_selftest.json
```

It checks learning on synthetic data and checkpoint round-trip. `policy_run` refuses that synthetic checkpoint for robot control.

## Lab 4 — Preview inference

```bash
ros2 launch arm_module_07 module_07.launch.py mode:=run \
  checkpoint:=/tmp/course_policy.json colour:=red execute:=false
```

The node uses the simulator image/state and logs inference time. Preview does not command joints. Train on your intended condition and reset to a comparable initial state before assessing predictions. Low offline MSE is not proof of successful picking.

## Lab 5 — Bounded simulated rollout

```bash
ros2 launch arm_module_07 module_07.launch.py mode:=run \
  checkpoint:=/tmp/course_policy.json colour:=red execute:=true steps:=5
```

The policy claims task ownership so MoveIt recipes do not dispatch at the same time. It rejects invalid joint targets, limits each arm increment to 0.015 rad and finger increment to 0.003 m, checks three intermediate configurations for collision, then sends 0.2-second arm/hand trajectories. These sampled checks are an educational execution guard, not continuous collision certification.

Completed runs write `/tmp/policy_rollout.csv` with inference latency, maximum observed lift, destination errors and manipulation success. Failure logs must be counted as rejected rollouts, not omitted from the denominator.

The runner does not automatically enable target-object contact for an untrusted learned grasp; collision rejection near contact can therefore stop a rollout. Study predicted actions and recorded failure reasons before adding a task-aware learned-contact controller. Full pick completion from this small model is an empirical learning objective, not guaranteed by the included implementation.

## Evaluation project

Keep a set of scene seeds/episodes out of training. Compare expert versus policy on target selection, physical lift, destination error, collisions/rejections, completion rate and inference latency. Evaluate complete trajectories in addition to action MSE. Use evaluator ground truth only for metrics, never as an unrecorded policy input.

Then read the official SmolVLA/OpenVLA material in [resources](../resources/README.md): compare observation layout, action chunking, normalization, language encoding, robot embodiment and fine-tuning requirements. Running those pretrained architectures is a separate integration; this course does not relabel the native MLP as pretrained VLA inference.

## Included reproducible sample

Two successful camera-guided red-cube episodes are included as [sample_demonstrations.tar.gz](../resources/sample_demonstrations.tar.gz), with 145 and 146 samples. [sample_policy.json](../resources/sample_policy.json) is the native C++ checkpoint trained on the first and evaluated on the second. These demonstrate the learning pipeline; the narrow two-episode dataset does not establish a robust manipulation policy.

```bash
mkdir -p "$COURSE_ROOT/Learning Path/artifacts/sample_demonstrations"
tar -xzf "$COURSE_ROOT/Learning Path/resources/sample_demonstrations.tar.gz" \
  -C "$COURSE_ROOT/Learning Path/artifacts/sample_demonstrations"
ros2 run arm_module_07 policy_train --ros-args \
  -p dataset:="$COURSE_ROOT/Learning Path/artifacts/sample_demonstrations" \
  -p output:='/tmp/retrained_sample_policy.json' -p epochs:=100
```

The sample training run produced normalized MSE 0.0105605 on training and 0.048335 on the held-out episode. Both episodes use red/left; collect other colours and destinations before evaluating those conditions. A separate randomized grasp failed physical lift and was excluded from this sample dataset.
