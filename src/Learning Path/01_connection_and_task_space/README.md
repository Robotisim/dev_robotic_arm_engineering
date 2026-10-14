# Module 1 — Connection, joint angles and task space

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [Display joint angles and sample the workspace](projects/01_display_workspace/README.md)
- [Mock controller angle commands](projects/02_mock_jog/README.md)
- [Gazebo angle commands](projects/03_gazebo_jog/README.md)

Connect the teaching examples to the repository's existing motor and robot model. Finish with a visible robot, a workspace cloud and a bounded one-joint command.

## Features and concepts

Angle commands, radians versus firmware units, joint limits, base/tool frames, joint space versus XYZ task space, workspace sampling and state provenance. Existing PlatformIO firmware remains the motor layer; the course introduces no new motor protocol.

## Read the implementation

| File | Responsibility |
|---|---|
| `arm_module_01/launch/module_01.launch.py` | Select display, mock, Gazebo or existing hardware backend |
| `arm_module_01/src/joint_lab.cpp` | Display synthetic angles, inspect state, sample workspace, preview/execute one-joint goals |
| `arm_module_01/launch/lab.launch.py` | Supply canonical URDF to a single focused C++ lab |
| `../../roboflex_pio/src/main.cpp` | Existing angle-command firmware |
| `../../roboflex_ros2/roboflex_control/config/ros2_controllers.yaml` | Existing arm/gripper controller contract |

The display backend publishes synthetic `/joint_states`. Mock uses `mock_components/GenericSystem`. Gazebo uses `gz_ros2_control`. Hardware uses the repository's existing launch and command-derived state. Never run the display publisher alongside a controller backend.

## Lab 1 — Model and joints

After the root setup instructions:

```bash
ros2 launch arm_module_01 module_01.launch.py backend:=display
```

In a second terminal, change the displayed four arm angles:

```bash
ros2 topic pub --once /learning/display_target sensor_msgs/msg/JointState \
  "{name: [joint_1, joint_2, joint_3, joint_4], position: [0.1, -0.4, -0.9, -0.3]}"
```

Compare `/joint_states`, TF and the model. Send a joint_1 angle above 1.57 rad and inspect the rejection. This display exercise commands no motor.

## Lab 2 — Task space and workspace

Keep display running. Run each lab separately:

```bash
ros2 launch arm_module_01 lab.launch.py operation:=inspect
ros2 launch arm_module_01 lab.launch.py operation:=workspace
```

Add `/learning/workspace` as a MarkerArray display in RViz, with durability set to Transient Local. The C++ node samples valid joint configurations and applies FK. The point cloud is a sampled kinematic envelope, not a guarantee of collision-free reachable motion or a dexterous workspace for every orientation.

## Lab 3 — Controller-backed angle command

Stop display, then start mock:

```bash
ros2 launch arm_module_01 module_01.launch.py backend:=mock
```

Preview and execute a small joint move from another terminal:

```bash
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05
ros2 launch arm_module_01 lab.launch.py operation:=jog joint:=joint_1 angle:=0.05 execute:=true
```

Repeat with `backend:=gazebo` to observe the same geometry in a physics simulation; use `use_sim_time:=true` on the lab launch. Launch files adjust only the generated simulation description where massless/mimic gripper details need physics-compatible values. Canonical real-robot limits are unchanged. This early RoboFlex Gazebo lab disables gravity in the generated description because servo dynamics have not been identified; it teaches geometry and commanded motion, not physical load prediction. Panda later uses gravity and contact physics.

## Lab 4 — Existing real connection, optional

Use the repository's working PlatformIO and connection procedure. Review the production environment in `roboflex_pio/platformio.ini`, build it as documented there, and keep its existing ARM/HOLD procedure. Uploading/arming is an explicit hardware step, not part of a simulation launch.

If a hardware stack is already running, attach the lab in that stack's ROS domain and do not launch another driver. Otherwise `module_01.launch.py backend:=hardware firmware_ip:=ACTUAL_DEVICE_IP` wraps the existing hardware launch. Replace `ACTUAL_DEVICE_IP` with the actual configured device address.

There is no measured feedback to read from these motors. An acknowledged command is not a measured joint position. Inspecting or saving angles reports the commanded-state estimate. The software does not require adding encoders to complete this path.

## Completion evidence

Capture a joint-angle screenshot, identify the base and tool frame, explain why sampled XYZ points differ from joint limits, and demonstrate rejection of an invalid angle. Explain the state source for each backend before moving to Module 2.
