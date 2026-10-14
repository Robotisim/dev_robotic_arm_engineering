# Module 2 — Robot-specific kinematics and XYZ motion

## Runnable projects

Start with these standalone guides; the sections below explain concepts and additional exercises.

- [DH, FK and XYZ motion without MoveIt](projects/01_fk_dh_xyz/README.md)

Compute FK and bounded position IK with KDL/Eigen, then send timed trajectories through the existing controllers. MoveIt is not used in this module.

## Features and concepts

URDF transforms, standard DH, base/tool offsets, homogeneous transforms, Jacobian, damped least squares, redundant position solutions, singularity behavior, limits, residuals and quintic interpolation. The existing transport interface is reused for hardware; the exercise focuses on the solver/controller boundary.

## Read in sequence

1. `../shared/arm_course_core/include/arm_course/kinematics.hpp` and `src/kinematics.cpp` — URDF/KDL chain, FK, numerical position IK and derived DH.
2. `arm_module_02/src/kinematics_node.cpp` — `/learning/fk_pose` and the `MoveXYZ` action server.
3. `arm_module_02/src/xyz_client.cpp` — goal submission, result and cancellation timeout.
4. `arm_module_02/launch/module_02.launch.py` — model and backend composition.
5. `../shared/arm_course_core/test/core_checks.cpp` — meaningful geometry and bound checks.

Use [the shared core](../shared/arm_course_core/README.md) as the clickable entry point.

## Lab 1 — Derive this robot's DH table

```bash
ros2 launch arm_module_02 module_02.launch.py start_backend:=false dh_report:=true
```

Record the table **and both base/tool matrices** printed by the program. See [the checked numeric report](../resources/roboflex_dh.txt). The convention is:

`T_base_tool = T_base_DH × Π[Rz(q_i + theta_offset_i) Tz(d_i) Tx(a_i) Rx(alpha_i)] × T_DH_tool`.

The exported CAD model contains almost parallel axes rounded to limited precision. Exact common normals can produce enormous, ill-conditioned DH offsets. The teaching table groups those nearly parallel axes with a small threshold; the checked maximum matrix discrepancy was approximately 1.18e-6 over 100 samples. Runtime FK/Jacobian/IK use the original KDL chain, not the approximate DH table. Regenerate the report if the URDF changes.

## Lab 2 — FK and current state

```bash
ros2 launch arm_module_02 module_02.launch.py backend:=mock rviz:=false
```

Second terminal:

```bash
ros2 topic echo /learning/fk_pose --once
```

The initial Arm Up tool position is approximately `(0.04330, 0.00985, 0.31008)` metres in `base_link`, for the model revision used in validation. Read the live result rather than treating this number as a physical measurement.

## Lab 3 — XYZ goal without MoveIt

```bash
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]'
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[0.048,0.01,0.308]' -p execute:=true
```

The server uses fresh joint state as a seed, solves XYZ, checks every joint against the canonical model and generates a timed controller trajectory. A four-joint robot cannot satisfy an arbitrary XYZ plus roll/pitch/yaw request; orientation is a consequence of this position-only solution.

Try an unreachable target:

```bash
ros2 run arm_module_02 xyz_client --ros-args -p xyz:='[3.0,0.0,2.0]'
```

The action must fail without dispatching a trajectory. A local numerical solver may fail on some reachable positions from a poor seed; compare seeds rather than assuming all failures prove geometric unreachability.

## Lab 4 — Existing hardware interface

Use `backend:=hardware start_backend:=false` to attach the same kinematics node to an already running real controller stack, in its existing ROS domain. No firmware IP is needed when attaching. Do not run the Module 1 display publisher. Goals remain previews until `execute:=true` is sent.

`roboflex_control` supplies the angle-command transport and mirrored state. Inspect its `read()` and `write()` responsibilities and trace action → controller → position command → firmware. Do not replace that working path. The solver respects joint bounds but **does not collision-check motion**; move to Module 3 for obstacle-aware planning.

## Completion evidence

Run the core checks, show FK/DH agreement with the stated approximation, execute the small mock XYZ goal, and show an unreachable goal rejection. Explain why a bounded IK endpoint does not guarantee a collision-free path.
