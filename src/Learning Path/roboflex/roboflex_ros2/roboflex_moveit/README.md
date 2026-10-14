# roboflex_moveit

MoveIt 2 configuration package for RoboFlex.

## Purpose

- Owns MoveIt semantic/planning configuration (SRDF, kinematics, joint limits).
- Owns MoveIt launches (`move_group`, MoveIt RViz, setup assistant, demo).
- Owns MoveIt controller mapping (`moveit_controllers.yaml`).

## Canonical Launches

```bash
ros2 launch roboflex_moveit move_group.launch.py
ros2 launch roboflex_moveit moveit_rviz.launch.py
```

Real hardware (no simulation clock):

```bash
ros2 launch roboflex_moveit move_group.launch.py use_sim_time:=false
ros2 launch roboflex_moveit moveit_rviz.launch.py use_sim_time:=false
```

Combined hardware stack (control + MoveIt + RViz) is provided by bringup:

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py firmware_ip:=<ESP32-IP>
```

## Small goals above the table

The `moveit_goal.py` script uses MoveIt IK, planning, collision checking, and
`ExecuteTrajectory`. It previews by default; `--execute` enables execution.
Use the actual ESP32 address printed over USB, not the broadcast default.

In each ROS terminal:

```bash
cd /home/luqman/repos/RoboFlexv2.1
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

Start the hardware launch in one terminal, and the USB status bridge in another:

```bash
ros2 launch roboflex_bringup hardware_gui.launch.py firmware_ip:=192.168.1.49
```

```bash
ros2 run roboflex_moveit firmware_monitor.py
```

The address above was observed on 14 October 2026 and can change. Close miniterm
with **Ctrl+]** before starting the bridge; only one process can own USB serial.
The bridge opens with DTR/RTS inactive and does not arm automatically. Opening
serial can still reset some boards. Establish firmware readiness with motor
power off, then restore power only with the arm supported at its known reference.

After the controllers are active, the physical arm matches **Arm Up**, and motor
power is on, explicitly arm from a third terminal:

```bash
ros2 service call /roboflex/arm std_srvs/srv/Trigger '{}'
```

Check `/roboflex/firmware_status` for `armed: 1`, `hold: 0`, `driver_fault: 0`,
connected Wi-Fi, and increasing `rx_count`. ROS joint states are command estimates;
do not arm if RViz has executed a different goal while the ESP32 was disarmed.
Reconcile the host and physical reference first. The goal script rejects a
disagreement between host and firmware command positions before execution.

Preview a 5-degree base turn while preserving the raised arm configuration:

```bash
ros2 run roboflex_moveit moveit_goal.py --base-offset-deg 5 --table-z 0
```

Execute it once:

```bash
ros2 run roboflex_moveit moveit_goal.py --base-offset-deg 5 --table-z 0 --execute
```

Offsets are relative to the current command state. Repeating the command makes
another turn. These alternatives preview a 5-mm upward tool goal or return to
Arm Up after the base-only turn:

```bash
ros2 run roboflex_moveit moveit_goal.py --offset 0 0 0.005 --table-z 0
ros2 run roboflex_moveit moveit_goal.py --named-pose 'Arm Up' --table-z 0
```

`--table-z` is the tabletop height in the model's `base_link` frame, in metres.
Use `0` only when the table is at or below the base mounting surface. The script
adds a 2-by-2-metre collision box whose top is 20 mm above that height; only the
fixed base may touch it. All moving links remain collision checked. The object
also appears in RViz and remains until MoveIt restarts or it is explicitly
removed. The base frame must be upright relative to the tabletop.

Additional script guards limit each joint change to 10 degrees, Cartesian
offsets to 20 mm, and prohibit downward Cartesian goals. A path constraint keeps
the tool within 1 mm of or above its starting height, with at least 80 mm above
the specified table. The gripper must remain at its confirmed closed reference.
A named-pose return that would lower the tool beyond this allowance is refused.

Plans use 25% of the 0.2-rad/s velocity and 0.4-rad/s² acceleration caps, with
additional timing headroom. Before execution the script checks the controller's
quintic interpolation at intervals no greater than 40 ms for joint bounds,
speed, acceleration, collisions, and path constraints. This is sampled model
validation, not a continuous physical collision guarantee. Calibration errors,
unmodelled obstacles, and servo tracking error remain possible.

During execution the script monitors USB status. Loss of status, a firmware
hold/fault, or an execution error triggers a hold request and action cancellation.
`HOLD` retains PWM torque; it does not cut motor power. Request it explicitly with:

```bash
ros2 service call /roboflex/hold std_srvs/srv/Trigger '{}'
```

An execution success is paired with final firmware command-angle verification.
It still requires physical observation because the servos have no encoder return
path. `--report /absolute/path/report.json` saves the planning and execution result.
Avoid simultaneous RViz goals while a script is executing.

Offline guard checks (no hardware access):

```bash
python3 roboflex_ros2/roboflex_moveit/test/test_goal_guards.py
```

## Key Config Files

- `config/roboflex_description.srdf`
- `config/kinematics.yaml`
- `config/joint_limits.yaml`
- `config/moveit_controllers.yaml`
