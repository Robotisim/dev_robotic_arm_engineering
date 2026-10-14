# Firmware

ESP32 firmware for RoboFlex real hardware control over plain UDP (no micro-ROS).

## Active Project

- PlatformIO project: [`.`](.)
- Main source: [`src/main.cpp`](src/main.cpp)
- PlatformIO config: [`platformio.ini`](platformio.ini)
- Dev/safety sketch: [`src/main_dev.cpp`](src/main_dev.cpp)

## Runtime Protocol

- Transport: UDP (one-way, ROS2 host -> ESP32)
- Listening port: `ROBOFLEX_UDP_LISTEN_PORT` (default `9999`)
- Payload format: `CMD <joint_1> <joint_2> <joint_3> <joint_4> <joint_gripper>`
- Units: radians
- Exactly five finite values are required, within the hardware URDF joint bounds.
  Malformed, truncated, non-finite, and out-of-range packets are rejected.

## Live MoveIt startup

Upload with motor power off and the ESP32 connected by USB. The production
firmware clears all PCA9685 outputs at boot. Wait for `[READY] PWM outputs off`
and check the printed Wi-Fi address and joint limits before enabling motor power.
Do not open a second serial monitor: opening the port can reset some boards.

The initial reference is the repository's **Arm Up** pose, which the user
physically confirmed on 14 October 2026. The five servo command angles are
approximately `[2.001, 52.011, 36.973, 68.989, 90]` degrees on channels 1..5.
These are setpoints, not sensed positions. First arming applies this reference;
it cannot ramp from an unknown physical position after the arm has been moved
or sagged while unpowered.

Launch `roboflex_bringup hardware_gui.launch.py` with `firmware_ip` set to the
ESP32's printed address and `firmware_port:=9999`. Wait for the arm and gripper
controllers to become active. With motor power on and the arm at its confirmed
reference, send `ARM` followed by a newline over USB serial at 115200 baud.
The host's 200 ms UDP keepalive must remain active.

- `ARM`: enable the initial reference after boot, or resume UDP commands after
  a hold. A driver fault requires a reset instead.
- `HOLD`: latch the current PWM setpoints and ignore motion packets until a
  subsequent `ARM`. This retains motor torque; it does not cut motor power.
- Loss of valid UDP commands for two seconds also latches this hold. Before
  resuming, reconcile the host goal with the held physical pose: ROS state is
  command-derived and can have advanced while the firmware was holding.

In RViz select the `arm` planning group, drag the end-effector marker to set a
goal, then use **Plan** to preview and **Execute** to move. **Plan & Execute**
combines those steps. Marker dragging alone does not command the motors.
The configured joint velocity cap is 0.2 rad/s and acceleration cap is
0.4 rad/s², with both RViz scaling factors initially at 25%.

The ESP32 status exposes `armed`, `hold`, `driver_fault`, received command count,
and current/target command angles. ROS currently mirrors its own commands;
neither RViz nor a successful trajectory result verifies physical arrival.

Example payload:

```text
CMD 0.000000 -1.570000 -0.174444 0.000000 0.100000
```

## Required Configuration

Override network and UDP settings locally when building. Keep the real Wi-Fi
password out of tracked files. The available PlatformIO build definitions are:

```ini
build_flags =
  -D ROBOFLEX_WIFI_SSID=\"<ssid>\"
  -D ROBOFLEX_WIFI_PASSWORD=\"<password>\"
  -D ROBOFLEX_UDP_LISTEN_PORT=9999
  -D ROBOFLEX_PCA9685_ADDR=0x40
```

## Build and Upload

```bash
cd ~/ros2_ws/src/RoboFlexv2.1/roboflex_pio
pio run -e prod
pio run -e prod -t upload
pio device monitor -b 115200
```

## Serial Health Checks

On boot you should see:

- `WiFi connected. Local IP: ...`
- `Listening for UDP motor commands on port ...`
- no `[ERROR] PCA9685 not detected on I2C bus.`
- periodic `[STATUS]` lines with increasing `rx_count` when ROS2 is commanding

## Related Guides

- Top-level setup: [`../README.md`](../README.md)
- Real hardware bringup: [`../roboflex_ros2/roboflex_bringup/REAL_HARDWARE.md`](../roboflex_ros2/roboflex_bringup/REAL_HARDWARE.md)

## Five-joint USB bench test

The `bench` environment uses [`src/main_bench.cpp`](src/main_bench.cpp). It follows
the production mapping: arm joints on PCA9685 channels **1..4**, gripper on **5**,
I2C address `0x40`, SDA GPIO21, SCL GPIO22. Physical movement has been confirmed
for base channel **1** and the next arm joint on **2**; channels 3..5 still need
physical observation. It has no Wi-Fi or ROS listener and does not write
calibration to NVS.

Build from this directory with `pio run -e bench`. Upload with motor power off:
`pio run -e bench -t upload --upload-port <connected-port>`. Keep motor power off
until serial at 115200 baud reports `READY outputs=off`. Setup disables all PWM
channels; only channels 1..5 can subsequently receive position commands.
Support the arm as needed when its other joints are unpowered.

- `status`: driver state and commanded angles; there is no measured feedback.
- `init <1..5> <deg>`: enable one channel at an absolute servo target. The limits
  are `0..120` degrees, except channel 3 which is limited to `0..80`.
  This first pulse is **not speed controlled**: the physical starting angle is
  unknown. Use a known matching starting angle when testing relative motion.
- `move <1..5> <deg>`: slew an initialized channel to an absolute target at 10
  command degrees/second. Each move is limited to 50 command degrees; only one
  channel may slew at a time. A `DONE` response means the PWM ramp finished,
  not that the servo physically reached its target.
- `ping`: send at least once per second while moving. Without valid contact for
  two seconds, the ramp pauses and retains its last PWM command.
- `stop`: pause the ramp and retain PWM commands. This does not disconnect motor
  power or guarantee an immediate physical stop; use the motor supply switch
  for a physical stop.

The test retains the project's 150..600 PWM count calibration at 50 Hz. A
command angle is not a verified mechanical angle. There is no automatic return
stroke or automatic movement after reset.

For a user-requested zero reference, command one channel at a time to absolute
servo target `0`: use `init` if the output is disabled, otherwise `move` from
its current command, respecting the 50-degree maximum change per move. Check
physical behavior before advancing to the next channel. Do not label this
reference as ROS zero until the user has matched the physical pose in RViz.

Native logic checks (do not access hardware):

```bash
g++ -std=c++17 -Wall -Wextra -Werror -Itest/bench_native \
  test/bench_native/check.cpp -o /tmp/roboflex-bench-check
/tmp/roboflex-bench-check
```
