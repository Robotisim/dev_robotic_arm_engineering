# Pick and Place – Python Node Implementation Plan

## Overview

A modular Python ROS2 package that:
1. Subscribes to a `geometry_msgs/PoseStamped` object pose topic (e.g. published by the existing `ObjectDetector` C++ node or any other perception pipeline)
2. Transforms the pose into the robot base frame via TF2
3. Plans and executes a pick trajectory using the MoveIt2 Python bindings (`moveit_py`)
4. Moves the object to a configurable fixed place pose and releases the gripper

The node is self-contained. It asks nothing of the vision stack and can run alongside or independently of any perception node that publishes on the configured pose topic.

---

## External Dependencies

| Package | Purpose |
|---|---|
| `rclpy` | ROS2 node, subscriptions, logging, parameters |
| `geometry_msgs` | `PoseStamped` |
| `tf2_ros` | `Buffer`, `TransformListener` |
| `tf2_geometry_msgs` | `do_transform_pose` helper |
| `moveit_py` | `MoveItPy`, `PlanRequestParameters`, Cartesian path planning |
| `moveit_msgs` | `CollisionObject`, `RobotTrajectory` |
| `shape_msgs` | Used by `CollisionObject` if scene objects are needed |
| `std_msgs` | `Bool` for simple status publishing |

> **ROS2 Jazzy note**: The official MoveIt2 Python bindings ship as `moveit_py`. Import paths are `from moveit.planning import MoveItPy` and `from moveit.core.robot_state import RobotState`. No `pymoveit2` third-party wrapper is used.

---

## Package Layout

```
pick_place/
├── ...                          # existing C++ files unchanged
├── implement_py.md              # this document
├── config/
│   └── py_params.yaml           # all tunable Python-node parameters
├── launch/
│   └── pick_place_py.launch.py  # launches pick_place_py_node
└── pick_place/                  # Python package (ament_python style, added alongside C++)
    ├── __init__.py
    ├── config_loader.py         # loads and validates py_params.yaml at startup
    ├── tf_helper.py             # thin wrapper around tf2_ros Buffer/TransformListener
    ├── gripper_controller.py    # open / close via MoveItPy hand group
    ├── pick_place_executor.py   # state machine + MoveItPy arm planning
    └── main.py                  # entry point, wires everything together
```

> The Python package lives inside the existing ament_cmake `pick_place` CMake package. `CMakeLists.txt` and `package.xml` need minor additions (see Phase 1).

---

## Module Responsibilities

### `config_loader.py`

**Purpose:** Single-source-of-truth for all tunable parameters. Reads from ROS2 node parameters (declared from `py_params.yaml`), validates types and ranges, and exposes a plain `PickPlaceConfig` dataclass to the rest of the code.

```python
@dataclass
class PickPlaceConfig:
    # Topic / frame settings
    object_pose_topic: str
    base_frame: str
    ee_link: str
    arm_group: str
    hand_group: str

    # Approach geometry
    approach_offset_z: float       # metres above object for pre-grasp
    retreat_offset_z: float        # metres above object to retreat after grasp
    cartesian_eef_step: float      # metres between Waypoints
    cartesian_jump_threshold: float

    # Place pose (in base_frame)
    place_x: float
    place_y: float
    place_z: float
    place_qx: float
    place_qy: float
    place_qz: float
    place_qw: float

    # Detection stability
    stability_count: int           # consecutive detections before triggering pick
    stability_dist_m: float        # max position drift (metres) between detections

    # Execution control
    execute_enabled: bool          # False = plan-only dry-run mode
    planning_time_s: float
    max_retries: int
```

All parameters are declared on the node at startup and can be overridden via `py_params.yaml` or command-line `--ros-args -p`.

---

### `tf_helper.py`

**Purpose:** Thin, reusable wrapper that transforms a `PoseStamped` into a target frame, with a configurable timeout and error-safe return.

```python
class TFHelper:
    def __init__(self, node: rclpy.node.Node) -> None: ...

    def transform_pose(
        self,
        pose: PoseStamped,
        target_frame: str,
        timeout_s: float = 1.0,
    ) -> PoseStamped | None:
        """Returns transformed pose or None on failure."""
```

---

### `gripper_controller.py`

**Purpose:** Wraps the `MoveItPy` `hand` planning group to open and close the gripper using named states defined in the SRDF (`"open"`, `"close"`).

```python
class GripperController:
    def __init__(self, moveit: MoveItPy, hand_group: str) -> None: ...

    def open(self) -> bool:   ...   # returns True on success
    def close(self) -> bool:  ...
```

Internally:
1. `hand = moveit.get_planning_component(hand_group)`
2. `hand.set_start_state_to_current_state()`
3. `hand.set_goal_state(configuration_name="open")` / `"close"`
4. `plan_result = hand.plan()`
5. `moveit.execute(plan_result.trajectory, ...)` if `execute_enabled`

---

### `pick_place_executor.py`

**Purpose:** Orchestrates the full pick-and-place loop via a state machine. Owns the MoveIt2 arm planning component and the gripper controller.

#### State Machine

```
IDLE
 │  N stable detections received
 ▼
DETECTING
 │  stability check passes
 ▼
PICKING
 │  a) Transform pose → base_frame
 │  b) Plan to pre-grasp (pose + Z offset)
 │  c) Execute pre-grasp move
 │  d) Cartesian descend to grasp pose
 │  e) gripper.close()
 ▼
PLACING
 │  a) Cartesian retreat upward
 │  b) Plan & execute to place pose
 │  c) gripper.open()
 │  d) Plan & execute to home ("ready" named target)
 ▼
IDLE  ← loops indefinitely
```

Any step returning `False` transitions to `RECOVERING` → logs error → returns to `IDLE`.

#### Key methods

```python
class PickPlaceExecutor:
    def __init__(
        self,
        node: rclpy.node.Node,
        config: PickPlaceConfig,
        tf_helper: TFHelper,
        gripper: GripperController,
        moveit: MoveItPy,
    ) -> None: ...

    # Called from subscription callback
    def on_object_pose(self, msg: PoseStamped) -> None: ...

    # Internal helpers
    def _check_stability(self, pose: PoseStamped) -> bool: ...
    def _plan_to_pose(self, pose: PoseStamped) -> bool: ...
    def _cartesian_move(self, waypoints: list[Pose], label: str) -> bool: ...
    def _go_home(self) -> bool: ...
```

---

### `main.py`

Entry point. Initialises `rclpy`, builds the `MoveItPy` instance, creates all modules, wires the subscription, and spins with `rclpy.spin`.

```python
def main():
    rclpy.init()
    node = rclpy.create_node("pick_place_py_node")

    config = ConfigLoader(node).load()
    moveit = MoveItPy(node_name="pick_place_moveit_py")
    tf_helper = TFHelper(node)
    gripper = GripperController(moveit, config.hand_group)
    executor = PickPlaceExecutor(node, config, tf_helper, gripper, moveit)

    node.create_subscription(
        PoseStamped,
        config.object_pose_topic,
        executor.on_object_pose,
        10,
    )

    rclpy.spin(node)
    rclpy.shutdown()
```

---

## Configuration (`config/py_params.yaml`)

```yaml
pick_place_py_node:
  ros__parameters:

    # ----- Topic / frame -----
    object_pose_topic: "/pick_place/object_pose"
    base_frame: "panda_link0"
    ee_link: "panda_hand_tcp"
    arm_group: "panda_arm"
    hand_group: "hand"

    # ----- Approach geometry -----
    approach_offset_z: 0.12       # metres above object for pre-grasp waypoint
    retreat_offset_z: 0.15        # metres to lift after grasp
    cartesian_eef_step: 0.01      # metres per Cartesian interpolation step
    cartesian_jump_threshold: 0.0 # 0 disables jump detection (safe in sim)

    # ----- Place pose (panda_link0 frame) -----
    place_x: 0.4
    place_y: -0.3
    place_z: 0.25
    place_qx: 1.0
    place_qy: 0.0
    place_qz: 0.0
    place_qw: 0.0

    # ----- Detection stability -----
    stability_count: 5            # N consecutive poses required before triggering
    stability_dist_m: 0.02        # max inter-frame drift (metres)

    # ----- Execution control -----
    execute_enabled: true         # set false for plan-only dry runs
    planning_time_s: 5.0
    max_retries: 3
```

All values exposed as ROS2 node parameters → changeable at runtime with `ros2 param set` without recompiling.

---

## Build System Changes

### `package.xml` additions

```xml
<!-- Python bindings and runtime -->
<depend>moveit_py</depend>
<depend>tf2_ros</depend>
<depend>tf2_geometry_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>

<!-- Ament Python support -->
<buildtool_depend>ament_cmake_python</buildtool_depend>
```

### `CMakeLists.txt` additions

```cmake
find_package(ament_cmake_python REQUIRED)

# Install the Python package
ament_python_install_package(pick_place)

# Install Python scripts as executable entry points
install(PROGRAMS
  pick_place/main.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pick_place_py_node
)

# Install config and launch
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
```

---

## Launch File (`launch/pick_place_py.launch.py`)

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory("pick_place")
    params = os.path.join(pkg, "config", "py_params.yaml")

    return LaunchDescription([
        Node(
            package="pick_place",
            executable="pick_place_py_node",
            name="pick_place_py_node",
            output="screen",
            parameters=[params],
        )
    ])
```

To also bring up the existing C++ object detector alongside it, include `arm_sim_bringup`'s launch and add an `object_detector_node` in the same `LaunchDescription`.

---

## Implementation Order

Each step is independently testable. Build and verify before proceeding.

### Phase 1 – Package Setup

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **1.1** | Add `ament_cmake_python` to `package.xml` + `CMakeLists.txt` | `colcon build --packages-select pick_place` | Builds with no errors |
| **1.2** | Create `pick_place/__init__.py` (empty) and stub `main.py` | `colcon build` | Python package visible via `ros2 pkg` |
| **1.3** | Create `config/py_params.yaml` | Manual inspection | File matches spec |
| **1.4** | Create `launch/pick_place_py.launch.py` (Node only, no params yet) | `ros2 launch pick_place pick_place_py.launch.py` | Node starts and exits cleanly |

### Phase 2 – Config Loader

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **2.1** | Implement `PickPlaceConfig` dataclass | Unit import in Python REPL | All fields accessible |
| **2.2** | Implement `ConfigLoader.load()` – declare all params, read values | `ros2 run pick_place pick_place_py_node` | Logging shows loaded config values |
| **2.3** | Add param validation (range checks, non-empty strings) | Pass invalid `planning_time_s: -1` | Node raises `ValueError` with clear message |

### Phase 3 – TF Helper

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **3.1** | Implement `TFHelper.__init__` with `Buffer` + `TransformListener` | Build only | Compiles |
| **3.2** | Implement `transform_pose` with timeout and `None` fallback | Publish a `PoseStamped` in camera frame, call helper | Transformed pose printed in correct frame |
| **3.3** | Test failure path (unknown frame) | Pass bogus frame name | Returns `None`, logs warning, no crash |

### Phase 4 – Gripper Controller

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **4.1** | Implement `GripperController.__init__` – get `hand` planning component | Build, node starts with MoveItPy | No exception at startup |
| **4.2** | Implement `open()` with named target | Call `open()` from a test script | Gripper opens in simulation / RViz |
| **4.3** | Implement `close()` with named target | Call `close()` | Gripper closes |
| **4.4** | Add `execute_enabled` guard | Run with `execute_enabled: false` | Logs "DRY RUN – would execute open/close", no motion |

**Checkpoint 4**: Gripper controllable independently. Verify in RViz and Gazebo.

### Phase 5 – Arm Planning (No Execution)

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **5.1** | Implement `PickPlaceExecutor.__init__` – create `arm` planning component | Build + start node | No crash at startup |
| **5.2** | Implement `on_object_pose` stub – just log received pose | `ros2 topic pub` a pose | Log line appears |
| **5.3** | Implement `_check_stability` | Publish same pose 5× | "Detection stable" log after N messages |
| **5.4** | Implement `_plan_to_pose` (pre-grasp pose only, plan but do not execute) | Trigger stability | Planned trajectory visible in RViz |
| **5.5** | Implement `_cartesian_move` for descend | Add to PICKING state, plan only | Straight descent path in RViz |
| **5.6** | Implement `_cartesian_move` for retreat | Add to PLACING state, plan only | Straight retreat path in RViz |
| **5.7** | Implement plan to place pose | Add to PLACING state | Path to place pose planned |
| **5.8** | Implement `_go_home` (named target `"ready"`) | Add at end of PLACING | Return-to-home path planned |

**Checkpoint 5**: Full pick-and-place sequence planned, zero execution. Review all paths in RViz before proceeding.

### Phase 6 – Execution and Integration

| Step | Task | Test | Pass criteria |
|------|------|------|---------------|
| **6.1** | Enable execution for PICKING sub-steps only | `execute_enabled: true` | Robot moves to pre-grasp and descends, gripper closes |
| **6.2** | Enable execution for PLACING sub-steps | Full loop with a stationary object | Object moved to place location, gripper opens |
| **6.3** | Add retry logic (`max_retries`) for planning failures | Introduce a collision to force failure | Node retries then transitions to IDLE |
| **6.4** | Add overall loop reset: state → IDLE after completing or aborting | Run two cycles back-to-back | Second pick triggered without restart |

**Checkpoint 6**: Full autonomous pick-and-place working in simulation.

### Phase 7 – Tuning

| Step | What to tune | How to test | Pass criteria |
|------|-------------|-------------|---------------|
| **7.1** | `approach_offset_z`, `retreat_offset_z` | Different object heights | No table collision, clean grasp |
| **7.2** | `cartesian_eef_step` | Dense / sparse scenes | Smooth Cartesian paths |
| **7.3** | `stability_count` + `stability_dist_m` | Moving object | No premature triggering |
| **7.4** | `place_pose` | Multiple place locations | Accurate, repeatable placement |
| **7.5** | `planning_time_s` | Complex scenes | Plans found reliably |

---

## Quick Reference Commands

```bash
# Build Python package only
colcon build --packages-select pick_place --symlink-install

# Launch Python pick-and-place node
ros2 launch pick_place pick_place_py.launch.py

# Run node directly (debug mode, plan only)
ros2 run pick_place pick_place_py_node --ros-args \
  -p execute_enabled:=false \
  --params-file install/pick_place/share/pick_place/config/py_params.yaml

# Override a single param at runtime (no rebuild)
ros2 param set /pick_place_py_node approach_offset_z 0.15

# Simulate an object pose for testing (bypass vision stack)
ros2 topic pub /pick_place/object_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'panda_link0'}, pose: {position: {x: 0.45, y: 0.0, z: 0.05}, orientation: {w: 1.0}}}"

# Echo current node parameters
ros2 param dump /pick_place_py_node
```

---

## Key Design Decisions

- **`moveit_py` only**: Uses the official ROS2 Jazzy MoveIt2 Python bindings, not third-party wrappers. `MoveItPy` is stable in Jazzy and avoids ActionClient overhead.
- **Dataclass config**: All tunable values live in `PickPlaceConfig`. No magic strings scattered across files – swap `py_params.yaml` values and everything adapts.
- **`execute_enabled` flag**: Setting `execute_enabled: false` turns the entire node into a dry-run planner. Essential for reviewing paths in RViz before committing to hardware motion.
- **`None`-returning helpers**: `_plan_to_pose`, `_cartesian_move`, and TF transforms all return `bool` or `None` rather than raising exceptions. The state machine handles failures explicitly without crashing.
- **Flat state machine (enum + if/elif)**: Keeps control flow readable and avoids the complexity of async state frameworks for a sequential pick-and-place task.
- **Subscription-driven**: The node stays responsive to new poses between cycles. If a new stable pose arrives while in `IDLE`, the next cycle starts automatically.
- **No synchronisation with vision**: The executor consumes whatever pose is on the topic. Tight coupling to a specific vision node is avoided by accepting any `PoseStamped` publisher on the configured topic.
- **`symlink-install`**: Recommended during development so Python file edits take effect without a rebuild.
