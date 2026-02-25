# Pick and Place – Implementation Plan

## Overview

A modular C++ ROS2 package that:
1. Detects a colored object using the wrist-eye depth camera
2. Computes the object's 3D pose in the robot frame via TF2
3. Plans and executes a pick trajectory with MoveIt2
4. Moves the object to a fixed place pose and releases it

---

## External Dependencies

| Library | Purpose |
|---|---|
| `rclcpp` | ROS2 node lifecycle, timers, subscriptions |
| `sensor_msgs` | `Image`, `CameraInfo` message types |
| `geometry_msgs` | `PoseStamped`, `Point` |
| `cv_bridge` | Convert `sensor_msgs/Image` ↔ `cv::Mat` |
| `OpenCV` (core + imgproc) | HSV color thresholding, contour extraction, morphological ops |
| `image_geometry` | `PinholeCameraModel` for depth backprojection using camera intrinsics |
| `tf2_ros` + `tf2_geometry_msgs` | Transform object pose into `panda_link0` frame |
| `moveit_ros_planning_interface` | `MoveGroupInterface` for arm + gripper planning |

No neural networks, no point clouds, no object recognition frameworks.

---

## Package Layout

```
pick_place/
├── CMakeLists.txt
├── package.xml
├── objective.txt
├── IMPLEMENTATION.md
├── config/
│   └── params.yaml              # tunable parameters (HSV range, depth filtering, place pose)
├── launch/
│   └── pick_place.launch.py     # starts pick_place_node
├── include/
│   └── pick_place/
│       ├── object_detector.hpp
│       ├── gripper_controller.hpp
│       └── pick_place_executor.hpp
└── src/
    ├── object_detector.cpp
    ├── gripper_controller.cpp
    ├── pick_place_executor.cpp
    └── main.cpp
```

---

## Node / Class Responsibilities

### `ObjectDetector` (`object_detector.hpp/.cpp`)

**Purpose:** Detect a colored object and compute its 3D pose using direct depth image lookup and backprojection.

**Subscriptions:**
- `/wrist_eye/image_raw` (`sensor_msgs/Image`) – RGB image for color detection
- `/wrist_eye/depth/image_raw` (`sensor_msgs/Image`) – depth image (32FC1 or 16UC1)
- `/wrist_eye/camera_info` (`sensor_msgs/CameraInfo`) – camera intrinsics for backprojection

**Publishes:**
- `/pick_place/object_pose` (`geometry_msgs/PoseStamped`) – in the camera optical frame
- `/pick_place/debug_mask` (`sensor_msgs/Image`) – debug: binary color mask visualization

**Algorithm – OpenCV color detection + depth lookup + backprojection**

**Stage 1 – 2D color detection**
1. Convert RGB image to HSV with `cv::cvtColor`
2. Apply `cv::inRange` to get a binary mask of the target color
3. Apply morphological operations (`cv::morphologyEx` with opening/closing) to remove noise
4. Find contours with `cv::findContours`
5. Select the largest contour above `min_contour_area`
6. Compute the contour's moments with `cv::moments` to get centroid pixel coordinates `(u_c, v_c)`

**Stage 2 – Depth lookup and averaging**
7. Extract a small region (e.g., 5×5 or 7×7 pixels) centered at `(u_c, v_c)` from the depth image
8. Filter out invalid depth values (NaN, inf, zero, or beyond `depth_max`)
9. Compute median or mean depth from valid pixels → `depth`
10. Validate that `depth` is within `[depth_min, depth_max]` from `params.yaml`

**Stage 3 – 3D backprojection**
11. Load camera intrinsics `(fx, fy, cx, cy)` from `CameraInfo` using `image_geometry::PinholeCameraModel`
12. Backproject to 3D camera frame:
    ```
    X = (u_c - cx) * depth / fx
    Y = (v_c - cy) * depth / fy
    Z = depth
    ```
13. Publish `PoseStamped` with `(X, Y, Z)` and identity quaternion in the camera optical frame
14. Publish the binary mask to `/pick_place/debug_mask` for tuning HSV thresholds

**Why this approach:**  Simpler implementation with fewer dependencies. Direct depth lookup is fast and sufficient for single-object scenarios. Small region averaging around the centroid provides robustness against depth noise without needing full clustering.

---

### `GripperController` (`gripper_controller.hpp/.cpp`)

**Purpose:** Open and close the panda finger gripper.

**Uses:** `MoveGroupInterface` on the `hand` planning group (or sends a named target like `"open"` / `"close"`)

**API:**
```cpp
void open();
void close();
```

Internally calls `move_group.setNamedTarget("open")` / `"close"` then `move_group.move()`.

---

### `PickPlaceExecutor` (`pick_place_executor.hpp/.cpp`)

**Purpose:** Orchestrate the full pick-and-place sequence.

**Subscription:**
- `/pick_place/object_pose` (`geometry_msgs/PoseStamped`)

**Uses:**
- `tf2_ros::Buffer` + `tf2_ros::TransformListener` to transform the object pose into `panda_link0`
- `MoveGroupInterface` on the `arm` planning group
- `GripperController`

**State machine (simple enum):**

```
IDLE → DETECTING → PICKING → PLACING → IDLE
```

**Step-by-step sequence:**
1. **DETECTING** – Wait for a stable object pose (N consecutive detections within a threshold)
2. **PICKING**
   a. Transform pose to `panda_link0` via TF2
   b. Add a pre-grasp offset (+Z above the object)
   c. `move_group.setPoseTarget(pre_grasp_pose)` → `plan()` → `execute()`
   d. Approach straight down via a Cartesian path (`computeCartesianPath`)
   e. `GripperController::close()`
3. **PLACING**
   a. Retreat straight up via Cartesian path
   b. `move_group.setPoseTarget(place_pose)` (loaded from `params.yaml`) → `plan()` → `execute()`
   c. `GripperController::open()`
   d. Retreat to home (`move_group.setNamedTarget("ready")` → `move()`)
4. **IDLE** – reset, ready for next cycle

---

### `main.cpp`

Initialises `rclcpp`, creates the executor node (which internally constructs `ObjectDetector` and `PickPlaceExecutor`), and spins with a `MultiThreadedExecutor` so camera callbacks and the MoveIt planning thread don't block each other.

---

## Configuration (`config/params.yaml`)

```yaml
object_detector:
  # 2D color mask (OpenCV HSV)
  hsv_lower: [35, 100, 100]      # green object (H, S, V)
  hsv_upper: [85, 255, 255]
  min_contour_area: 500           # pixels² below which contours are ignored
  
  # Morphological operations
  morph_kernel_size: 5            # kernel size for opening/closing operations
  
  # Depth filtering
  depth_min: 0.1                  # metres; reject depth values closer than this
  depth_max: 2.0                  # metres; reject depth values further than this
  depth_sample_region: 5          # pixels; N×N region around centroid for depth averaging

pick_place_executor:
  approach_offset_z: 0.1          # metres above object before descent
  cartesian_step: 0.01            # metres per Cartesian step
  place_pose:                     # fixed place location in panda_link0
    x: 0.4
    y: -0.3
    z: 0.2
    qx: 1.0
    qy: 0.0
    qz: 0.0
    qw: 0.0
  detection_stability_count: 5    # frames before accepting detection
  detection_stability_dist: 0.02  # metres
```

---

## Build System (`CMakeLists.txt` + `package.xml`)

**Dependencies declared in `package.xml`:**
- `rclcpp`, `sensor_msgs`, `geometry_msgs`
- `cv_bridge`, `image_geometry`
- `libopencv-dev` (system dependency via rosdep)
- `tf2_ros`, `tf2_geometry_msgs`
- `moveit_ros_planning_interface`

**Two executables in `CMakeLists.txt`:**
- `object_detector_node` – links `ObjectDetector` only (useful for testing perception + PCL pipeline in isolation)
- `pick_place_node` – links everything; this is the main executable

---

## Launch (`launch/pick_place.launch.py`)

1. Load `config/params.yaml` as node parameters for `pick_place_node`
2. Launch `pick_place_node`
3. Optionally include the existing `panda_gz_control.launch.py` from `arm_sim_bringup`

> No additional image processing nodes required – the node subscribes directly to RGB and depth images from the camera.

---

## Implementation Order

Each step is designed to be independently testable. Build, test, verify, then move to the next step.

### Phase 1: Package Setup (Build System)

| Step | What to build | How to test | Success criteria |
|------|--------------|-------------|------------------|
| **1.1** | Create `package.xml` with all dependencies | `rosdep install --from-paths . --ignore-src -y` | No missing dependencies |
| **1.2** | Create minimal `CMakeLists.txt` (no executables yet) | `colcon build --packages-select pick_place` | Package builds without errors |
| **1.3** | Create `config/params.yaml` with all parameters from spec | Manual inspection | File matches config section |
| **1.4** | Create empty header files in `include/pick_place/` | `colcon build --packages-select pick_place` | Still builds |

### Phase 2: Object Detection (Vision Pipeline)

| Step | What to build | How to test | Success criteria |
|------|--------------|-------------|------------------|
| **2.1** | Basic `ObjectDetector` class skeleton with RGB subscriber only | Launch node, echo `/wrist_eye/image_raw` | Node starts, subscribes to camera |
| **2.2** | Add HSV conversion + color masking (Stage 1 only) | View `/pick_place/debug_mask` in `rqt_image_view` | Binary mask shows colored object |
| **2.3** | Add morphological operations (opening/closing) | View `/pick_place/debug_mask` | Cleaner mask, less noise |
| **2.4** | Add contour detection + centroid calculation | Publish centroid as `Point` on `/pick_place/debug_centroid` <br> Print `(u, v)` to console | Centroid tracks object center in 2D |
| **2.5** | Add depth image subscriber + camera_info subscriber | Echo all three topics | Node receives all camera data |
| **2.6** | Add depth lookup around centroid (Stage 2) | Print depth value to console | Reasonable depth (e.g., 0.3-1.5m) |
| **2.7** | Add depth region averaging + filtering | Print median depth | Stable depth despite noise |
| **2.8** | Add backprojection to 3D (Stage 3) using `image_geometry` | Echo `/pick_place/object_pose` <br> Visualize in RViz with TF frames | 3D pose appears at object location |
| **2.9** | Add `object_detector_node` executable to `CMakeLists.txt` | `ros2 run pick_place object_detector_node` | Standalone node publishes poses |

**Checkpoint 2**: You now have a working vision system that publishes 3D object poses. Test with different colored objects and lighting.

### Phase 3: Gripper Control (Hardware Interface)

| Step | What to build | How to test | Success criteria |
|------|--------------|-------------|------------------|
| **3.1** | Create `GripperController` class with MoveIt `hand` group | Build only, no executable yet | Compiles successfully |
| **3.2** | Implement `open()` method using named target | Create test node that calls `open()` | Gripper opens in simulation |
| **3.3** | Implement `close()` method using named target | Test node calls `close()` | Gripper closes in simulation |
| **3.4** | Add error handling (check if execution succeeded) | Test both methods, add obstacles | Logs success/failure appropriately |

**Checkpoint 3**: Gripper can be controlled independently. Verify in RViz and simulation.

### Phase 4: Motion Planning (Pick and Place Logic)

| Step | What to build | How to test | Success criteria |
|------|--------------|-------------|------------------|
| **4.1** | Create `PickPlaceExecutor` skeleton with object pose subscriber | Echo `/pick_place/object_pose` | Node subscribes, receives poses |
| **4.2** | Add TF2 buffer/listener, transform pose to `panda_link0` | Print transformed pose to console | Coordinates in robot base frame |
| **4.3** | Add detection stability logic (N consecutive detections) | Move object, watch console logs | Only triggers after stable detection |
| **4.4** | Implement pre-grasp pose calculation (offset above object) | Visualize pre-grasp pose in RViz as marker | Pose is above object by offset |
| **4.5** | Add MoveIt `arm` group and plan to pre-grasp (no execute) | View planned path in RViz MoveIt plugin | Valid collision-free path |
| **4.6** | Add Cartesian descent path (computeCartesianPath) | Plan only, visualize in RViz | Straight-line approach to object |
| **4.7** | Integrate gripper close after approach | **Plan only** - add to state machine | State transitions look correct in logs |
| **4.8** | Add Cartesian retreat (straight up) | Plan retreat path | Straight-line lift from object |
| **4.9** | Add place pose target and plan | Move to place location (plan only) | Path to place pose valid |
| **4.10** | Integrate gripper open at place | **Plan only** - complete state machine | Full sequence logged correctly |
| **4.11** | Add return to home position | Plan to "ready" named target | Valid path back to home |

**Checkpoint 4**: Full pick-and-place sequence **planned** but not executed. Review all paths in RViz.

### Phase 5: Execution and Integration

| Step | What to build | How to test | Success criteria |
|------|--------------|-------------|------------------|
| **5.1** | Change all `plan()` calls to `execute()` in PICKING state only | Execute pick sequence only | Robot moves to object and grasps |
| **5.2** | Enable execution for PLACING state | Full pick and place with stationary object | Object moved to place location |
| **5.3** | Add error recovery (re-try on failure, timeout checks) | Induce failures (remove object mid-pick) | Graceful recovery or abort |
| **5.4** | Add `pick_place_node` executable to `CMakeLists.txt` | `ros2 run pick_place pick_place_node` | Full node runs end-to-end |
| **5.5** | Create `pick_place.launch.py` with parameters | `ros2 launch pick_place pick_place.launch.py` | Everything launches together |

**Checkpoint 5**: Full autonomous pick-and-place working in simulation.

### Phase 6: Tuning and Optimization

| Step | What to tune | How to test | Success criteria |
|------|-------------|-------------|------------------|
| **6.1** | HSV color thresholds (`hsv_lower`, `hsv_upper`) | Different lighting, object colors | Reliable detection in varied conditions |
| **6.2** | Depth filtering (`depth_min`, `depth_max`, `sample_region`) | Different object distances | Stable depth readings |
| **6.3** | Approach parameters (`approach_offset_z`, `cartesian_step`) | Different object heights | Smooth approach without collision |
| **6.4** | Place pose and detection stability | Different scenarios | Repeatable, accurate placement |
| **6.5** | Timing and state transitions | Rapid object movement | Robust detection, no false triggers |

**Checkpoint 6**: Package is robust and tuned for your specific scenario.

---

### Quick Testing Commands

```bash
# Test object detection only
ros2 run pick_place object_detector_node

# View debug mask
ros2 run rqt_image_view rqt_image_view /pick_place/debug_mask

# Echo object pose
ros2 topic echo /pick_place/object_pose

# Launch full system
ros2 launch pick_place pick_place.launch.py

# Visualize in RViz
rviz2
```

### Troubleshooting Tips per Phase

**Phase 2 (Vision):**
- If mask is noisy → increase `morph_kernel_size`
- If no mask → adjust HSV ranges, check camera is publishing
- If depth is NaN → check depth image encoding, increase `sample_region`
- If 3D pose is wrong → verify camera_info intrinsics, check TF tree

**Phase 3 (Gripper):**
- If gripper doesn't move → verify SRDF has "open"/"close" named targets
- If planning fails → check collision geometry in URDF

**Phase 4 (Planning):**
- If no valid plan → increase planning time, simplify scene
- If Cartesian path fails → reduce `cartesian_step`, check reachability
- If TF transform fails → verify frames exist with `tf2_echo`

**Phase 5 (Execution):**
- If robot moves erratically → review planned paths in step 5.1 before executing
- If execution hangs → add timeouts, check controller state
- If grip fails → verify object size matches gripper width

---

## Key Design Decisions

- **Direct depth lookup approach**: Simpler than point clouds – no PCL dependency, less memory, faster processing. Sufficient for single-object pick-and-place scenarios.
- **image_geometry for backprojection**: Standard ROS2 package that handles camera model intrinsics cleanly, avoiding manual matrix math.
- **Depth region averaging**: Sampling a small N×N region around the color centroid and taking the median provides robustness against depth sensor noise without needing full clustering.
- **Morphological filtering**: Simple opening/closing operations on the binary mask remove small noise blobs and fill small holes, making contour detection more stable.
- **Debug mask topic**: Publishing the binary mask on `/pick_place/debug_mask` makes HSV threshold tuning straightforward by visualizing exactly what the detector sees.
- **No synchronisation overhead**: RGB and depth images are processed when available; slight timestamp mismatch is acceptable for slow-moving or static objects. If needed, `message_filters::ApproximateTime` can be added later.
- **Cartesian approach/retreat**: Avoids collision with the table by constraining the final centimetres to a straight line rather than a free joint-space plan.
- **Named gripper targets**: Relies on existing MoveIt2 SRDF named states (`"open"`, `"close"`) already defined in the panda SRDF – no custom action client needed.
- **TF2 transform at execution time**: The pose is transformed only when the arm starts moving, ensuring the freshest available transform is used.
