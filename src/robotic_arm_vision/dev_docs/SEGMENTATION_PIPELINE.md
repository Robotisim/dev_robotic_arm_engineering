# Pick and Place Pipeline: Object Detection and Segmentation

## Overview
This document outlines the pipeline for detecting and localizing colored cubes on a table using an **eye-in-hand** (gripper-mounted) depth camera for robotic pick-and-place operations.

**Camera Configuration**: The depth camera is mounted directly on the robot gripper, moving with the end-effector. This provides flexible viewpoints but requires accounting for the moving reference frame during detection.

## Available Sensor Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wrist_eye/image_raw` | `sensor_msgs/Image` | RGB color image |
| `/wrist_eye/depth/image_raw` | `sensor_msgs/Image` | Depth image (16UC1 or 32FC1) |
| `/wrist_eye/depth/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsic parameters |

## Pipeline Architecture

```
┌─────────────────┐
│  RGB Image      │
│  /wrist_eye/    │
│  image_raw      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐     ┌─────────────────┐
│ Color-Based     │────▶│ Blob Detection  │
│ Segmentation    │     │ & Filtering     │
│ (HSV Masking)   │     └────────┬────────┘
└─────────────────┘              │
                                 │ (u,v) pixel coords
                                 ▼
┌─────────────────┐     ┌─────────────────┐
│  Depth Image    │────▶│ Depth Lookup    │
│  /wrist_eye/    │     │ Z = depth[u,v]  │
│  depth/image_raw│     └────────┬────────┘
└─────────────────┘              │
                                 │
         ┌───────────────────────┘
         │
         ▼
┌─────────────────┐
│ Camera Info     │
│ (K matrix)      │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────┐
│ 3D Point Cloud Generation   │
│ (Deproject using intrinsics)│
│                             │
│ X = (u - cx) * Z / fx       │
│ Y = (v - cy) * Z / fy       │
│ Z = depth[u,v]              │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ Object Clustering           │
│ - Group points by color     │
│ - DBSCAN/Euclidean cluster  │
│ - Compute centroids         │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ 3D Object Localization      │
│ - Centroid in camera frame  │
│ - Bounding box estimation   │
│ - Orientation (optional)    │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ Transform to Robot Base     │
│ camera_frame → base_frame   │
│ (using TF2)                 │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ Grasp Pose Generation       │
│ - Position: object centroid │
│ - Orientation: top-down     │
│ - Pre-grasp offset          │
└─────────────────────────────┘
```

## Detailed Pipeline Steps

### 1. Color-Based Segmentation
**Purpose**: Identify pixels belonging to each colored cube

**Method**:
- Convert RGB image to HSV color space
- Define HSV ranges for each cube color (red, blue, green, etc.)
- Apply `cv2.inRange()` to create binary masks for each color
- Apply morphological operations to clean up masks:
  - Opening: Remove noise
  - Closing: Fill small gaps
  - Dilation: Expand regions slightly

**Output**: Binary mask per cube color (H × W, uint8)

### 2. Blob Detection and Filtering
**Purpose**: Extract distinct objects and their 2D locations

**Method**:
- Find contours in each color mask using `cv2.findContours()`
- Filter contours by:
  - Minimum area threshold (remove noise)
  - Maximum area threshold (remove false positives)
  - Aspect ratio (cubes should be roughly square when viewed from above)
- Compute 2D properties:
  - Centroid: $(u_c, v_c)$ in pixel coordinates
  - Bounding rectangle
  - Rotation angle (using moments or minimum area rect)

**Output**: List of detected objects with (u, v) pixel coordinates

### 3. Depth Lookup
**Purpose**: Get Z-depth for each detected object

**Method**:
- For each object centroid $(u_c, v_c)$:
  - Sample depth value: $Z = \text{depth\_image}[v_c, u_c]$
  - Handle invalid depths (NaN, 0, or > max_range)
  - Option A: Use single pixel at centroid
  - Option B: Average depth over small region (3×3 or 5×5 window)
  - Option C: Use median depth within object mask (robust to outliers)

**Considerations**:
- Depth images may have holes or invalid values
- Apply filtering to get robust depth estimate
- Convert depth encoding (mm to meters if needed)

**Output**: Depth value Z for each object (in meters)

### 4. 3D Point Deprojection
**Purpose**: Convert (u, v, Z) to 3D point in camera frame

**Method**:
Using camera intrinsic matrix K from `/wrist_eye/depth/camera_info`:

```
K = [fx  0  cx]
    [0  fy  cy]
    [0   0   1]
```

Deproject to 3D:
```
X_cam = (u - cx) * Z / fx
Y_cam = (v - cy) * Z / fy
Z_cam = Z
```

**Alternative**: Use `image_geometry.PinholeCameraModel` or `cv2.rgbd` utilities

**Output**: 3D point $(X_{cam}, Y_{cam}, Z_{cam})$ in camera optical frame

### 5. Point Cloud Clustering (Optional but Recommended)
**Purpose**: Refine object segmentation using 3D spatial information

**Method**:
- Generate point cloud from all pixels within each color mask
- Apply statistical outlier removal
- Perform Euclidean clustering (PCL or Open3D)
- Extract cluster centroids and bounding boxes
- Verify cluster size consistency (reject too small/large clusters)

**Benefits**:
- More robust to lighting variations
- Better handling of partial occlusions
- Accurate 3D extent estimation

**Output**: Refined 3D centroids and bounding boxes per object

### 6. Coordinate Transformation
**Purpose**: Convert object poses from camera frame to robot base frame

**Method**:
- Use TF2 to look up transform: `camera_optical_frame → base_link`
- Transform each object centroid:
  ```python
  from tf2_geometry_msgs import do_transform_point
  point_base = do_transform_point(point_camera, camera_to_base_tf)
  ```

**Important Frames** (Eye-in-Hand Configuration):
- `wrist_eye_optical_frame`: Camera optical frame (Z forward, X right, Y down)
- `wrist_eye_link`: Camera mount frame (physically attached to gripper)
- `tool0` or `wrist_3_link`: Gripper/end-effector frame
- `base_link`: Robot base frame (planning reference)

**Eye-in-Hand Considerations**:
- Camera pose changes with every robot movement
- TF tree: `base_link → ... → wrist_3_link → wrist_eye_link → wrist_eye_optical_frame`
- Transform is **time-dependent** - must use synchronized timestamps
- For detection: Position robot to get good view of workspace before capturing
- For grasping: Object positions are valid only at time of capture
- **Recommended workflow**:
  1. Move to observation pose (overlook workspace)
  2. Capture images and detect objects
  3. Transform object poses to base frame **at capture time**
  4. Use base frame coordinates for motion planning
  5. Re-detect after approach if needed (verification)

**Output**: 3D positions in robot base frame, stamped with capture time

### 7. Grasp Pose Generation
**Purpose**: Create target poses for pick-and-place operations

**Method**:
For each detected cube:
- **Position**: Object centroid + gripper offset
  - Z-offset: Approach from above (e.g., +0.05m above surface)
  - XY: Centered on object
- **Orientation**:
  - Top-down grasp: Gripper pointing downward
  - Quaternion: Typically (0, 0.707, 0, 0.707) for downward approach
  - Adjust based on gripper design
- **Pre-grasp pose**: Hover pose before gripping
- **Post-grasp pose**: Lift pose after successful grasp

**Output**: `geometry_msgs/PoseStamped` for MoveIt2 planning

## Technical Considerations

### Color Segmentation Parameters
- **HSV Ranges**: Must be calibrated for actual lighting conditions
  - Red cube: H=[0-10, 170-180], S=[100-255], V=[50-255]
  - Blue cube: H=[100-130], S=[100-255], V=[50-255]
  - Green cube: H=[40-80], S=[100-255], V=[50-255]
- **Adaptive Thresholding**: Consider lighting variations
- **Shadow Handling**: Shadows can affect color detection

### Depth Data Quality
- **Noise**: Intel RealSense typically has ~2% accuracy at 1m
- **Invalid Regions**: Reflective surfaces, edges may lack depth
- **Filtering**: Apply bilateral or Gaussian filtering to depth image
- **Temporal Smoothing**: Average over multiple frames for stability

### Calibration Requirements
1. **Camera Intrinsics**: Should be in camera_info (usually factory calibrated)
2. **Hand-Eye Calibration (Eye-in-Hand)**:
   - Transform from gripper/tool frame to camera optical frame
   - Usually fixed after camera mounting
   - Verify with `ros2 run tf2_ros tf2_echo tool0 wrist_eye_optical_frame`
   - If not calibrated, use calibration tools:
     - `easy_handeye2` package for ROS2
     - MoveIt2 hand-eye calibration tutorial
     - Requires checkerboard/ArUco marker and multiple poses
3. **Table Height**: Known or estimated from depth data
4. **Workspace Bounds**: Define safe operating volume
5. **Observation Pose**: Pre-defined pose for optimal view of workspace
   - Height: Typically 30-50cm above table
   - Angle: Slight tilt (15-30°) or top-down view
   - Coverage: All target objects visible in FOV

### Performance Considerations
- **Processing Rate**: Target 10-30 Hz for responsive control
- **Latency**: Minimize delay between detection and grasp execution
- **Multi-threaded**: Separate threads for perception and motion planning
- **Caching**: Store camera intrinsics, TF lookups to reduce overhead

### Error Handling
- **No Objects Detected**: Timeout, trigger error recovery
- **Multiple Objects**: Prioritize by distance or task order
- **Depth Dropout**: Fallback to 2D detection with assumed table height
- **TF Lookup Failures**: Buffer transforms with timeout

### Eye-in-Hand Workflow Strategy
**Challenge**: Camera moves with gripper, requiring deliberate observation poses

**Solution - Two-Phase Approach**:

**Phase A: Observation**
1. Move robot to pre-defined "scan pose" above workspace
2. Ensure all target objects are in camera FOV
3. Capture synchronized RGB + Depth images
4. Perform detection and localization
5. Transform all object poses to base frame
6. Store results for planning phase

**Phase B: Manipulation**
1. Use stored base frame coordinates for motion planning
2. Approach first target object
3. Optional: Re-verify object position when close
4. Execute grasp
5. Move to place location
6. Return to scan pose for next object

**Advantages**:
- Clear separation of perception and action
- Object positions referenced to stable base frame
- Can detect all objects in single observation
- Predictable and safe motion

**Alternative - Active Perception**:
- Move camera to different poses for better views
- Useful for occluded or distant objects
- Requires workspace exploration strategy
- More complex but more flexible

## Implementation Phases

### Phase 0: Setup and Calibration
- Verify camera is publishing to all three topics
- Check TF tree for camera→gripper→base transforms
- Define observation pose with good workspace view
- Test manual move to observation pose
- Verify camera intrinsics in camera_info

### Phase 1: Basic Color Segmentation
- Subscribe to RGB topic
- Implement HSV-based color masking
- Detect blob centroids in 2D
- Visualize detections on image
- Publish annotated image for debugging

### Phase 2: 3D Localization
- Subscribe to depth topic
- Implement deprojection using camera intrinsics
- Compute 3D positions in camera frame
- Publish as PointCloud2 or marker array for RViz

### Phase 3: Coordinate Transformation
- Set up TF2 buffer and listener
- Transform object poses to base frame with proper timestamping
- Publish as PoseArray for visualization
- Validate against known object positions (if available)
- Test observation workflow (move → detect → transform)

### Phase 4: Integration with Planner
- Generate grasp poses with approach vectors
- Interface with MoveIt2 action server
- Implement observation → detect → plan → execute sequence
- Add collision avoidance and error recovery

### Phase 5: Robustness and Refinement
- Add point cloud clustering
- Implement multi-frame averaging at observation pose
- Tune parameters for real-world conditions
- Add re-detection after failed grasps
- Implement active perception for difficult cases

## Expected Outputs

### ROS2 Messages
1. **Detection Results**: Custom message or MarkerArray
   ```
   Object[] detections
     - string color
     - geometry_msgs/Point position_camera
     - geometry_msgs/Point position_base
     - float32 confidence
     - int32 pixel_count
   ```

2. **Segmented Point Cloud**: `sensor_msgs/PointCloud2`
   - Colored point cloud with object labels
   - For visualization and validation

3. **Grasp Poses**: `geometry_msgs/PoseArray`
   - Target poses for each detected object
   - Published to MoveIt2 planning interface

### Visualization (RViz2)
- Annotated RGB image with bounding boxes
- 3D point cloud colored by segmentation
- TF frames: camera, objects, target poses
- Interactive markers for manual verification

## Next Steps

1. **Create ROS2 Package Structure**
   - Package name: `cube_segmentation` or `wrist_eye_perception`
   - Dependencies: cv_bridge, sensor_msgs, geometry_msgs, tf2_ros, tf2_geometry_msgs, PCL

2. **Node Architecture**
   - Option A: Single monolithic node
   - Option B: Separate nodes for segmentation, localization, planning
   - Recommended: Single node with service/action interface for detection on-demand

3. **Testing Strategy**
   - Unit tests: Color segmentation with sample images
   - Integration tests: End-to-end detection with rosbag
   - System tests: Physical grasp success rate
   - **Eye-in-hand specific**: Test TF synchronization at different robot poses

4. **Parameter Tuning Interface**
   - Dynamic reconfigure for HSV ranges
   - Launch file parameters for thresholds
   - Calibration script for new objects
   - Observation pose configuration (joint states or Cartesian pose)

5. **Key Development Tasks**
   - Implement observation pose manager
   - Create detection service for on-demand scanning
   - Add timestamp synchronization for RGB+Depth
   - Build TF buffer with sufficient history for moving camera

---

## References and Tools

### Libraries
- **OpenCV**: Image processing and color segmentation
- **cv_bridge**: ROS2-OpenCV conversion
- **image_geometry**: Camera model utilities
- **TF2**: Coordinate transformations
- **PCL/Open3D**: Point cloud processing (optional)

### ROS2 Packages
- `vision_msgs`: Standard detection message types
- `perception_pcl`: Point cloud processing nodes
- `depth_image_proc`: Depth to point cloud conversion

### Debugging Tools
- `rqt_image_view`: View camera topics
- `RViz2`: Visualize 3D data and TF frames
- `ros2_numpy`: Efficient array conversions
- `plotjuggler`: Real-time data plotting
- **Eye-in-hand specific**:
  - `ros2 run tf2_ros tf2_echo base_link wrist_eye_optical_frame`: Monitor camera pose
  - `ros2 run tf2_tools view_frames`: Generate TF tree PDF
  - `ros2 topic echo --once /wrist_eye/depth/camera_info`: Verify intrinsics
  - Message filters for RGB+Depth synchronization debugging

---

**Document Version**: 1.0  
**Last Updated**: February 23, 2026  
**Status**: Pipeline Design Phase
