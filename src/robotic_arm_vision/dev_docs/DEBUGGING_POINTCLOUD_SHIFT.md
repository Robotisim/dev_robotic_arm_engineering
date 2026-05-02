# Debugging Point Cloud Position Shifts

## Problem
The detected cubes appear correctly oriented (below gripper) but their positions in the point cloud don't match the actual positions in Gazebo simulation.

## Diagnostic Steps

### Step 1: Run the Calibration Checker

```bash
# Terminal 1: Make sure your simulation is running
# Terminal 2: Run the diagnostic tool
cd /home/darth-yoseph/ros2_ws
source install/setup.bash
python3 src/dev_robotic_arm_engineering/src/segmentation/scripts/camera_calibration_check.py
```

**What to check:**
- Camera intrinsics (fx, fy, cx, cy) - should match camera resolution
- Depth values at center pixel - should be reasonable (0.3-1.0m typically)
- 3D position in base frame - compare with Gazebo model positions

### Step 2: Verify Camera Frame Orientation

```bash
# Check the transform chain
ros2 run tf2_ros tf2_echo panda_wrist_eye_link panda_wrist_eye_optical_frame
```

**Expected rotation matrix** (with current 90° Y rotation):
```
Matrix should show:
  X_optical aligned with Z_camera (forward from gripper)
  Y_optical aligned with Y_camera (left)
  Z_optical aligned with X_camera (down)
```

### Step 3: Test Alternative Optical Frame Rotations

If positions are still shifted, try different optical frame rotations:

**Option A: Current (90° around Y)**
```python
"0", "0.7071068", "0", "0.7071068"
```

**Option B: Different convention (try if Option A doesn't work)**
Edit [segmentation.launch.py](../launch/segmentation.launch.py) line ~43:
```python
# Replace rotation quaternion with:
"-0.5", "0.5", "0.5", "0.5"  # Combination of rotations
```

**Option C: Just identity (no rotation)**
```python
"0", "0", "0", "1"  # Use camera link directly
```

### Step 4: Check Depth/RGB Alignment

```bash
# View both images side by side
ros2 run rqt_image_view rqt_image_view /wrist_eye/image_raw &
ros2 run rqt_image_view rqt_image_view /wrist_eye/depth/image_raw
```

**Look for:**
- Do the depth and RGB show the same view?
- Is there a spatial offset between them?
- Are objects in the same position in both images?

### Step 5: Verify Depth Encoding and Scaling

```bash
# Check depth image properties
ros2 topic echo /wrist_eye/depth/image_raw --once | grep -E "(encoding|height|width)"
```

**Common issues:**
- Encoding `16UC1` = millimeters (need to divide by 1000)
- Encoding `32FC1` = meters (use directly)
- Wrong scaling factor causes wrong distances

### Step 6: Manual Position Verification

1. **Place a cube at a known position in Gazebo** (e.g., X=0.5, Y=0.0, Z=0.1 from panda_link0)
2. **Move gripper to look at the cube** (e.g., hover above it)
3. **Check detected position**:
   ```bash
   ros2 topic echo /segmentation/object_poses_base --once
   ```
4. **Compare** the detected position with the known Gazebo position
5. **Calculate error**: `sqrt((dx)^2 + (dy)^2 + (dz)^2)`

## Common Issues and Solutions

### Issue 1: Position Offset is Constant
**Symptom**: All objects shifted by same amount (e.g., +10cm in X)
**Cause**: Optical frame transform has wrong translation
**Fix**: Add translation offset in launch file:
```python
arguments=[
    "0.1", "0", "0",  # Add X offset (adjust as needed)
    "0", "0.7071068", "0", "0.7071068",
    ...
]
```

### Issue 2: Positions Scaled Wrong
**Symptom**: Objects appear 2x too far or 0.5x too close
**Cause**: Depth scale or camera intrinsics wrong
**Fix**: Check camera intrinsics with diagnostic tool, verify against Gazebo camera plugin parameters

### Issue 3: X and Y Swapped
**Symptom**: Position is correct distance but wrong direction
**Cause**: Wrong optical frame rotation
**Fix**: Try alternative rotation quaternions in Step 3

### Issue 4: Depth Values are NaN or 0
**Symptom**: No 3D detections, warnings about invalid depth
**Cause**: Depth camera not working or wrong topic
**Fix**: 
- Check `ros2 topic list | grep depth`
- Verify depth image shows actual depth data (not all black)
- Check Gazebo camera plugin configuration

### Issue 5: TF Transform Failures
**Symptom**: Warnings about "frame does not exist"
**Cause**: Optical frame not published or wrong frame name
**Fix**:
- Verify launch file is running: `ros2 node list | grep static`
- Check TF tree: `ros2 run tf2_tools view_frames && evince frames.pdf`

## Testing with Known Markers

Place objects at known positions for calibration:

```bash
# In Gazebo, spawn a cube at known position (example)
ros2 run gazebo_ros spawn_entity.py -entity test_cube \
    -x 0.5 -y 0.0 -z 0.05 \
    -file <path_to_cube_model>
```

Then verify detected position matches `X=0.5, Y=0.0, Z=0.05` in base frame.

## Visualization Tips

```bash
# In RViz, add these displays:
# 1. TF - Show all frames, especially:
#    - panda_link0 (base)
#    - panda_wrist_eye_link (camera mount)
#    - panda_wrist_eye_optical_frame (optical frame)
# 2. PoseArray - topic: /segmentation/object_poses_base
# 3. PointCloud2 - topic: /segmentation/detected_objects_cloud
# 4. Image - topic: /segmentation/annotated_image
# 5. Markers - topic: /segmentation/detection_markers

# Set Fixed Frame to: panda_link0
```

## Expected Accuracy

With proper calibration:
- **Position accuracy**: ±2cm at 0.5m distance
- **Depth accuracy**: ±1% of distance (Intel RealSense typical)
- **Detection rate**: >95% with good lighting

## Next Steps After Fixing

Once positions are accurate:
1. Document the correct optical frame transform
2. Update configuration files
3. Test with multiple objects at different positions
4. Move to Phase 4: Grasp pose generation

## Contact Information

If positions remain inaccurate after trying:
- All rotation options
- Verified camera intrinsics
- Checked depth encoding

Then check:
- Gazebo camera plugin URDF/SDF configuration
- Hand-eye calibration (relationship between gripper and camera)
- Physical camera mounting (if using real hardware later)
