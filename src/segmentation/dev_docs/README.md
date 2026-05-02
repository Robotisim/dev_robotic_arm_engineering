# Cube Segmentation Package

Color-based segmentation for cube detection and localization using an eye-in-hand depth camera.

## Phase 1: Basic Color Segmentation ✅

Phase 1 implements HSV-based color detection with blob extraction and 2D visualization.

### Features
- ✅ Subscribe to RGB camera topic
- ✅ HSV color masking for multiple colors (red, blue, green, yellow)
- ✅ Blob detection with contour filtering
- ✅ 2D centroid extraction
- ✅ Annotated image visualization
- ✅ Parameter configuration

### Build Instructions

```bash
cd /home/darth-yoseph/ros2_ws
colcon build --packages-select cube_segmentation
source install/setup.bash
```

### Launch

```bash
# Launch the segmentation node
ros2 launch cube_segmentation segmentation.launch.py

# Optional: Specify custom config
ros2 launch cube_segmentation segmentation.launch.py config_file:=/path/to/config.yaml
```

### Topics

**Subscribed:**
- `/wrist_eye/image_raw` (`sensor_msgs/Image`) - RGB camera input

**Published:**
- `/segmentation/annotated_image` (`sensor_msgs/Image`) - Visualization with bounding boxes
- `/segmentation/detection_markers` (`visualization_msgs/MarkerArray`) - RViz markers

### Visualization

View the annotated image:
```bash
ros2 run rqt_image_view rqt_image_view /segmentation/annotated_image
```

Or in RViz2:
1. Add `Image` display
2. Set topic to `/segmentation/annotated_image`
3. Add `MarkerArray` display
4. Set topic to `/segmentation/detection_markers`

### Parameters

Edit `config/segmentation_params.yaml` to tune detection:

- `min_area`: Minimum blob area in pixels² (default: 500)
- `max_area`: Maximum blob area in pixels² (default: 50000)
- `min_aspect_ratio`: Minimum width/height ratio (default: 0.5)
- `max_aspect_ratio`: Maximum width/height ratio (default: 2.0)
- `morph_kernel_size`: Morphological operation kernel size (default: 5)
- `visualization_enabled`: Enable/disable visualization (default: true)
- `debug_images`: Show OpenCV debug windows (default: false)

### Color Calibration

The HSV ranges for each color are defined in `src/color_segmentation_node.cpp`:

```cpp
// Red: H=[0-10, 170-180], S=[100-255], V=[50-255]
// Blue: H=[100-130], S=[100-255], V=[50-255]
// Green: H=[40-80], S=[100-255], V=[50-255]
// Yellow: H=[20-35], S=[100-255], V=[50-255]
```

**Adjust these values** based on your lighting conditions and cube colors.

### Architecture

```
color_segmentation_node
├── Subscribes to RGB image
├── Converts BGR → HSV
├── Applies color masks for each target color
├── Performs morphological operations (open, close)
├── Detects contours and filters by area/aspect ratio
├── Computes 2D centroids using image moments
├── Visualizes detections with bounding boxes
└── Publishes annotated image and markers
```

## Coming Next

- **Phase 2**: 3D localization using depth data
- **Phase 3**: Coordinate transformation to base frame
- **Phase 4**: MoveIt2 integration for pick-and-place
- **Phase 5**: Robustness improvements

## Troubleshooting

### No image displayed
- Check if camera is publishing: `ros2 topic echo /wrist_eye/image_raw --once`
- Check topic remapping in launch file
- Verify node is running: `ros2 node list`

### No detections
- Adjust HSV ranges for your lighting
- Lower `min_area` threshold
- Enable `debug_images: true` to see masks
- Check cube colors match defined ranges

### Poor segmentation
- Increase `morph_kernel_size` for more smoothing
- Adjust aspect ratio thresholds
- Improve lighting conditions
- Calibrate HSV ranges

## Dependencies

- ROS2 Jazzy
- OpenCV 4.x
- cv_bridge
- image_transport
- sensor_msgs
- geometry_msgs
- visualization_msgs

## License

Apache-2.0
