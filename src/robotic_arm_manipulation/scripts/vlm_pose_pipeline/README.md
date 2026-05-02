# PaliGemma Pose From Image

Standalone Python 3.12 script to estimate object 3D camera-frame positions `(x, y, z)` from aligned RGB-D input.

## Files

- `paligemma_pose_from_image.py`: Runs PaliGemma for object localization and projects detections to 3D.
- `requirements.txt`: Python dependencies.

## Install (Python 3.12)

```bash
python3.12 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

## Intrinsics File Format

Use JSON or YAML with these keys:

```json
{
  "fx": 615.0,
  "fy": 615.0,
  "cx": 320.0,
  "cy": 240.0
}
```

## Run

```bash
python paligemma_pose_from_image.py \
  --rgb /path/to/rgb.png \
  --depth /path/to/depth.png \
  --intrinsics /path/to/intrinsics.json \
  --objects cup bottle \
  --output-json /tmp/poses.json
```

## Capture One Snapshot From A ROS2 Bag

If you already have a bag recording, extract one RGB-D frame plus intrinsics like this.

1. Inspect available topics:

```bash
ros2 bag info /path/to/bag
```

1. In terminal A, play the bag:

```bash
ros2 bag play /path/to/bag
```

1. In terminal B, run the snapshot tool:

```bash
python ros2_rgbd_snapshot.py \
  --output-dir /tmp/vlm_snapshot \
  --rgb-topic /wrist_eye/image_raw \
  --depth-topic /wrist_eye/depth/image_raw \
  --camera-info-topic /wrist_eye/depth/camera_info
```

1. Run pose inference from the saved files:

```bash
python paligemma_pose_from_image.py \
  --rgb /tmp/vlm_snapshot/rgb.png \
  --depth /tmp/vlm_snapshot/depth.npy \
  --intrinsics /tmp/vlm_snapshot/intrinsics.json \
  --objects cup bottle \
  --output-json /tmp/vlm_snapshot/poses.json
```

Snapshot output files:

- rgb.png
- depth.png
- depth.npy
- intrinsics.json
- metadata.json

## Notes

- Depth must be aligned to RGB.
- Depth input supports `.npy` or image files readable by OpenCV.
- Integer depth is interpreted as millimeters; floating depth is interpreted as meters.
- This script outputs 3D position only (not orientation).
- ros2_rgbd_snapshot.py requires ROS2 Python packages available in your sourced environment (rclpy, sensor_msgs, cv_bridge).
