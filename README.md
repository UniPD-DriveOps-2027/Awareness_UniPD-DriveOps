# Awareness_UniPD-DriveOps

ROS 2 repository for the UniPD DriveOps robot spatial-awareness stack. Awareness is kept separate from localization, planning, and vehicle control.

## Repository layout

```
Awareness_UniPD-DriveOps/
├── src/bfmc_sign_detection/
│   ├── bfmc_sign_detection/road_sign_detector.py
│   ├── config/awareness.yaml
│   ├── launch/awareness.launch.py
│   ├── models/bfmc_signs_best.pt
│   ├── package.xml
│   ├── setup.py
│   └── setup.cfg
├── requirements.txt
└── README.md
```

This is an umbrella package. The current sign-detection module is `bfmc_sign_detection`. Future lane, stop-line, obstacle, and traffic-light modules should be added beside it.

## Current node: road_sign_detector

The node:

1. Runs the trained Ultralytics YOLO model on an RGB frame.
2. Selects the highest-confidence road-sign detection.
3. Reads RGB-registered stereo depth inside the detection ROI.
4. Uses the median of the inner ROI to reject background and invalid pixels.
5. Backprojects the detection centre using stereo `CameraInfo` intrinsics.
6. Publishes the Euclidean 3-D range and sign metadata as JSON.

Stereo depth is used whenever it is valid and time-compatible. If it is unavailable, stale, invalid, or has fewer than five valid pixels, the node uses a monocular bounding-box estimate. The output field `range_source` is either `stereo` or `bbox`.

## Requirements

- Ubuntu with ROS 2 Jazzy or compatible ROS 2.
- Python 3, `rclpy`, `sensor_msgs`, `std_msgs`, and `cv_bridge`.
- OpenCV and NumPy.
- Python `ultralytics` with PyTorch.
- RGB camera and RGB-registered stereo depth.

The supplied model is approximately 122 MB. It is intentionally excluded
from Git and must be copied manually after cloning:

```
src/bfmc_sign_detection/models/bfmc_signs_best.pt
```

After cloning, copy the model into that path from your secure storage:

```bash
cp /path/to/bfmc_signs_best.pt \\
  src/bfmc_sign_detection/models/bfmc_signs_best.pt
```

Alternatively, download it from your private storage using your own
authenticated transfer method. Confirm it is present before launching:

```bash
ls -lh src/bfmc_sign_detection/models/bfmc_signs_best.pt
```

## Installation and build

The virtual environment must use system site packages so it can import ROS Python modules:

```bash
cd ~/BFMC_2027/Awareness_UniPD-DriveOps
python3 -m venv --system-site-packages .awareness
source .awareness/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt

source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Keep `.awareness` active when launching the node. The `setuptools` warning about an unbuilt `pytest-repeat` egg is harmless when colcon finishes successfully.

## Running

```bash
cd ~/BFMC_2027/Awareness_UniPD-DriveOps
source /opt/ros/jazzy/setup.bash
source .awareness/bin/activate
source install/setup.bash
ros2 launch bfmc_sign_detection awareness.launch.py
```

Direct execution with the config file:

```bash
ros2 run bfmc_sign_detection road_sign_detector \
  --ros-args --params-file src/bfmc_sign_detection/config/awareness.yaml
```

## Topics

### Inputs

| Topic | Type | Purpose |
|---|---|---|
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | RGB image for YOLO |
| `/oak/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB calibration |
| `/oak/stereo/image_raw` | `sensor_msgs/msg/Image` | RGB-registered depth |
| `/oak/stereo/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsics |

The depth stream must be aligned/registered to the RGB image. If it is an unregistered left/right stereo image, RGB bounding-box coordinates cannot be sampled directly.

### Outputs

| Topic | Type | Purpose |
|---|---|---|
| `/traffic/detection` | `std_msgs/msg/String` | Detection JSON for localization |
| `/traffic/detection/debug` | `sensor_msgs/msg/Image` | Optional annotated RGB image |

Debug output is disabled by default.

Example detection:

```json
{"sign":"stop","distance_m":2.14,"confidence":0.91,"range_source":"stereo","bbox":[812.0,176.0,864.0,241.0],"stamp":{"sec":1788226981,"nanosec":400662817}}
```

The localization consumer requires `sign`, `distance_m`, and `confidence`. The other fields are diagnostic.

Inspect the system with:

```bash
ros2 topic list
ros2 topic echo /traffic/detection
ros2 topic hz /traffic/detection
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
```

## Parameters

Configuration file: `src/bfmc_sign_detection/config/awareness.yaml`.

| Parameter | Default | Description |
|---|---:|---|
| `image_topic` | `/oak/rgb/image_raw` | RGB input |
| `camera_info_topic` | `/oak/rgb/camera_info` | RGB calibration |
| `depth_topic` | `/oak/stereo/image_raw` | Registered depth |
| `depth_camera_info_topic` | `/oak/stereo/camera_info` | Depth calibration |
| `detection_topic` | `/traffic/detection` | JSON output |
| `model_path` | Installed model | YOLO weights override |
| `confidence` | `0.50` | Minimum detection confidence |
| `inference_size` | `1024` | YOLO image size |
| `sign_height_m` | `0.30` | Assumed height for fallback range |
| `focal_length_px` | `700.0` | Initial/fallback focal length |
| `depth_timeout_s` | `0.15` | Max RGB-depth timestamp difference |
| `depth_scale` | `0.001` | Integer depth scale, normally mm→m |
| `publish_debug_image` | `false` | Enable annotated output |
| `debug_image_topic` | `/traffic/detection/debug` | Debug output topic |

For `32FC1` depth, values are interpreted as metres. For `16UC1`, `depth_scale: 0.001` converts millimetres to metres.

## Stereo backprojection

For depth `Z` at RGB-aligned pixel `(u,v)` and intrinsics `fx, fy, cx, cy`:

```
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
range = sqrt(X² + Y² + Z²)
```

The detector samples the inner 50% of the bounding box, discards NaN/zero values and values outside 0.05–50 m, then uses the median depth. The RGB and depth timestamps must differ by no more than `depth_timeout_s`.

## Troubleshooting

### `No module named 'ultralytics'`

Activate the venv and install dependencies:

```bash
source ~/BFMC_2027/Awareness_UniPD-DriveOps/.awareness/bin/activate
python -m pip install -r ~/BFMC_2027/Awareness_UniPD-DriveOps/requirements.txt
source install/setup.bash
```

### No detections

Verify that input topics exist and publish images:

```bash
ros2 topic list | grep -E 'oak|traffic'
ros2 topic hz /oak/rgb/image_raw
ros2 topic hz /oak/stereo/image_raw
ls -lh src/bfmc_sign_detection/models/bfmc_signs_best.pt
```

### `range_source: bbox`

Depth was unavailable or stale. Check the depth topic, timestamp synchronization, encoding, registration, and depth `CameraInfo`.

### Incorrect distances

Confirm that the depth image is RGB-registered and that its `CameraInfo` belongs to the same depth stream. Confirm `16UC1` versus `32FC1` and adjust `depth_scale` if needed.

### Slow inference

The supplied model is relatively large. Reduce `inference_size`, use a smaller/exported model, or use the target GPU. The implementation currently requests Ultralytics device `0`, intended for the GPU-enabled target setup.

## Relationship with localization

Awareness publishes perception results; it does not perform map matching or pose estimation. The localization stack can consume `/traffic/detection` and associate the detected sign with its map position.

Source both workspaces when running them together:

```bash
source /opt/ros/jazzy/setup.bash
source ~/BFMC_2027/Localization_UniPD-DriveOps/install/setup.bash
source ~/BFMC_2027/Awareness_UniPD-DriveOps/.awareness/bin/activate
source ~/BFMC_2027/Awareness_UniPD-DriveOps/install/setup.bash
```

The awareness repository is independent and must not be placed inside the localization repository.

## Adding future detectors

Add new nodes under the same package, expose them as console scripts in `setup.py`, add parameters to `config/awareness.yaml`, and add launch actions to `launch/awareness.launch.py`. Suggested nodes:

```
stop_line_detector.py
lane_detector.py
obstacle_detector.py
traffic_light_detector.py
```

If a detector later needs separate dependencies or a separate release cycle, it can be split into another package while remaining in this repository.
