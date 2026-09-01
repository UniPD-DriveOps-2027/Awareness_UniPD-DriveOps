# bfmc_sign_detection

Umbrella ROS 2 package for vehicle-awareness perception nodes.

Current node:

- `road_sign_detector`: YOLO road-sign detection on `/oak/rgb/image_raw`,
  registered stereo-depth backprojection, and JSON detections on
  `/traffic/detection`.

Planned nodes such as stop-line and lane detection should be added under the
same Python package and exposed as additional console scripts and launch
entries. Start the current awareness stack with:

```bash
ros2 launch bfmc_sign_detection awareness.launch.py
```
