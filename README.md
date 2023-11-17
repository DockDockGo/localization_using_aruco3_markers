# Instructions to Launch fiducial marker detection for FVD

### Build realsense camera node:
```
ros2 build --packages-select realsense_camera
```

### Build marker detection:
```
ros2 build --packages-select aruco_ros fiducial_marker_fvd
```

------

### Launch realsense 2 camera node in a terminal:
```
ros2 launch realsense2_camera rs_launch.py
```
Ensure that the message `Realsense Node is Up!` message is published before launching marker detection.

### Launch aruco_ros node
```
ros2 launch aruco_ros marker_publisher.launch.py
```

### Launch fvd node to broadcast simplified detection measurements to use for docking
```
ros2 run fiducial_marker_fvd fiducial_marker_fvd
```
In order to disable the fvd node GUI, set ENABLE_GUI (https://github.com/DockDockGo/localization_using_aruco3_markers/blob/9397909dfe5d1fa1543b068d90c3cdd8fc425152/fiducial_marker_fvd/fiducial_marker_fvd/fiducial_marker_fvd.py#L19) to False. 

### In order view fiducial marker detections overlaid on image:
Launch rqt image view and view the fiducial markers topic ending in `result` or `image_result`
