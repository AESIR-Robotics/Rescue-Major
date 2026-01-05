# Vision Package

## ROS interface

###  Topics

| Topic               | Type                                  | Description                                              |
| ------------------- | ------------------------------------- | -------------------------------------------------------- |
| cam_ram/sensors     | **sensor_msgs/msg/Image**  | Raw feed from the selected camera sent to the detection node.       |
| cam_sensors/image   | **sensor_msgs/msg/Image**  | Processing image from multi-detection node.                         |
| cam[N]/image_raw    | **sensor_msgs/msg/Image**  | Individual raw streams for each connected camera (N = index).       |

### Services
ros2 service call /command_vision_sensors vision/srv/Command "{data: 'vision:mode,2'}"
| Services                   | Type                    | Description                                                           |
| ---------------------------| ----------------------- | --------------------------------------------------------------------- |
| command_vision_sensors     | **vision/srv/Command**  | Control for modes and thresholds.                                     |
| command_vision_videoStream | **vision/srv/Command**  | Control for thermal and selected camera sent to the detection node.   |

### Commands Syntax

The nodes listen to the **commands_vision** topic. Send strings in the following formats:

Switch Camera Input: **"vision:camera,[index]"** (e.g., vision:num_cam,0).

Switch thermal **"vision:thermal,[state]"** (e.g., vision:thermal,on).

Change Detection Mode: **"vision:mode,[mode_index]"**.

- **0**: None (Passthrough).

- **1**: QR Code Detection.

- **2**: Hazmat Detection (YOLO).

- **3**: Motion Detection

Adjust Motion Sensitivity: **"vision:threshold,[0-255]"**.

### Parameters
- **enable_jetson** (bool): Set to **true** to use NVIDIA hardware-accelerated GStreamer pipelines.
- **camera_devices** (int_array): List of **/dev/videoX** indices to open.
- **thermal_camera_index** (int): Index specifically assigned to a thermal sensor.

Cameras.
```sh
ros2 run vision video_stream_publisher
```
Node of vision sensors.
```sh
ros2 run vision sensors_master.py
```

Examples

```sh
ros2 run vision video_stream_publisher --ros-args -p camera_devices:="[0, 2]"

ros2 service call /command_vision_sensors vision/srv/Command "{data: 'vision:mode,2'}"

ros2 service call /command_vision_sensors vision/srv/Command "{data: 'vision:threshold,50'}"
```