# Vision Package

## ROS interface

### Subscribed Topics

| Topic               | Type                                  | Description                                              |
| ------------------- | ------------------------------------- | -------------------------------------------------------- |
| cam_ram/sensors     | **sensor_msgs/msg/Image**  | Raw feed from the selected camera sent to the detection no          |
| comands_vision      | **std_msgs/msg/String**    | Control topic for switching modes and cameras.                      |

### Published Topics

| Topic               | Type                       | Description                                                         |
| ------------------- | -------------------------- | ------------------------------------------------------------------- |
| cam[N]/image_raw    | **sensor_msgs/msg/Image**  | Individual raw streams for each connected camera (N = index).       |
| cam/sensors         | **sensor_msgs/msg/Image**  | The processed video stream (with bounding boxes/drawings).          |

### Commands Syntax

The nodes listen to the **commands_vision** topic. Send strings in the following formats:

Switch Camera Input: **"vision:num_cam,[index]"** (e.g., vision:num_cam,0).

Change Detection Mode: **"vision:mode,[mode_index]"**.

- **0**: None (Passthrough).

- **1**: QR Code Detection.

- **2**: Hazmat Detection (YOLO).

- **3**: Motion Detection

Adjust Motion Sensitivity: **"vision:threshold,[0-255]"**.

### Parameters
- **enable_jetson** (bool): Set to **true** to use NVIDIA hardware-accelerated GStreamer pipelines.
- **camera_devices** (int_array): List of **/dev/videoX** indices to open.

Cameras.
```sh
ros2 run vision video_stream_publisher
```
Node of vision sensors.
```sh
ros2 run vision sensors_master.py
```