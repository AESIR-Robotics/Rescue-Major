# Vision Package

## ROS interface

###  Topics

| Topic                 | Type                                  | Description                                              |
| --------------------- | ------------------------------------- | -------------------------------------------------------- |
| cam_ram/sensors       | **sensor_msgs/msg/Image**  | Raw feed from the selected camera sent to the detection node.       |
| cam_sensors/image     | **sensor_msgs/msg/Image**  | Processing image from multi-detection node.                         |
| cam[N]/image_raw      | **sensor_msgs/msg/Image**  | Individual raw streams for each connected camera (N = index).       |
| cam_thermal/image_raw | **sensor_msgs/msg/Image**  | Raw image from thermal camera.                                      |

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
Already configured in the **.yaml** file inside the package:

- **device** (string): Defines the hardware platform you are running on (e.g., "jetson", "generic", "raspberry"). This dictates which pipeline templates or capture methods to use.

- **camera_devices** (int_array): List of **/dev/videoX** indices to open.

- **camera_devices** (int_array): List of /dev/videoX indices or sensor-id numbers to open.

- **camera_formats** (string_array): The specific template format to apply to each camera in the array (e.g., ["MJPG", "CSI", "YUYV"]).

- **camera_widths** (int_array): The desired horizontal resolution in pixels for each camera (e.g., [1920, 640]).

- **camera_heights** (int_array): The desired vertical resolution in pixels for each camera (e.g., [1080, 480]).

- **camera_fps** (int_array): The targeted frames per second for each camera (e.g., [30, 30]).

- **thermal_camera_index** (int): The specific hardware index assigned to the thermal sensor. Set to -1 if no thermal camera is connected.

- **thermal_enable** (bool): Set to true to enable specific processing or topic publishing for the thermal camera.

- **thermal_camera_index** (int): Index specifically assigned to a thermal sensor.

- **pipeline_templates.[FORMAT]** (string): Dynamic GStreamer templates used when device is set to "jetson" or when using GStreamer on a PC. These use {dev}, {w}, {h}, and {fps} as placeholders.

    -  *pipeline_templates.MJPG*: Template for hardware-accelerated Motion JPEG decoding.

    - *pipeline_templates.CSI*: Template for MIPI ribbon cameras using nvarguscamerasrc.

    - *pipeline_templates.H264*: Template for hardware-accelerated H.264 decoding.

    - *pipeline_templates.YUYV*: Template for raw, uncompressed video streams.

## RUN

Cameras.
```sh
ros2 run vision video_stream_publisher
```
Node of vision sensors.
```sh
ros2 run vision sensors_master.py
```
Launch
```sh
ros2 launch vision vision.launch.py
```

Examples

```sh
ros2 run vision video_stream_publisher --ros-args -p camera_devices:="[0, 2]"

ros2 service call /command_vision_sensors vision/srv/Command "{data: 'vision:mode,2'}"

ros2 service call /command_vision_sensors vision/srv/Command "{data: 'vision:threshold,50'}"
```