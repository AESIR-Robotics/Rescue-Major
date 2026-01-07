# Teleoperation 

This document describes how to run the teleoperation components for the GUI and the ROS-side services used by the Rescue-Major teleoperation stack. It covers required prerequisites (client and server), bootstrap commands, and short troubleshooting notes.

## Overview

- The web GUI connects to ROS using `rosbridge` (WebSocket) via `roslibjs`.
- Motor commands are published to `/dc_motors` (std_msgs/Float32MultiArray).
- WebRTC commands and controls are exchanged via `/web_rtc_commands` or an equivalent service.
- Key/button mappings are loaded from `static/keys_map_keyboard.json` and `static/keys_map_controller.json`.

## Prerequisites

### Client (browser)
- Modern browser with WebRTC support (Chrome, Firefox, Edge). Ensure access to camera/microphone if needed.
- If serving the GUI over plain HTTP (insecure), the browser must allow the required connections. For production, serve the GUI over HTTPS to avoid mixed-content and get full WebRTC behavior.

### Server (ROS)
- ROS 2 distribution used by this project: ROS 2 Humble (or compatible).
- rosbridge suite installed and running (provides the WebSocket endpoint).

## How to run

1. runn rosbridge server (rosbridge_server)

```bash
# Run in a shell with ROS 2 Humble sourced
ros2 run rosbridge_server rosbridge_websocket --port 9090
```

2. run webrtc server 

```bash
# Run in a shell with ROS 2 Humble sourced
python3 workspace/src/teleoperation/scripts/server.py
```
4. Open the client in a browser at : host:8081

