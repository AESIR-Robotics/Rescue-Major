# ROSbridge - Web Communication  

This document explains how the GUI communicates with ROS via rosbridge (using `roslibjs`), the architecture of the current codebase, and how to extend the system by adding new key mappings and commands.

Scope: applies to `workspace/src/teleoperation/GUI/com_client.js` and how keymaps are loaded from `static/keys_map_keyboard.json`.

## Quick Summary
- **Main classes**: `RobotAPI` (manages ROS topics/services), `InputHandler` (keyboard/gamepad input)
- **Primary topic**: `/dc_motors` (std_msgs/Float32MultiArray) - publishes joystick/keyboard velocities
- **Keymaps**: loaded from `static/keys_map_keyboard.json` - defines key-to-action mappings
- **Control flow**: Start/Stop button enables/disables continuous joystick velocity publishing at 10Hz

## 1. Architecture Overview

### RobotAPI Class
The `RobotAPI` class is a centralized manager for ROS communication that:
- Takes a `ROSLIB.Ros` instance in its constructor
- Implements caching of topics and services to avoid redundant connections
- Maps keys/buttons to ROS actions (topics or services) using JSON configuration
- Provides an `executeAction(keyId)` method to trigger mapped actions

**Key features**:
- **Topic cache**: `topicCache` stores reusable `ROSLIB.Topic` objects
- **Service cache**: `serviceCache` stores reusable `ROSLIB.Service` objects  
- **Action mapping**: `actionMap` associates key IDs with their ROS objects and payloads
- **Pre-caching**: Topics/services are created once during initialization, not on every execution

### InputHandler Class
The `InputHandler` class manages user input:
- Listens to keyboard events (`keydown`, `keyup`)
- Polls gamepad state via `requestAnimationFrame` 
- Applies deadzone filtering to analog stick values (default: 0.12)
- Emits events: `buttonDown`, `buttonUp`, `axis`, `gamepadConnected`, `gamepadDisconnected`
- Provides `readJoystick()` method returning `{x, y, active}` from left analog stick

### Connection and Lifecycle
- A `ROSLIB.Ros` instance connects to `ws://<host>:<port>` (defaults to `location.hostname:9090`)
- On connection, `RobotAPI` is initialized with keyboard keymaps from `static/keys_map_keyboard.json`
- Handled events: `connection`, `close`, `error`
- Emergency stop command `[0.0, 0.0]` is sent on page unload

## 2. ROS Topics and Services

### Action-Mapped Topics and Services
Topics and services can be dynamically mapped to keyboard keys through the JSON configuration. The `RobotAPI` supports:

**Topic actions** (`type: "topic"`):
- Creates/reuses a `ROSLIB.Topic` with specified `target` name and `msg_type`
- Publishes the configured `payload` when the key is pressed
- Default message type: `std_msgs/String`

## 3. Keymap Structure and JSON Schema

Keymaps are loaded from `static/keys_map_keyboard.json` and define the mapping between keyboard keys and ROS actions.

### JSON Format
Each key in the JSON object maps to an action configuration:
```json
{
  "key_id": {
    "type": "topic" | "service",
    "target": "/topic_or_service_name",
    "msg_type": "pkg/MsgType",     // for topics (optional)
    "srv_type": "pkg/SrvType",     // for services (optional)
    "payload": { /* message data */ }
  }
}
```

### Field Descriptions
- **key_id**: The keyboard key (lowercase, e.g., `"w"`, `" "` for space)
- **type**: Either `"topic"` or `"service"`
- **target**: The ROS topic or service name (e.g., `"/dc_motors"`)
- **msg_type**: (Optional) Message type for topics (default: `"std_msgs/String"`)
- **srv_type**: (Optional) Service type for services (default: `"std_srvs/Trigger"`)
- **payload**: The message data or service request to send

### Runtime Operation
**Keyboard input**:
- `keydown` events trigger `robotAPI.executeAction(key)`
- Mapped actions are executed immediately (topic publish or service call)
- Unmapped keys are silently ignored

**Gamepad input**:
- Continuous polling at ~60Hz via `requestAnimationFrame`
- Joystick values filtered through deadzone (0.12)
- When control is enabled, velocities published at 10Hz to `/dc_motors`

**Start/Stop Control**:
- Button click toggles `isControlEnabled` flag
- When enabled: starts 10Hz interval for joystick velocity publishing
- When disabled: clears interval and sends stop command `[0.0, 0.0]`

### Safety Features
- Emergency stop on page unload/refresh
- Stop command sent when control is disabled
- Deadzone filtering prevents drift from neutral joystick position
