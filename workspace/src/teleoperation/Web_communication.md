# ROSbridge - Web commounicommunication  

This document explains how the GUI communicates with ROS via rosbridge (using `roslibjs`), which topics/services/parameters are present in the current codebase, and how to extend the system by adding new key mappings, commands, and communication methods.

Scope: applies to `workspace/src/teleoperation/GUI/com_client.js` and how keymaps are loaded from `static/keys_map_*.json`.

Quick summary
- Topics used: `/dc_motors` (publisher), `/web_rtc_commands` (publisher), `/client_info_dummy` (subscriber) — see details below.
- Services: optional support to call a WebRTC service if available (service name/type depends on deployment).
- Parameters: `/vision_camera_modes` (param) — readable/writable via bindings.
- Bindings: loaded from JSON (`static/keys_map_keyboard.json` and `/static/keys_map_controller.json`) and determine what each key/button does.

1. Connection and lifecycle
- The `RobotComms` class creates a `ROSLIB.Ros` instance connected to `ws://<host>:<port>` (defaults to `location.hostname:9090`).
- Handled events: `connection`, `close`, `error`.
- On reconnect, topics/services/params are (re)initialized; if the server is not available the GUI will retry connecting (see `_connect`).

2. Topics and types present (current code)
- `this.publisher` → topic `/dc_motors`
  - Expected type in code: `std_msgs/Float32MultiArray`.
  - Usage: publish two floats `[x, y]` from the `InputHandler` (keyboard/gamepad). Example message: `{ data: [0.0, 1.0] }`.
- `this.webrtcPub` → topic `/web_rtc_commands`
  - Type: `std_msgs/String`.
  - Usage: send textual commands related to WebRTC (e.g. `toggle_client_audio`). Example: `{ data: "toggle_client_audio" }`.
- `this.infoSub` → topic `/client_info_dummy`
  - Type: `std_msgs/String`.
  - Usage: the GUI subscribes to display informational messages sent by the server/robot.

3. Parameters and services
- Parameter `/vision_camera_modes` (`ROSLIB.Param`) — used by bindings of mode `param` to `get` or `set`.
  - `get` (without value) calls `param.get(callback)`.
  - `set` (with value) calls `param.set(value)`.
- Services: in `_prepareBindings()` bindings of mode `service` create a `ROSLIB.Service` with default `serviceType` `std_srvs/Trigger`.
  - In `sendMappedKey`, if a binding is `service`, the code calls `svc.callService(new ROSLIB.ServiceRequest(req), callback)`.
  - Note: services used by the keymap may require the correct service type and request structure; the code assumes `std_srvs/Trigger` unless extended.

4. Keymap structure and JSON schema
- Keymaps are JSON objects using the format: `"<key>": [ <mode>, <target>, <optional value> ]`.
- `mode` can be:
  - `"topic"` → `target` is the topic name (e.g. `/dc_motors`). `value` is the payload or a template. If `value` is an array the code infers `std_msgs/Float32MultiArray`.
  - `"service"` → `target` is the service name (e.g. `/reset_odometry`). `value` is an optional request payload.
  - `"param"` → `target` is the parameter name (e.g. `/vision_camera_modes`). If `value` is present, a `set` is performed; otherwise a `get` is performed.
- Examples:
  - Map key `c` to send a WebRTC command:
    ```json
    { "c": ["topic", "/web_rtc_commands", "toggle_client_audio"] }
    ```
  - Map `space` to publish stop command to motors (two floats):
    ```json
    { " ": ["topic", "/dc_motors", [0.0, 0.0]] }
    ```
  - Call a service with payload:
    ```json
    { "r": ["service", "/restart_system", {"force": true}] }
    ```
  - Set vision parameter:
    ```json
    { "1": ["param", "/vision_camera_modes", 1] }
    ```

5. How to add more keys/commands (practical steps)
- 1) Edit or create the JSON file `static/keys_map_keyboard.json` (or `static/keys_map_controller.json`).
  - Add entries using the structure shown above.
- 2) Ensure the topic/service/parameter exists on the ROS side (names and types must match).
  - For topics: decide the message type. Current code infers `std_msgs/Float32MultiArray` when `value` is an array, else `std_msgs/String` by default.
  - For services: if the service type is not `std_srvs/Trigger`, extend `_prepareBindings()` to create the `ROSLIB.Service` with the correct `serviceType`.
- 3) Reload the GUI (or implement dynamic reload) so `RobotComms` re-reads the keymaps.

6. Extending the code for new types and safety
- Complex messages: to publish messages that are not `std_msgs/String` or `Float32MultiArray`, modify `_prepareBindings()` to accept a fourth field `msgType` in the JSON: `["topic","/foo","payload","my_msgs/ComplexType"]`.
- Services with different types: allow the JSON to include `serviceType`. Example: `["service", "/my_svc", {"foo":1}, "my_pkg/MySrv"]` and construct `new ROSLIB.Service(..., serviceType: 'my_pkg/MySrv')`.
- Validation: add validation when loading JSON and emit `onInfoMessage` or `console.warn` if a binding is invalid.
- Safety: avoid executing destructive actions without confirmation; consider a `requireConfirm` flag in the JSON for services or params that are sensitive.

7. Operational and UX recommendations
- Do not rely on hardcoded fallbacks: prefer serving keymaps from the server and let administrators manage bindings centrally.
- Expose an `/api/keymaps` endpoint to manage bindings through a UI.
- Add `RobotComms.ready` or an `on('ready')` event to notify when `bindings` and publishers are ready (avoid polling in `main`).
- Normalize axis conventions: document if negative `y` equals forward or the opposite, and offer an `invertY` option in `InputHandler`.
- Configurable logging: add `DEBUG`/`INFO` levels to avoid spam in the UI.

8. Full example keymap for keyboard
```json
{
  "w": ["topic", "/dc_motors", [0.0, -1.0]],
  "s": ["topic", "/dc_motors", [0.0, 1.0]],
  "a": ["topic", "/dc_motors", [-1.0, 0.0]],
  "d": ["topic", "/dc_motors", [1.0, 0.0]],
  " ": ["topic", "/dc_motors", [0.0, 0.0]],
  "c": ["topic", "/web_rtc_commands", "toggle_client_audio"],
  "1": ["param", "/vision_camera_modes", 1],
  "r": ["service", "/restart_system", {"force": true}, "my_pkg/Restart"]
}
```