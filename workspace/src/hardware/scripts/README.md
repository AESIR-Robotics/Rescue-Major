# Dual-Mode Keyboard Teleoperation for ROS 2

A professional, SSH-friendly keyboard teleoperation node with dual control modes for robotics applications. Features deadman switches, tmux integration, and real-time visual feedback.

## Table of Contents

- [Features](#-features)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
- [Control Modes](#-control-modes)
  - [Joint Control Mode](#1-joint-control-mode)
  - [Twist Control Mode](#2-twist-control-mode)
- [Safety Features](#-safety-features)
- [Configuration](#-configuration)
- [Troubleshooting](#-troubleshooting)
- [Architecture](#-architecture)

---

## Features

### Core Capabilities
- **Dual Control Modes**: Switch seamlessly between joint-level and differential drive control
- **SSH-Compatible**: Works flawlessly over SSH using raw terminal mode
- **Tmux Integration**: Automatic deadman switch when detaching from tmux sessions
- **Real-Time Feedback**: Live HUD with velocity, position, and control state visualization
- **Safety First**: Multiple deadman switches and automatic timeout on inactivity

### Advanced Features
- **Velocity Inversion Braking**: Press opposite direction to instantly stop
- **Continuous Republishing**: Maintains commanded velocities at configurable rate (10 Hz)
- **Hold vs Constant Modes**:
  - **Hold Mode**: Velocities applied only while key is pressed (position = -1)
  - **Constant Mode**: Persistent velocity commands with position tracking
- **Dynamic Speed Adjustment**: Change velocities on-the-fly without stopping
- **Session Monitoring**: Detects SSH disconnections and tmux detachments

---

## Requirements

### System Dependencies
```bash
# ROS 2 (Humble/Iron/Jazzy)
sudo apt install ros-<distro>-rclpy ros-<distro>-std-msgs ros-<distro>-geometry-msgs

# Python 3.8+
python3 --version

# Tmux (optional, for deadman features)
sudo apt install tmux
```

### Custom Messages
Requires the `hardware` package with the `JointControl` message definition:

```msg
# hardware/msg/JointControl.msg
std_msgs/Header header
string[] joint_names
float64[] position
float64[] velocity
float64[] acceleration
float64[] effort
```

---

## Installation

1. **Clone into your ROS 2 workspace:**
   ```bash
   cd ~/ros2_ws/src/hardware/scripts/
   wget https://your-repo/joint_mux.py
   chmod +x joint_mux.py
   ```

2. **Build your workspace:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select hardware
   source install/setup.bash
   ```

3. **Verify installation:**
   ```bash
   ros2 run hardware joint_mux.py --ros-args -h
   ```

---

## Usage

### Basic Launch
```bash
ros2 run hardware joint_mux.py
```

### With Custom Parameters
```bash
ros2 run hardware joint_mux.py --ros-args \
  -p joint_topic:=/my_robot/joint_command \
  -p twist_topic:=/my_robot/cmd_vel \
  -p joint_names:="['joint_1','joint_2','joint_3']"
```

### Inside Tmux (Recommended for SSH)
```bash
tmux new -s teleop
ros2 run hardware joint_mux.py
# Press Ctrl+B, then D to detach robot stops automatically
```

---

## Control Modes

### [1] Joint Control Mode

Controls individual joints with position or velocity commands.

#### **Interface Layout**
```
╔══════════════════════════════════════════════════════╗
║         [1] JOINT CONTROL MODE                       ║
╚══════════════════════════════════════════════════════╝
  Topic : hardware_node/joint_command
  Step: 0.1000 rad   Speed: 0.5000 rad/s ◐   Mode: HOLD

  Joint                Inc  Dec   Position    Vel
  ──────────────────────────────────────────────────────
  flipper_0            [q]  [a]     (hold)    +0.500
  flipper_1            [w]  [s]     (hold)    +0.000
  ...
```

#### **Keybindings**

| Key | Function | Description |
|-----|----------|-------------|
| `q/a`, `w/s`, `e/d`, `r/f`, `t/g`, `y/h` | Joint control | Increment/decrement individual joints |
| `↑` (or `+`) | Speed up | Increase global velocity by 0.1 rad/s |
| `↓` (or `-`) | Slow down | Decrease global velocity by 0.1 rad/s |
| `v` | Toggle mode | Switch between HOLD ↔ CONSTANT |
| `z` | Reset | Zero all positions/velocities |
| `x` | Emergency stop | Stop all motion immediately |
| `2` | Switch mode | Go to Twist Control Mode |

#### **Operating Modes**

**HOLD Mode (Default)**
- `position = -1.0` (no target position)
- Velocity applied **only while key is pressed**
- Ideal for manual fine-tuning and exploration
- Safer for teleoperation

**CONSTANT Mode**
- Position tracking enabled (`0 to 2π`)
- Velocity persists after key release
- Position increments by `step` parameter
- Better for choreographed movements

#### **Velocity Inversion Behavior**
- Pressing the **opposite direction key** → **instant stop**
- Example: Joint moving forward (`w`), press `s` → velocity = 0
- Works in both HOLD and CONSTANT modes

---

### [2] Twist Control Mode

Differential drive control using standard Twist messages (geometry_msgs).

#### **Interface Layout**
```
╔══════════════════════════════════════════════════════╗
║         [2] TWIST CONTROL MODE                       ║
╚══════════════════════════════════════════════════════╝
  Topic : hardware_node/cmd_vel
  Linear  : +0.500 m/s   (max: 0.50)
  Angular : +0.000 rad/s (max: 1.00)

  Movement:
        [u]    [i]    [o]
         ↰      ↑      ↱

        [j]    [k]    [l]
         ←    STOP    →

        [m]    [,]    [.]
         ↲      ↓      ↳
```

#### **Keybindings**

| Key | Movement | Linear | Angular |
|-----|----------|--------|---------|
| `i` | Forward | +1.0 | 0.0 |
| `k` | **STOP** | 0.0 | 0.0 |
| `,` | Backward | -1.0 | 0.0 |
| `j` | Turn left | 0.0 | +1.0 |
| `l` | Turn right | 0.0 | -1.0 |
| `u` | Forward-left | +1.0 | +1.0 |
| `o` | Forward-right | +1.0 | -1.0 |
| `m` | Backward-left | -1.0 | -1.0 |
| `.` | Backward-right | -1.0 | +1.0 |

**Speed Adjustments:**

| Key | Function | Multiplier |
|-----|----------|------------|
| `q` | Increase linear | ×1.1 |
| `z` | Decrease linear | ×0.9 |
| `w` | Increase angular | ×1.1 |
| `x` | Decrease angular | ×0.9 |
| `1` | Switch to Joint mode | — |

---

## Safety Features

### 1. **Connection Deadman Switch** (Tmux Integration)
- Monitors tmux client connections every 0.5 seconds
- **Detachment detection**: Stops all motors when you detach from tmux
- **Reattachment**: Resumes control when you reattach
- Debug log shows: `"User de-attached from tmux"`

```bash
# Example workflow:
tmux attach -t teleop
# Robot is moving...
# Press Ctrl+B, D  → Detach
# Robot stops automatically
tmux attach -t teleop
# Resume control
```

### 2. **Inactivity Deadman Switch**
- Triggers after **60 seconds** of no keyboard input
- Stops all interfaces automatically
- Debug log shows: `"Inactivity detected, stopping everything"`
- Reset by pressing any key

### 3. **SIGHUP Handler**
- Catches terminal hangup signals (SSH disconnections)
- Gracefully stops all motion before shutdown
- Works outside tmux environments

### 4. **Graceful Shutdown**
- `Ctrl+C` or `ESC` → Clean exit
- Restores terminal settings
- Publishes final stop command

---

## Configuration

### ROS 2 Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joint_topic` | string | `hardware_node/joint_command` | JointControl publisher topic |
| `twist_topic` | string | `hardware_node/cmd_vel` | Twist publisher topic |
| `joint_names` | string[] | `['flipper_0', ..., 'flipper_5']` | List of joint names |

### Internal Constants (modify in code)

```python
# JointControlInterface
self.step = 0.1          # Position increment per keypress (rad)
self.speed = 0.5         # Initial velocity (rad/s)
self.speed_step = 0.1    # Velocity adjustment delta
self.min_speed = 0.0     # Minimum allowed velocity
self.max_speed = 2.0     # Maximum allowed velocity
self.republish_rate = 10.0  # Hz

# TwistControlInterface
self.linear_speed = 0.5   # m/s
self.angular_speed = 1.0  # rad/s
self.max_linear = 2.0
self.max_angular = 3.0

# Deadman Switches
self.tmux_check_interval_ = 0.5  # seconds
INACTIVITY_TIMEOUT = 60.0        # seconds
```

---

## Troubleshooting

### Problem: Interface text overlaps / buffering issues in tmux
**Solution**: The code uses full screen clear (`CLEAR`) before each render. If you still see issues:
```bash
# In your tmux session:
tmux set-option -g default-terminal "screen-256color"
# Or force UTF-8:
export LANG=en_US.UTF-8
```

### Problem: Keys not responding
**Causes**:
1. Another process has captured stdin
2. Terminal not in raw mode
3. Running without a proper TTY

**Solutions**:
```bash
# Check if stdin is a TTY:
python3 -c "import sys; print(sys.stdin.isatty())"  # Should print True

# Run with explicit TTY allocation over SSH:
ssh -t user@robot "ros2 run hardware joint_mux.py"
```

### Problem: Deadman not triggering on tmux detach
**Debug**:
```bash
# Check tmux environment variable:
echo $TMUX  # Should show socket path

# Test tmux client detection:
tmux list-clients
```

The debug log at the bottom of the interface shows real-time deadman status.

### Problem: Position wrapping unexpectedly
- Positions are wrapped to `[0, 2π]` using modulo in CONSTANT mode
- In HOLD mode, position is always `-1` (no target)
- Use `v` key to toggle between modes

---

## Architecture

### Class Hierarchy

```
TeleopInterface (ABC)
    ├── JointControlInterface
    │   ├── Key mapping system
    │   ├── Velocity/position state
    │   ├── Continuous republishing
    │   └── Mode switching (hold/constant)
    │
    └── TwistControlInterface
        ├── Movement bindings
        ├── Speed multipliers
        └── Twist message publishing

DualTeleopNode (rclpy.Node)
    ├── Interface management
    ├── Keyboard input (KeyboardReader)
    ├── Deadman monitoring
    │   ├── Tmux client checker
    │   ├── Inactivity timer
    │   └── SIGHUP handler
    └── Rendering loop
```

### Message Flow

```
Keyboard Input
    ↓
KeyboardReader (raw terminal mode)
    ↓
DualTeleopNode.run()
    ├─→ Deadman checks
    ├─→ Interface routing
    └─→ current_interface.handle_key()
            ↓
        Interface state update
            ↓
        ROS 2 Publisher
            ↓
    JointControl / Twist message
```

### Deadman Logic

```python
# Connection deadman
if tmux_detached:
    stop_all_interfaces()
    
# Inactivity deadman
if time_since_last_key > 60s:
    stop_all_interfaces()

# SIGHUP deadman
signal(SIGHUP):
    stop_all_interfaces()
    exit()
```

---

## Message Specifications

### JointControl (Published by Joint Mode)
```yaml
header:
  stamp: <current_time>
  frame_id: "world"
joint_names: ["flipper_0", "flipper_1", ...]
position: [<rad>, ...]     # -1.0 in HOLD mode
velocity: [<rad/s>, ...]
acceleration: [20000.0, ...] # Fixed high value
effort: [0.0, ...]          # Not used
```

### Twist (Published by Twist Mode)
```yaml
linear:
  x: <m/s>
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: <rad/s>
```

---

## Visual Feedback

### Color Coding

| Color | Meaning |
|-------|---------|
| 🟢 **GREEN** | Active/positive velocity, increment keys |
| 🔴 **RED** | Negative velocity, decrement keys, active motion |
| 🟡 **YELLOW** | Current values, mode indicators |
| 🔵 **CYAN** | Headers, informational text |
| 🟣 **MAGENTA** | HOLD mode, special states |
| ⚪ **DIM** | Inactive/zero values |

### Status Indicators

- `●` (CONSTANT mode) - Position tracking active
- `◐` (HOLD mode) - Velocity-only control
- `◉` (RED) - Joint actively moving
- `○` (DIM) - Joint stopped
- `(hold)` - Position = -1 (no target)

---

## Testing

### Manual Test Procedure

1. **Start the node:**
   ```bash
   ros2 run hardware joint_mux.py
   ```

2. **Test Joint Mode:**
   - Press `q`  Should see `flipper_0` velocity increase
   - Press `a`  Should stop (velocity inversion)
   - Press `v`  Should toggle to CONSTANT mode
   - Press `q`  Should see position increment

3. **Test Twist Mode:**
   - Press `2`  Switch to twist interface
   - Press `i`  Forward motion
   - Press `k`  Stop
   - Press `j`  Rotate left

4. **Test Deadman:**
   - In tmux: `Ctrl+B, D` → Motors should stop
   - Wait 60s → Inactivity deadman should trigger

5. **Test Speed Adjustment:**
   - Press `+` Speed should increase
   - Verify with active joint velocity magnitude changes

---

## License

This software is part of the `hardware` ROS 2 package. Refer to the package license for terms of use.

---

## Support

For issues or feature requests, check:
- ROS 2 console output for errors
- Debug log in the interface (bottom of screen)
- Verify message publication: `ros2 topic echo /hardware_node/joint_command`

**Common Debug Commands:**
```bash
# Check if topics are being published:
ros2 topic hz /hardware_node/joint_command
ros2 topic hz /hardware_node/cmd_vel

# Monitor message content:
ros2 topic echo /hardware_node/joint_command

# Check node status:
ros2 node info /dual_teleop
```

---

**Happy Teleoperating!**