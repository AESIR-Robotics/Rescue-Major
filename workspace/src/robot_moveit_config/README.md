# AESIR MoveIt Config Package

This package provides the MoveIt 2 configuration and launch files for the **AESIR robotic robot** (`rescue_robot`), but it works for the arm. It features a complete hybrid control architecture, supporting collision-aware motion planning, real-time servo teleoperation, and low-level joint-by-joint control modes.

## Package Overview

* **Robot:** Custom 6-DOF robotic arm
* **Planning Group:** `arm`
* **End Effector:** `tool_link`
* **Base Frame:** `arm_base_link`
* **Joints:** `joint_1` through `joint_6`

## Dependencies

This package requires the following:
* ROS 2 (Humble / Jazzy)
* MoveIt 2
* `aesir_robot_description`
* `controller_manager`
* `moveit_servo`
* `ros2_controllers` (joint_trajectory, forward_command, position, velocity, effort)

List of dependences
```bash
sudo apt update
sudo apt install ros-humble-hardware-interface \
   ros-humble-pluginlib \
   ros-humble-xacro \
   ros-humble-moveit \
   ros-humble-moveit-servo \ 
   ros-humble-ros2-control \
   ros-humble-ros2-controllers \ 
   ros-humble-joy \
   ros-humble-forward-command-controller \
   ros-humble-teleop-twist-joy
```
---

## Building and Setup

1. Build the packages:
```bash
colcon build --packages-select robot_robot_description
colcon build --packages-select robot_moveit_config
```

## Launch Architecture

### 1. Basic MoveIt Demo

Launches only MoveIt and RViz with the Joint State Publisher GUI (Useful for testing URDF limits without Servo or controllers).

```bash
ros2 launch robot_moveit_config demo.launch.py
```

### 2. Bringup

Launches the complete infrastructure: MoveIt, RViz, inactive low-level controller, and MoveIt Servo (which starts automatically after a 3-second safety delay).

```bash
ros2 launch robot_moveit_config bringup.launch.py
```

### 3. Bringup with All Controllers

Launches everything including position, velocity, and effort controllers:

```bash
ros2 launch robot_moveit_config bringup_controllers.launch.py
```

### 4. Commander Node

Launches the arm command node for programmatic control:

```bash
ros2 launch robot_moveit_config commander.launch.py
```

This starts the `arm_command_node` executable that listens for:
- `/arm_command/pose_goal` (geometry_msgs/PoseStamped) - Cartesian goals
- `/arm_command/joint_goal` (sensor_msgs/JointState) - Joint space goals

### 5. Servo Teleoperation

Launches MoveIt Servo for real-time teleoperation:

```bash
ros2 launch robot_moveit_config servo_teleop.launch.py
```

## Control Modes & APIs

### MoveIt Commander (Planning-Based Control)

The Commander node provides collision-aware motion planning. It listens on dedicated topics and calculates safe trajectories.

#### Cartesian Pose Control
Send pose goals to `/arm_command/pose_goal`:

```bash
ros2 topic pub --once /your_pose_topic geometry_msgs/msg/PoseStamped "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.5, y: 0.0, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
```

#### Joint Space Control
Send joint goals to `/arm_command/joint_goal`:

```bash
ros2 topic pub --once /your_joint_topic sensor_msgs/msg/JointState "{name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'], position: [0.0, 0.5, 0.0, 0.0, 0.0, 0.0]}"
```

### Servo Teleoperation (Real-Time Control)

For real-time control, use MoveIt Servo. The servo node subscribes to `/servo_node/delta_twist_cmds` (geometry_msgs/TwistStamped).

#### Cartesian Keyboard Teleoperation

Drive the end-effector through 3D space.

```bash
ros2 run robot_moveit_config test_teleop.py
```

**Controls:**
- **Translation (End Effector):**
  - `w`/`s`: Forward/Back (+/- X)
  - `a`/`d`: Left/Right (+/- Y)
  - `q`/`e`: Up/Down (+/- Z)

- **Rotation (Wrist):**
  - `u`/`o`: Roll (+/-)
  - `i`/`k`: Pitch (+/-)
  - `j`/`l`: Yaw (+/-)

- **Other:**
  - `spacebar`: Stop
  - `Ctrl+C`: Exit

#### Joint Keyboard Teleoperation
Drive individual joints using velocities while keeping MoveIt's collision and singularity protection active.

```bash
ros2 run robot_moveit_config joint_keyboard.py
```
-  Joint 1: `q/a` : (+/-)
-  Joint 2: `w/s` : (+/-)
-  Joint 3: `e/d` : (+/-)
-  Joint 4: `r/f` : (+/-)
-  Joint 5: `t/g` : (+/-)
-  Joint 6: `y/h` : (+/-)

- Stop All: `Spacebar`

#### Programmatic Servo Control

Publish TwistStamped messages to `/servo_node/delta_twist_cmds`:

```bash
ros2 topic pub -r 10 /servo_node/delta_twist_cmds geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}}"
```

### Hardware Brigde: TopicBrigdeHardware
This package includes a custom `ros2_control` System Interface that acts as a bridge. Instead of talking to a specific driver, it packages MoveIt’s trajectory commands into a custom ROS 2 message and "screams" them over a topic for a Serial/Hardware node to consume.

#### Communication Interface

| Topic | Type | Direction | Descrition |
| :--- | :--- | :--- | :--- |
| `/comands_hardware` | `hardware/mgs/JointControl` | **OUT** | Contains position, velocity, acceleration, and effort for all 6 joints. |
| `harware_brigde/set_acceleration` | `std_msgs/msg/Float64MultiArray` | **IN** | Used to update joint acceleration limits at runtime. |


#### Setting Joint Acceleration
To update the acceleration for all 6 joints at once, publish a Float64MultiArray containing exactly 6 values (one per joint in radians/sec²):

```bash
ros2 topic pub --once /hardware_bridge/set_acceleration std_msgs/msg/Float64MultiArray "{data: [2.0, 2.0, 1.5, 1.5, 1.0, 1.0]}"
```

### Switching Between Control Modes

#### Pause Commander Node

By default, the arm_controller (MoveIt) owns the hardware. To bypass MoveIt completely and send raw signals to the motors, use the ROS 2 Control CLI to swap controllers.

**Note:** Only ONE controller can be active per joint at a time.

#### Direct Controller Control

Use position, effort, or velocity controllers for direct joint control:

1. **Raw Position Control:**
   ```bash
   ros2 control switch_controllers --activate position_controller --deactivate arm_controller
   ```

2. **Raw Effort (Torque) Control:**
   ```bash
   ros2 control switch_controllers --activate effort_controller --deactivate arm_controller
   ```
3. **Raw Velocity Control:**
   ```bash
   ros2 control switch_controllers --activate velocity_controller --deactivate arm_controller
   ```

4. **Restore MoveIt Control:**
   ```bash
   ros2 control switch_controllers --activate arm_controller --deactivate position_controller effort_controller velocity_controller
   ```

*Example Velocity Command:*
```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, 0.0, 0.0, 0.0, 0.0, 0.0]}" -r 10
```

## Debugging and Analysis

### Moveit Servo Alarms & Status Codes
MoveIt Servo broadcasts its internal safety state to `/servo_node/status`. You can monitor this topic to detect imminent collisions or singularities.

```bash
ros2 topic echo /status
```

| Code | Status | Description |
| :--- | :--- | :--- |
| 0 | 🟢 No warnings | All clear. Robot is operating normally. |
| 1 | 🟡 Approaching singularity | Decelerating. The arm is moving closer to a mathematical singularity. |
| 2 | 🔴 Halt for singularity | Emergency Stop. The robot is stuck mathematically. |
| 3 | 🟡 Leaving singularity | Decelerating. The arm is safely moving away from a singularity. |
| 4 | 🟡 Approaching collision | Decelerating. An obstacle has been detected nearby. |
| 5 | 🔴 Halt for collision | Emergency Stop. Imminent collision detected. |
| 6 | 🔴 Joint_bound | Emergency Stop. A motor has reached its physical position or velocity limit. |
| -1 | ❌ Invalid | Servo received an invalid or corrupt command. |

### General Diagnostics

#### Check active controllers
```bash
ros2 control list_controllers
```
#### Monitor Joint States & Effort:
```bash
ros2 topic echo /joint_states
```

### Servo parameters

Configured in `config/servo_params.yaml`:
- **Linear scale**: 0.4 (m/s)
- **Rotational scale**: 0.8 (rad/s)
- **Joint scale**: 0.5 (rad/s)

### Controllers

- **arm_controller**: Follows MoveIt planned trajectories
- **position_controller**: Direct position control (velocity depends on hardware)
- **effort_controller**: Direct torque control
- **velocity_controller**: Direct velocity control

## Configuration Files

- `config/custom_arm.urdf.xacro`: Robot description
- `config/custom_arm.srdf`: Semantic robot description
- `config/moveit_controllers.yaml`: MoveIt controller configuration
- `config/ros2_controllers.yaml`: ROS 2 controller configuration
- `config/servo_params.yaml`: Servo parameters
- `config/joint_limits.yaml`: Joint limits
- `config/kinematics.yaml`: Kinematics solver config

## Troubleshooting

### Common Issues

1. **Servo not responding**: Ensure servo node is started and `/servo_node/start_servo` service is called
2. **Controller conflicts**: Only one controller can be active per joint at a time
3. **Planning failures**: Check joint limits and collision environment

## Architecture

```
robot_moveit_config/
├── config/
│   ├── custom_arm.srdf                     # Semantic Robot Description
│   ├── kinematics.yaml                      # IK Solver parameters
│   ├── moveit_controllers.yaml         # MoveIt Trajectory controllers
│   ├── ros2_controllers.yaml             # Hardware controller definitions
│   └── servo_params.yaml                 # Real-time Servo configurations
├── launch/
│   ├── bringup.launch.py                    # Master launch file
│   ├── bringup_controllers.launch.py # Master launch file
│   ├── commander.launch.py              # Autonomous commander node
│   ├── demo.launch.py                        # Visualization demo
│   └── servo_teleop.launch.py            # Servo engine
├── scripts/
│   ├── test_teleop_joints.py                # Joint keyboard teleop
│   └── test_teleop.py                           # Keyboard / Teleop utilities
└── src/
    └── arm_command_node.cpp           # Main API interface
```