# hardware

ROS 2 package that bridges a single I2C microcontroller to the rest of the robot. It drives four stepper motors (flippers) and two DC motors (differential drive) through a custom binary protocol, and publishes their feedback back into the ROS graph.

---

## Package layout

```
hardware/
  include/
    hardware_driver_node.hpp   # ROS node + state structs
    protocol_handler_i2c.hpp   # I2C framing and I/O
    commands.hpp               # Wire-level command types
    tuple_utils.hpp            # Serialisation helpers + Guarded<T>
    crc.hpp                    # CRC-8 implementation
  src/
    dc_motors.cpp              # main() entry point
  msg/
    JointControl.msg           # Custom message for stepper commands
```

---

## Node: `hardware_node`

Executable: `dc_motors`  
Loop rate: 500 Hz (busy-loop with `rclcpp::spin_some` + `tick()`)

### Subscribed topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `hardware_node/cmd_vel` | `geometry_msgs/Twist` | best-effort, depth 1 | Desired base velocity. `linear.x` in m/s, `angular.z` in rad/s. |
| `hardware_node/joint_command` | `hardware/JointControl` | best-effort, depth 1 | Desired stepper positions and speeds by joint name, in rad and rad/s. |

### Published topics

| Topic | Type | QoS | Description |
|---|---|---|---|
| `hardware_node/joint_states` | `sensor_msgs/JointState` | reliable, depth 10 | Stepper feedback: position (rad), velocity (rad/s), effort. |
| `hardware_node/state_vel` | `geometry_msgs/Twist` | reliable, depth 10 | DC motor feedback: reconstructed linear (m/s) and angular (rad/s) from MCU percentages. |

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `i2c_port` | string | `/dev/i2c-7` | I2C device file. |
| `i2c_address` | int | `0x30` | 7-bit slave address of the MCU. |
| `flipper_revolution` | int | `40000` | Steps per full revolution of the stepper motors. |
| `joint_names` | string[] | `[flipper_0..3]` | Names matching the four stepper channels in order. |
| `track_width_m` | double | `1.1` | Wheel-to-wheel distance in metres, used for differential drive kinematics. |
| `velocity_scale` | double | `70.0` | Scalar applied when converting SI velocity to MCU percentage units. |

---

## Architecture

### Tick loop

Every tick follows a fixed sequence:

1. **Connection guard** — if the I2C fd is not open, attempt reconnection (rate-limited to every `max_wait_ticks_` ticks). On successful reconnect, `needs_full_resync_` is set.
2. **I/O** — `readPending()` dispatches any waiting MCU responses, then `sendQueue()` flushes pending outbound commands. If either call causes a disconnect the tick exits early.
3. **Feedback poll** — every `feedback_poll_interval_ticks_` ticks, read commands for POSITION, SPEED, ACCEL, and DCVEL are enqueued.
4. **Command preparation** — if `needs_full_resync_` is set, any input has changed, or the periodic update interval has elapsed, `prepareCommands()` is called.
5. **Publish** — if the I2C callbacks have written new data into the feedback structs since the last publish, `publishJointFeedback()` and/or `publishDCFeedback()` are called.

### State structs and thread safety

ROS callbacks (spin thread) and the tick (main thread) share two pieces of desired state:

- `Guarded<StepperState<4>> in_joints`
- `Guarded<DCState> in_dc`

`Guarded<T>` wraps a value with a `std::mutex` and exposes `with(fn)` and `snapshot()`. ROS callbacks write through setters; the tick snapshots once per cycle and holds no lock during I2C operations.

`StepperState<N>` and `DCState` own their `updated` bitset internally. Setters mark a bit only when the value actually changes. `clearUpdated(mask)` clears only the bits that were snapshotted, so any writes that arrive during the I2C window are not lost.

Feedback structs (`feedback_joints`, `feedback_dc`) are written exclusively by I2C response callbacks, which fire on the tick thread — no mutex is needed for them.

### Desired-to-wire conversion

**Steppers.** `enqueueJointInfo` groups motors with identical targets into a single masked write command to minimise bus traffic. Position is expressed in steps modulo `steps_per_revolution`; speed and acceleration are converted from rad/s and rad/s² to steps/s and steps/s² as floats.

**DC motors.** `enqueueDCInfo` applies standard differential drive kinematics:

```
right_pct = (linear + angular * track_width/2) * velocity_scale
left_pct  = (linear - angular * track_width/2) * velocity_scale
```

Both values are then normalised so neither exceeds ±70 (the MCU's effective maximum) while preserving the ratio between them.

### Resync on reconnect

When the I2C connection is re-established, `needs_full_resync_` forces `prepareCommands()` to call `markAllUpdated()` on both snapshots before enqueuing, ensuring the MCU receives the full desired state even though no ROS message arrived since the disconnect.

---

## Protocol layer (`Protocol_Handler_I2C`)

Manages a single non-blocking I2C file descriptor. All timing is expressed as an absolute `deadline_t` derived from `std::chrono::steady_clock` and propagated downward — the public API accepts `micros timeout` and converts once at the boundary.

### Frame format

```
[0xAA] [instruction] [length] [payload...] [CRC-8]
```

- **Sync byte** `0xAA` — start of every frame.
- **Instruction** — `0x01` ping, `0x02` write command, `0x03` read command.
- **Length** — payload byte count (not including header or CRC).
- **CRC-8** — covers header + payload.

The MCU uses a "no message" sentinel (`0xAA 0x00 <crc>`) when it has nothing to send. `ReadHeader` handles sentinel frames with a bounded do-while loop (max 4 retries) to recover from sentinels that span a chunk boundary.

### I/O model

Reads are buffered internally in 8-byte chunks. Each chunk fetch is a single `ioctl(I2C_RDWR)` that simultaneously reads 8 bytes and sends a 4-byte "skip" message to advance the MCU's transmit pointer.

Writes are issued in 16-byte chunks padded with `0xBB`. Both read and write paths call `ppoll()` before each `ioctl` to respect the absolute deadline with nanosecond resolution without blocking indefinitely.

The file descriptor is opened with `O_NONBLOCK`. This relies on the I2C driver honouring non-blocking mode; drivers known to work include `i2c-bcm2835` (Raspberry Pi) and `i2c-tegra` (Jetson). Intel `i2c-designware` ignores `O_NONBLOCK` — on that hardware, a PREEMPT_RT kernel is the complementary approach for bounding jitter.

### Callback contract

`read_callbacks` and `write_callbacks` are invoked synchronously during `readPending()`, on the same thread as the tick. They must be non-blocking and complete in O(1) — no locks, no I/O, no complex computation.

---

## Serialisation (`tuple_utils.hpp`)

Commands are defined as `std::tuple` types. `pack_tuple_to_buffer` and `unpack_tuple_from_buffer` serialise and deserialise them field-by-field, passing each value through `to_le` / `from_le` to guarantee little-endian wire format regardless of host byte order. On little-endian hosts (ARM, x86) these functions compile to zero-overhead no-ops.

`Guarded<T>` is also defined here as a general-purpose thread-safe wrapper used by the node for shared desired state.

---

## Command types (`commands.hpp`)

Three namespaces define the wire vocabulary:

- `WriteCommandsNC` — commands the host sends to the MCU: SPEED, POSITION, ACCEL (stepper, with motor bitmask), DCVEL, DCACCEL (DC motors, as float pairs).
- `ReadCommandsNC` — commands requesting state from the MCU: same IDs, responses are tuples of values per motor.
- `CommandsNC` — polymorphic `Command` base and `GeneralInstruction<T>` template that binds a command enum value to its packet types and instruction byte. Aliases `WriteInst<CMD>`, `ReadInst<CMD>`, and `MiscInst<CMD>` are used throughout the node.

`dispatch_one<ID>` is a helper that deserialises a raw response buffer into the appropriate tuple type and invokes a handler, returning false on size mismatch.

---

## Building

```bash
cd <your_ws>
colcon build --packages-select hardware
```

Requires ROS 2 (tested on Humble/Iron), `rclcpp`, `sensor_msgs`, `geometry_msgs`, and Linux kernel headers for I2C (`linux/i2c-dev.h`, `linux/i2c.h`).

## Running

```bash
ros2 run hardware dc_motors --ros-args \
  -p i2c_port:=/dev/i2c-7 \
  -p i2c_address:=0x30 \
  -p track_width_m:=1.1 \
  -p velocity_scale:=70.0
```
