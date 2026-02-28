#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "hardware/msg/joint_control.hpp"

#include "commands.hpp"
#include "protocol_handler_i2c.hpp"
#include "tuple_utils.hpp"

// =============================================================================
// StepperState<N>
// Owns desired stepper positions/speeds/accels for N motors.
// Setters automatically mark the corresponding updated bit — callers never
// touch `updated` directly.  clearUpdated(mask) is called by the tick after
// it has snapshotted and enqueued the pending changes.
// =============================================================================

template <int N>
struct StepperState {
  // -------------------------------------------------------------------
  // Data (SI units: rad, rad/s, rad/s²)
  // -------------------------------------------------------------------
  std::array<double, N> position{};
  std::array<double, N> speed{};
  std::array<double, N> acceleration{};
  std::array<double, N> effort{};   // reserved for future use

  // One bit per motor per field.
  // Layout: bits [0..N-1]=position, [N..2N-1]=speed, [2N..3N-1]=accel, [3N..4N-1]=effort
  static constexpr int FIELDS = 4;
  std::bitset<FIELDS> updated{};

  void setPosition(int motor, double rad) {
    if (motor < 0 || motor >= N) return;
    if (position[motor] != rad) { position[motor] = rad; updated.set(0); }
  }
  void setSpeed(int motor, double rad_s) {
    if (motor < 0 || motor >= N) return;
    if (speed[motor] != rad_s) { speed[motor] = rad_s; updated.set(1); }
  }
  void setAcceleration(int motor, double rad_s2) {
    if (motor < 0 || motor >= N) return;
    if (acceleration[motor] != rad_s2) { acceleration[motor] = rad_s2; updated.set(2); }
  }
  void setEffort(int motor, double eff) {
    if (motor < 0 || motor >= N) return;
    if (effort[motor] != eff) { effort[motor] = eff; updated.set(3); }
  }

  // -------------------------------------------------------------------
  // Helpers
  // -------------------------------------------------------------------
  bool updatedPosition() const {
    return updated.test(0);
  }
  bool updatedSpeed() const {
    return updated.test(1);
  }
  bool updatedAccel() const {
    return updated.test(2);
  }

  bool anyUpdated() const { return updated.any(); }

  /// Clear only the bits present in mask — bits set by callbacks during this
  /// window are preserved.
  void clearUpdated(std::bitset<FIELDS> mask) { updated &= ~mask; }

  /// Force-mark all fields for all motors as updated (used after reconnect).
  void markAllUpdated() { updated.set(); }
};

// =============================================================================
// DCState
// Owns desired DC motor velocities in SI units.
// Bit 0 = linear updated, Bit 1 = angular updated.
//
// MCU conversion (differential drive):
//   right_pct = (linear + angular * track_width/2) * velocity_scale
//   left_pct  = (linear - angular * track_width/2) * velocity_scale
// clamped to [-100, 100].
// =============================================================================

struct DCState {
  double linear{0.0};   // m/s
  double angular{0.0};  // rad/s
  std::bitset<1> updated{};

  void setLinear(double v)  { if (linear  != v) { linear  = v; updated.set(0); } }
  void setAngular(double w) { if (angular != w) { angular = w; updated.set(0); } }

  bool anyUpdated() const { return updated.any(); }
  void clearUpdated(std::bitset<1> mask) { updated &= ~mask; }
  void markAllUpdated() { updated.set(); }
};

using namespace CommandsNC;

// =============================================================================
// Declaration
// =============================================================================

class HardwareDriverNode : public rclcpp::Node {
public:
  HardwareDriverNode();
  void tick();

private:
  // --- Unit conversion helpers ----------------------------------------------
  int32_t radToSteps(double rad) const;
  double  stepsToRad(int32_t steps) const;
  /// Differential drive: [m/s, rad/s] → [left_pct, right_pct] clamped [-100,100]
  std::pair<float, float> twistToMotorPct(double linear, double angular) const;

  // --- Setup ----------------------------------------------------------------
  void generateCallbacks();

  // --- ROS callbacks --------------------------------------------------------
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg);

  // --- Command preparation --------------------------------------------------
  void prepareCommands();
  void enqueueJointInfo(const StepperState<4> &snap);
  void enqueueDCInfo(const DCState &snap);

  // --- Error recovery -------------------------------------------------------
  bool errorRecovery();

  // --- Publishing -----------------------------------------------------------
  void publishJointFeedback();
  void publishDCFeedback();

  // =========================================================================
  // Members
  // =========================================================================
  Protocol_Handler_I2C stepper_micro{this->get_logger()};

  int steps_per_revolution{40000};
  static constexpr int steppers{4};

  double track_width_m{1.25};    ///< Distance between wheels [m]
  double velocity_scale{1.0};   ///< (m/s or rad/s) → percent on MCU

  std::vector<std::string> joint_names{"flipper_0", "flipper_1", "flipper_2", "flipper_3"};

  // Desired state — written by ROS callbacks, snapshotted by tick
  Guarded<StepperState<steppers>> in_joints;
  Guarded<DCState>                in_dc;

  // Feedback state — written by I2C callbacks, read by publish methods.
  // Entirely on the tick thread: no mutex needed.
  StepperState<steppers> feedback_joints;
  DCState                feedback_dc;

  // Timing counters
  int ticks_since_feedback_poll_{0};
  static constexpr int feedback_poll_interval_ticks_{3};

  int ticks_since_update_{0};
  static constexpr int update_interval_ticks_{3};

  int wait_counter_{0};
  static constexpr int max_wait_ticks_{5};

  // Set true on first connect and after every reconnect — forces prepareCommands()
  // to send the full state regardless of updated bits.
  bool needs_full_resync_{true};

  // ROS interfaces
  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_state_pub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_state_pub_;
};

// =============================================================================
// Templates
// =============================================================================

// (none — HardwareDriverNode is not a template class)

// =============================================================================
// Inline definitions
// =============================================================================

inline HardwareDriverNode::HardwareDriverNode() : Node("hardware_node") {
  this->declare_parameter<std::string>("i2c_port", "/dev/i2c-7");
  this->declare_parameter<int>("i2c_address", 0x30);
  this->declare_parameter<int>("flipper_revolution", 40000);
  this->declare_parameter<std::vector<std::string>>(
      "joint_names", {"flipper_0", "flipper_1", "flipper_2", "flipper_3"});
  this->declare_parameter<double>("track_width_m", 1.5);
  this->declare_parameter<double>("velocity_scale", 1.0);

  std::string i2c_port_;
  int slave_addr_{};
  this->get_parameter("i2c_port",           i2c_port_);
  this->get_parameter("i2c_address",        slave_addr_);
  this->get_parameter("flipper_revolution", steps_per_revolution);
  this->get_parameter("joint_names",        joint_names);
  this->get_parameter("track_width_m",      track_width_m);
  this->get_parameter("velocity_scale",     velocity_scale);

  stepper_micro.init(i2c_port_, slave_addr_);
  generateCallbacks();

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "hardware_node/cmd_vel", qos_cmd_,
      std::bind(&HardwareDriverNode::cmdVelCallback, this, std::placeholders::_1));

  joint_cmd_sub_ = this->create_subscription<hardware::msg::JointControl>(
      "hardware_node/joint_command", qos_cmd_,
      std::bind(&HardwareDriverNode::jointCommandCallback, this, std::placeholders::_1));

  vel_state_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "hardware_node/state_vel", qos_feedback_);

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hardware_node/joint_states", qos_feedback_);
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::tick() {
  // 1. Connection guard
  if (!stepper_micro.connected() && !errorRecovery()) return;

  // 2. Flush queue + read responses
  stepper_micro.readPending();
  stepper_micro.sendQueue();
  
  if (!stepper_micro.connected()) { wait_counter_ = 0; return; }

  // 3. Periodic feedback poll
  if (ticks_since_feedback_poll_++ >= feedback_poll_interval_ticks_) {
    ticks_since_feedback_poll_ = 0;
    stepper_micro.addCommand(std::make_unique<ReadInst<ReadCommandsNC::POSITION>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<ReadCommandsNC::SPEED>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<ReadCommandsNC::ACCEL>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<ReadCommandsNC::DCVEL>>());
  }

  // 4. Send commands when something changed, periodically, or on resync
  bool joints_changed = in_joints.with([](auto &d) { return d.anyUpdated(); });
  bool dc_changed     = in_dc.with(    [](auto &d) { return d.anyUpdated(); });
  bool periodic       = (ticks_since_update_++ >= update_interval_ticks_);

  if (needs_full_resync_ || periodic || joints_changed || dc_changed) {
    ticks_since_update_ = 0;
    prepareCommands();
  }

  // 5. Publish feedback if I2C callbacks updated the feedback structs
  if (feedback_joints.anyUpdated()) {
    publishJointFeedback();
    feedback_joints.clearUpdated(feedback_joints.updated);
  }
  if (feedback_dc.anyUpdated()) {
    publishDCFeedback();
    feedback_dc.clearUpdated(feedback_dc.updated);
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::prepareCommands() {
  // Snapshot under lock — no I/O while holding the mutex.
  auto joints_snap = in_joints.snapshot();
  auto dc_snap     = in_dc.snapshot();

  if (needs_full_resync_) {
    joints_snap.markAllUpdated();
    dc_snap.markAllUpdated();
    needs_full_resync_ = false;
  }

  enqueueJointInfo(joints_snap);
  enqueueDCInfo(dc_snap);

  // Clear only the bits we snapshotted — preserve any bits set concurrently.
  in_joints.with([&](auto &d) { d.clearUpdated(joints_snap.updated); });
  in_dc.with(    [&](auto &d) { d.clearUpdated(dc_snap.updated);    });
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::enqueueJointInfo(const StepperState<4> &snap) {
  using positionW = WriteInst<WriteCommandsNC::POSITION>;
  using speedW    = WriteInst<WriteCommandsNC::SPEED>;
  using accelW    = WriteInst<WriteCommandsNC::ACCEL>;

  // Position — group motors with identical target into one masked command
  if (snap.updatedPosition()) {
    std::unordered_map<int32_t, uint8_t> groups;
    for (int i = 0; i < steppers; ++i) {
      int32_t key = static_cast<int32_t>(
          radToSteps(snap.position[i]) % steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<positionW>(positionW::packet{mask, key}));
  }

  // Speed
  if (snap.updatedSpeed()) {
    std::unordered_map<int32_t, uint8_t> groups;
    std::unordered_map<int32_t, float>   values;
    for (int i = 0; i < steppers; ++i) {
      int32_t key = radToSteps(snap.speed[i]);
      float   val = static_cast<float>(snap.speed[i] / (2.0 * M_PI) * steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
      values.insert_or_assign(key, val);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<speedW>(speedW::packet{mask, values[key]}));
  }

  // Acceleration
  if (snap.updatedAccel()) {
    std::unordered_map<int32_t, uint8_t> groups;
    std::unordered_map<int32_t, float>   values;
    for (int i = 0; i < steppers; ++i) {
      int32_t key = radToSteps(snap.acceleration[i]);
      float   val = static_cast<float>(snap.acceleration[i] / (2.0 * M_PI) * steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
      values.insert_or_assign(key, val);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<accelW>(accelW::packet{mask, values[key]}));
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::enqueueDCInfo(const DCState &snap) {
  if (!snap.anyUpdated()) return;
  using dcvelW = WriteInst<WriteCommandsNC::DCVEL>;
  auto [left_pct, right_pct] = twistToMotorPct(snap.linear, snap.angular);
  stepper_micro.addCommand(std::make_unique<dcvelW>(dcvelW::packet{left_pct, right_pct}));
}

// -----------------------------------------------------------------------------
inline std::pair<float, float> HardwareDriverNode::twistToMotorPct(
    double linear, double angular) const {
  double half_w = track_width_m * 0.5;
  double right  = (linear + angular * half_w) * velocity_scale;
  double left   = (linear - angular * half_w) * velocity_scale;

  float mult = 1.0;
  float maxVel = std::max(std::fabs(right), std::fabs(left));
  if(maxVel > 70.0){
    mult = 70.0 / maxVel;
  }

  return {left * mult, right * mult};
}

// -----------------------------------------------------------------------------
inline int32_t HardwareDriverNode::radToSteps(double rad) const {
  return static_cast<int32_t>(std::llround(rad / (2.0 * M_PI) * steps_per_revolution));
}

inline double HardwareDriverNode::stepsToRad(int32_t steps) const {
  return static_cast<double>(steps) * (2.0 * M_PI) / steps_per_revolution;
}

// -----------------------------------------------------------------------------
// generateCallbacks
// All MCU-unit → SI conversion happens here.
// These run on the tick thread — write only to feedback_* structs, O(1), no locks.
// -----------------------------------------------------------------------------
inline void HardwareDriverNode::generateCallbacks() {

  // POSITION: steps → rad
  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::POSITION,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<ReadCommandsNC::POSITION>(
            data, size, [&](const auto &info) {
              auto &[p0, p1, p2, p3] = info;
              feedback_joints.setPosition(0, stepsToRad(p0));
              feedback_joints.setPosition(1, stepsToRad(p1));
              feedback_joints.setPosition(2, stepsToRad(p2));
              feedback_joints.setPosition(3, stepsToRad(p3));
            });
        if (!ok) RCLCPP_WARN(get_logger(), "POSITION size mismatch (got %zu)", size);
      });

  // SPEED: steps/s → rad/s
  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::SPEED,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<ReadCommandsNC::SPEED>(
            data, size, [&](const auto &info) {
              auto &[s0, s1, s2, s3] = info;
              auto c = [&](float s) {
                return static_cast<double>(s) * (2.0 * M_PI) / steps_per_revolution;
              };
              feedback_joints.setSpeed(0, c(s0));
              feedback_joints.setSpeed(1, c(s1));
              feedback_joints.setSpeed(2, c(s2));
              feedback_joints.setSpeed(3, c(s3));
            });
        if (!ok) RCLCPP_WARN(get_logger(), "SPEED size mismatch (got %zu)", size);
      });

  // ACCEL: steps/s² → rad/s²
  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::ACCEL,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<ReadCommandsNC::ACCEL>(
            data, size, [&](const auto &info) {
              auto &[a0, a1, a2, a3] = info;
              auto c = [&](int32_t a) {
                return static_cast<double>(a) * (2.0 * M_PI) / steps_per_revolution;
              };
              feedback_joints.setAcceleration(0, c(a0));
              feedback_joints.setAcceleration(1, c(a1));
              feedback_joints.setAcceleration(2, c(a2));
              feedback_joints.setAcceleration(3, c(a3));
            });
        if (!ok) RCLCPP_WARN(get_logger(), "ACCEL size mismatch (got %zu)", size);
      });

  // DCVEL: [left_pct, right_pct] → [m/s, rad/s]
  // Inverse of twistToMotorPct:
  //   linear  = (right + left) / (2 * velocity_scale)
  //   angular = (right - left) / (track_width_m * velocity_scale)
  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::DCVEL,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<ReadCommandsNC::DCVEL>(
            data, size, [&](const auto &info) {
              auto &[left_pct, right_pct] = info;
              double scale = (velocity_scale > 0.0) ? velocity_scale : 1.0;
              double l = static_cast<double>(left_pct);
              double r = static_cast<double>(right_pct);
              feedback_dc.setLinear( (r + l) / (2.0 * scale));
              feedback_dc.setAngular((r - l) / (track_width_m * scale));
            });
        if (!ok) RCLCPP_WARN(get_logger(), "DCVEL size mismatch (got %zu)", size);
      });
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Twist is already SI — write directly through setters.
  in_dc.with([&](DCState &d) {
    d.setLinear(msg->linear.x);
    d.setAngular(msg->angular.z);
  });
}

inline void HardwareDriverNode::jointCommandCallback(
    const hardware::msg::JointControl::SharedPtr msg) {
      // Usar with solo cuando se esta cambiando ya que puede haber otro tipos de joints que no son los steppers
  in_joints.with([&](StepperState<steppers> &d) {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      auto it = std::find(joint_names.begin(), joint_names.end(), msg->joint_names[i]);
      if (it == joint_names.end()) {
        RCLCPP_WARN(get_logger(), "Unknown joint: %s", msg->joint_names[i].c_str());
        continue;
      }
      int idx = static_cast<int>(std::distance(joint_names.begin(), it));
      if (idx >= steppers) {
        RCLCPP_WARN(get_logger(), "Joint index out of range: %s", msg->joint_names[i].c_str());
        continue;
      }
      // msg fields are already in rad / rad/s
      if (i < msg->position.size()) d.setPosition(idx, msg->position[i]);
      if (i < msg->velocity.size()) d.setSpeed(idx,    msg->velocity[i]);
    }
  });
}

// -----------------------------------------------------------------------------
inline bool HardwareDriverNode::errorRecovery() {
  if (++wait_counter_ < max_wait_ticks_) return false;
  wait_counter_ = 0;
  if (stepper_micro.reconnect()) {
    needs_full_resync_ = true;
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::publishJointFeedback() {
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name         = joint_names;
  msg.position.resize(steppers);
  msg.velocity.resize(steppers);
  msg.effort.resize(steppers);
  for (int i = 0; i < steppers; ++i) {
    msg.position[i] = feedback_joints.position[i];
    msg.velocity[i] = feedback_joints.speed[i];
    msg.effort[i]   = feedback_joints.effort[i];
  }
  joint_state_pub_->publish(msg);
}

inline void HardwareDriverNode::publishDCFeedback() {
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = feedback_dc.linear;
  msg.angular.z = feedback_dc.angular;
  vel_state_pub_->publish(msg);
}