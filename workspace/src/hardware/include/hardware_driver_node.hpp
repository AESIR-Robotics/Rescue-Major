#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <memory>
#include <rclcpp/logging.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include "hardware/msg/joint_control.hpp"

#include "commands.hpp"
#include "protocol_handler.hpp"
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
  std::array<double, N> acceleration{M_PI};
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

using namespace Cmd;

// =============================================================================
// HardwareDiagnostics
// Holds the last-known diagnostic state written by the tick thread (I2C
// callbacks and error recovery).  Wrapped in Guarded<> so it is safe to
// snapshot from a future MultiThreadedExecutor timer callback without changes
// to the write sites.
//
// NOTE: with the current SingleThreadedExecutor all writes and the diag timer
// run on the same thread — the mutex has zero contention.  If the executor is
// ever changed to MultiThreadedExecutor this wrapper already provides safety.
// =============================================================================

struct HardwareDiagnostics {
  // Connection
  bool     i2c_connected{false};
  int      reconnect_attempts{0};

  // Byte-loss counter reported by the MCU (populated when ERROR message added)
  uint32_t tx_byte_loss_count{0};
  uint32_t rx_byte_loss_count{0};

  // Set to true by any write site so diagTick() knows a publish is warranted.
  bool updated{false};

  void markUpdated() { updated = true; }
  bool anyUpdated()  const { return updated; }
  void clearUpdated()      { updated = false; }
};

class HardwareDriverNode : public rclcpp::Node {
public:
  HardwareDriverNode();
  void tick();

private:
  // --- Unit conversion helpers ----------------------------------------------
  int32_t radToSteps(double rad) const;

  template<typename T>
  double stepsToRad(T steps) const;
  /// Differential drive: [m/s, rad/s]  [left_pct, right_pct] clamped [-100,100]
  std::pair<float, float> twistToMotorPct(double linear, double angular) const;

  // --- Setup ----------------------------------------------------------------
  void generateCallbacks();

  // --- ROS callbacks --------------------------------------------------------
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg);

  // --- Command preparation --------------------------------------------------
  static constexpr int number_arms{7};
  void prepareCommands();
  void enqueueFlipperInfo(const StepperState<4> &snap);
  void enqueueArmInfo(const StepperState<number_arms> &snap);
  void enqueueDCInfo(const DCState &snap);

  // --- Error recovery -------------------------------------------------------
  bool errorRecovery();

  // --- Publishing -----------------------------------------------------------
  void publishJointFeedback();
  void publishDCFeedback();
  void publishDiagnostics();

  void diagTick();

  Protocol_Handler_I2C<> stepper_micro;
  std::array<Protocol_Handler_CAN<>, number_arms> stepper_arms;

  int steps_per_revolution{40000};
  static constexpr int steppers{10};

  double track_width_m{1.25};    ///< Distance between wheels [m]
  double velocity_scale{1.0};   ///< (m/s or rad/s)  percent on MCU

  std::vector<std::string> joint_names{
    "flipper_0", "flipper_1", "flipper_2", "flipper_3",
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"
  };

  // Desired state — written by ROS callbacks, snapshotted by tick
  // Desired state — flippers (I2C) and arms (CAN) kept separate so their
  // updated bits do not cross-contaminate and error recovery is independent.
  Guarded<StepperState<4>>             in_flipper;   ///< I2C stepper (4 motors)
  Guarded<StepperState<number_arms>>   in_arms;      ///< CAN arm joints (7 motors)
  Guarded<DCState>                in_dc;

  // Feedback state — written by I2C callbacks, read by publish methods.
  // Entirely on the tick thread: no mutex needed.
  StepperState<4>            feedback_flipper;  ///< Feedback from I2C stepper
  StepperState<number_arms>  feedback_arms;     ///< Feedback from CAN arms
  DCState                feedback_dc;

  // Timing counters
  int ticks_since_feedback_poll_{0};
  static constexpr int feedback_poll_interval_ticks_{4};

  int ticks_since_feedback_publish_joint_{0};
  static constexpr int feedback_publish_joint_interval_ticks_{250};

  int ticks_since_feedback_publish_dc_{0};
  static constexpr int feedback_publish_dc_interval_ticks_{250};

  int ticks_since_update_{0};
  static constexpr int update_interval_ticks_{3};

  int wait_counter_{0};
  static constexpr int max_wait_ticks_{5};

  // Set true on first connect and after every reconnect — forces prepareCommands()
  // to send the full state regardless of updated bits.
  bool needs_full_resync_{true};

  // Diagnostics state — written by tick thread, snapshotted by diag timer.
  // Safe for MultiThreadedExecutor via Guarded<>.
  Guarded<HardwareDiagnostics> diag_data_;

  // diagTick() increments this each call; when it reaches diag_force_ticks_
  // a publish is forced even if nothing changed (keeps aggregator alive at ~1 Hz).
  int  diag_ticks_since_pub_{0};
  // At diag_hz_=5 this gives a forced publish every 5 ticks = 1 Hz.
  static constexpr int  diag_force_ticks_{5};
  static constexpr double diag_hz_{5.0};

  // ROS interfaces
  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr   cmd_vel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      vel_state_pub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr   joint_state_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr   error_state_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

// =============================================================================
// Inline definitions
// =============================================================================

inline HardwareDriverNode::HardwareDriverNode() : Node("hardware_node") {
  this->declare_parameter<std::string>("i2c_port", "/dev/i2c-1");
  this->declare_parameter<int>("i2c_address", 0x30);
  this->declare_parameter<int>("flipper_revolution", 40000);
  this->declare_parameter<std::vector<std::string>>(
      "joint_names", {"flipper_0", "flipper_1", "flipper_2", "flipper_3"});
  this->declare_parameter<double>("track_width_m", 1.1);
  this->declare_parameter<double>("velocity_scale", 70.0);

  std::string i2c_port_;
  int slave_addr_{};
  this->get_parameter("i2c_port",           i2c_port_);
  this->get_parameter("i2c_address",        slave_addr_);
  this->get_parameter("flipper_revolution", steps_per_revolution);
  this->get_parameter("joint_names",        joint_names);
  this->get_parameter("track_width_m",      track_width_m);
  this->get_parameter("velocity_scale",     velocity_scale);

  stepper_micro.setLogger(
      [this](const std::string &msg) { RCLCPP_INFO( get_logger(), "%s", msg.c_str()); },
      [this](const std::string &msg) { RCLCPP_WARN( get_logger(), "%s", msg.c_str()); },
      [this](const std::string &msg) { RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); });

  stepper_micro.init(i2c_port_, slave_addr_);

  for (uint8_t i = 0; i < number_arms; ++i) {
      stepper_arms[i].setLogger(
      [this](const std::string &msg) { RCLCPP_INFO( get_logger(), "%s", msg.c_str()); },
      [this](const std::string &msg) { RCLCPP_WARN( get_logger(), "%s", msg.c_str()); },
      [this](const std::string &msg) { RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); });

      stepper_arms[i].init("can0", 0 - i, i, /*channel=*/0);
        
      using Cmd::ESP32::Read;
      const auto logger = [this](const char* fmt, auto... args) {
        RCLCPP_WARN(get_logger(), fmt, args...);
      };
      
      stepper_arms[i].read_callbacks.emplace(
        Read::POSITION,
        Cmd::make_callback<Read::POSITION>([this, i](const auto &info) {
          auto &[p0] = info;
          feedback_arms.setPosition(i, stepsToRad(p0));
        }, logger));


      stepper_arms[i].read_callbacks.emplace(
        Read::SPEED,
        Cmd::make_callback<Read::SPEED>([this, i](const auto &info) {
          auto &[s0] = info;
          feedback_arms.setSpeed(i, stepsToRad(s0));
        }, logger));

      stepper_arms[i].read_callbacks.emplace(
        Read::ACCEL,
        Cmd::make_callback<Read::ACCEL>([this, i](const auto &info) {
          auto &[a0] = info;
          feedback_arms.setAcceleration(i, stepsToRad(a0));
        }, logger));
  }

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

  error_state_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "hardware_node/diagnostic_feed", qos_feedback_);

  diag_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / diag_hz_),
      std::bind(&HardwareDriverNode::diagTick, this));
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
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::POSITION>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::SPEED>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::ACCEL>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::DCVEL>>());
  }

  // 4. Send commands when something changed, periodically, or on resync
  bool joints_changed = in_flipper.with([](auto &d) { return d.anyUpdated(); })
                     || in_arms.with(   [](auto &d) { return d.anyUpdated(); });
  bool dc_changed     = in_dc.with(    [](auto &d) { return d.anyUpdated(); });
  bool periodic       = (ticks_since_update_++ >= update_interval_ticks_);

  if (needs_full_resync_ || periodic || joints_changed || dc_changed) {
    ticks_since_update_ = 0;
    prepareCommands();
  }

  // 5. Publish feedback if I2C callbacks updated the feedback structs
  if (feedback_flipper.anyUpdated() || feedback_arms.anyUpdated() ||
      ++ticks_since_feedback_publish_joint_ >= feedback_publish_joint_interval_ticks_) {
    ticks_since_feedback_publish_joint_ = 0;
    publishJointFeedback();
    feedback_flipper.clearUpdated(feedback_flipper.updated);
    feedback_arms.clearUpdated(feedback_arms.updated);
  }
  if (feedback_dc.anyUpdated() || ++ticks_since_feedback_publish_dc_ >= feedback_publish_dc_interval_ticks_) {
    ticks_since_feedback_publish_dc_ = 0;
    publishDCFeedback();
    feedback_dc.clearUpdated(feedback_dc.updated);
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::prepareCommands() {
  // Snapshot under lock — no I/O while holding the mutex.
  auto flipper_snap = in_flipper.snapshot();
  auto arms_snap    = in_arms.snapshot();
  auto dc_snap     = in_dc.snapshot();

  if (needs_full_resync_) {
    flipper_snap.markAllUpdated();
    arms_snap.markAllUpdated();
    dc_snap.markAllUpdated();
    needs_full_resync_ = false;
  }

  enqueueFlipperInfo(flipper_snap);
  enqueueArmInfo(arms_snap);
  enqueueDCInfo(dc_snap);

  in_flipper.with([&](auto &d) { d.clearUpdated(flipper_snap.updated); });
  in_arms.with(  [&](auto &d) { d.clearUpdated(arms_snap.updated);    });
  in_dc.with(    [&](auto &d) { d.clearUpdated(dc_snap.updated);      });
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::enqueueFlipperInfo(const StepperState<4> &snap) {
  using positionW = WriteInst<Cmd::Teensy::Write::POSITION>;
  using speedW    = WriteInst<Cmd::Teensy::Write::SPEED>;
  using accelW    = WriteInst<Cmd::Teensy::Write::ACCEL>;

  constexpr int flippers = 4;

  if (snap.updatedPosition()) {
    std::unordered_map<int32_t, uint8_t> groups;
    for (int i = 0; i < flippers; ++i) {
      int32_t key = snap.position[i] < 0 ? -1 : static_cast<int32_t>(
          radToSteps(snap.position[i]) % steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<positionW>(positionW::packet{mask, key}));
  }

  if (snap.updatedSpeed()) {
    std::unordered_map<int32_t, uint8_t> groups;
    std::unordered_map<int32_t, float>   values;
    for (int i = 0; i < flippers; ++i) {
      int32_t key = radToSteps(snap.speed[i]);
      float   val = static_cast<float>(snap.speed[i] / (2.0 * M_PI) * steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
      values.insert_or_assign(key, val);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<speedW>(speedW::packet{mask, values[key]}));
  }

  if (snap.updatedAccel()) {
    std::unordered_map<int32_t, uint8_t> groups;
    std::unordered_map<int32_t, float>   values;
    for (int i = 0; i < flippers; ++i) {
      int32_t key = radToSteps(snap.acceleration[i]);
      float   val = static_cast<float>(snap.acceleration[i] / (2.0 * M_PI) * steps_per_revolution);
      groups[key] |= static_cast<uint8_t>(1u << i);
      values.insert_or_assign(key, val);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<accelW>(accelW::packet{mask, values[key]}));
  }
}

inline void HardwareDriverNode::enqueueArmInfo(const StepperState<number_arms> &snap) {
  using positionE = WriteInstESP<Cmd::ESP32::Write::POSITION>;
  using speedE    = WriteInstESP<Cmd::ESP32::Write::SPEED>;
  using accelE    = WriteInstESP<Cmd::ESP32::Write::ACCEL>;

  for (int i = 0; i < number_arms; ++i) {
    if (snap.updatedPosition())
      stepper_arms[i].addCommand(
          std::make_unique<positionE>(positionE::packet{snap.position[i]}));
    if (snap.updatedSpeed())
      stepper_arms[i].addCommand(
          std::make_unique<speedE>(speedE::packet{snap.speed[i]}));
    if (snap.updatedAccel())
      stepper_arms[i].addCommand(
          std::make_unique<accelE>(accelE::packet{snap.acceleration[i]}));
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::enqueueDCInfo(const DCState &snap) {
  if (!snap.anyUpdated()) return;
  using dcvelW = WriteInst<Cmd::Teensy::Write::DCVEL>;
  auto [left_pct, right_pct] = twistToMotorPct(snap.linear, snap.angular);
  auto cmand = std::make_unique<dcvelW>(dcvelW::packet{left_pct, right_pct});
  stepper_micro.addCommand(std::move(cmand));
  
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

template<typename T>
inline double HardwareDriverNode::stepsToRad(T steps) const {
  return static_cast<double>(steps) * (2.0 * M_PI) / steps_per_revolution;
}

// -----------------------------------------------------------------------------
// generateCallbacks
// All MCU-unit  SI conversion happens here.
// These run on the tick thread — write only to feedback_* structs, O(1), no locks.
// -----------------------------------------------------------------------------
inline void HardwareDriverNode::generateCallbacks() {
  using Cmd::Teensy::Read;

  const auto logger = [this](const char* fmt, auto... args) {
        RCLCPP_WARN(get_logger(), fmt, args...);
      };

  stepper_micro.read_callbacks.emplace(
      Read::BYTELOSS,
      Cmd::make_callback<Read::BYTELOSS>([&](const auto &info) {
        auto &[p0, p1] = info;
        diag_data_.with([&p0, &p1](HardwareDiagnostics &d){
          d.tx_byte_loss_count = p0;
          d.rx_byte_loss_count = p1;
        });
      }, logger));

  // POSITION: steps  rad
  stepper_micro.read_callbacks.emplace(
      Read::POSITION,
      Cmd::make_callback<Read::POSITION>([&](const auto &info) {
        auto &[p0, p1, p2, p3] = info;
        feedback_flipper.setPosition(0, stepsToRad(p0));
        feedback_flipper.setPosition(1, stepsToRad(p1));
        feedback_flipper.setPosition(2, stepsToRad(p2));
        feedback_flipper.setPosition(3, stepsToRad(p3));
      }, logger));

  // SPEED: steps/s  rad/s
  stepper_micro.read_callbacks.emplace(
      Read::SPEED,
      Cmd::make_callback<Read::SPEED>([&](const auto &info) {
        auto &[s0, s1, s2, s3] = info;
        feedback_flipper.setSpeed(0, stepsToRad(s0));
        feedback_flipper.setSpeed(1, stepsToRad(s1));
        feedback_flipper.setSpeed(2, stepsToRad(s2));
        feedback_flipper.setSpeed(3, stepsToRad(s3));
      }, logger));

  // ACCEL: steps/s2  rad/s2
  stepper_micro.read_callbacks.emplace(
      Read::ACCEL,
      Cmd::make_callback<Read::ACCEL>([&](const auto &info) {
        auto &[a0, a1, a2, a3] = info;
        feedback_flipper.setAcceleration(0, stepsToRad(a0));
        feedback_flipper.setAcceleration(1, stepsToRad(a1));
        feedback_flipper.setAcceleration(2, stepsToRad(a2));
        feedback_flipper.setAcceleration(3, stepsToRad(a3));
      }, logger));

  // DCVEL: [left_pct, right_pct]  [m/s, rad/s]
  // Inverse of twistToMotorPct:
  //   linear  = (right + left) / (2 * velocity_scale)
  //   angular = (right - left) / (track_width_m * velocity_scale)
  stepper_micro.read_callbacks.emplace(
      Read::DCVEL,
      Cmd::make_callback<Read::DCVEL>([&](const auto &info) {
        auto &[left_pct, right_pct] = info;
        double scale = (velocity_scale > 0.0) ? velocity_scale : 1.0;
        double l = static_cast<double>(left_pct);
        double r = static_cast<double>(right_pct);
        feedback_dc.setLinear( (r + l) / (2.0 * scale));
        feedback_dc.setAngular((r - l) / (track_width_m * scale));
      }, logger));
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
  // Write to in_flipper (idx 0-3) or in_arms (idx 4-10) depending on joint index.
  in_flipper.with([&](StepperState<4> &d) {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      auto it = std::find(joint_names.begin(), joint_names.end(), msg->joint_names[i]);
      if (it == joint_names.end()) continue;
      int idx = static_cast<int>(std::distance(joint_names.begin(), it));
      if (idx >= 4) continue;  // handled by in_arms below
      d.setPosition(idx, msg->position[i]);
      d.setSpeed(idx,    msg->velocity[i]);
      d.setAcceleration(idx, msg->acceleration[i]);
    }
  });
  in_arms.with([&](StepperState<number_arms> &d) {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      auto it = std::find(joint_names.begin(), joint_names.end(), msg->joint_names[i]);
      if (it == joint_names.end()) continue;
      int idx = static_cast<int>(std::distance(joint_names.begin(), it));
      if (idx < 4 || idx >= steppers) continue;  // flipper range handled above
      int arm_idx = idx - 4;
      d.setPosition(arm_idx, msg->position[i]);
      d.setSpeed(arm_idx,    msg->velocity[i]);
      d.setAcceleration(arm_idx, msg->acceleration[i]);
    }
  });
}

// -----------------------------------------------------------------------------
inline bool HardwareDriverNode::errorRecovery() {
  if (++wait_counter_ < max_wait_ticks_) return false;
  wait_counter_ = 0;
  if (stepper_micro.reconnect()) {
    needs_full_resync_ = true;
    diag_data_.with([](HardwareDiagnostics &d) {
      d.i2c_connected = true;
      d.reconnect_attempts = 0;
      d.markUpdated();
    });
    return true;
  }
  diag_data_.with([](HardwareDiagnostics &d) {
    ++d.reconnect_attempts;
    d.i2c_connected = false;
    d.markUpdated();
  });
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
  for (int i = 0; i < 4; ++i) {
    msg.position[i] = feedback_flipper.position[i];
    msg.velocity[i] = feedback_flipper.speed[i];
    msg.effort[i]   = feedback_flipper.effort[i];
  }
  for (int i = 0; i < number_arms; ++i) {
    msg.position[4+i] = feedback_arms.position[i];
    msg.velocity[4+i] = feedback_arms.speed[i];
    msg.effort[4+i]   = feedback_arms.effort[i];
  }
  joint_state_pub_->publish(msg);
}

inline void HardwareDriverNode::publishDCFeedback() {
  geometry_msgs::msg::Twist msg;
  msg.linear.x  = feedback_dc.linear;
  msg.angular.z = feedback_dc.angular;
  vel_state_pub_->publish(msg);
}

inline void HardwareDriverNode::diagTick() {
  bool changed = diag_data_.with([](HardwareDiagnostics &d) {
    return d.anyUpdated();
  });

  bool force = (++diag_ticks_since_pub_ >= diag_force_ticks_);

  if (changed || force) {
    diag_ticks_since_pub_ = 0;
    publishDiagnostics();
    diag_data_.with([](HardwareDiagnostics &d) { d.clearUpdated(); });
  }
}

// -----------------------------------------------------------------------------
// publishDiagnostics — builds and publishes the DiagnosticArray from a
// snapshot of diag_data_.  Add KeyValue entries here when MCU error messages
// are implemented.
// -----------------------------------------------------------------------------
inline void HardwareDriverNode::publishDiagnostics() {
  auto snap = diag_data_.snapshot();

  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = this->now();
  {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name      = this->get_name();
  status.hardware_id = stepper_micro.getDevice();

  if (snap.i2c_connected) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Connected";
  } else {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "I2C disconnected";
  }

  // Reconnect attempt counter — always visible in rqt_robot_monitor
  diagnostic_msgs::msg::KeyValue kv_reconnects;
  kv_reconnects.key   = "reconnect_attempts";
  kv_reconnects.value = std::to_string(snap.reconnect_attempts);
  status.values.push_back(kv_reconnects);
  array_msg.status.push_back(status);
  }

  {
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name      = "Byte Loss";
  status.hardware_id = stepper_micro.getDevice();

  if (snap.i2c_connected) {
    status.level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Info";
  }

  // Reconnect attempt counter — always visible in rqt_robot_monitor
  diagnostic_msgs::msg::KeyValue kv_reconnects_tx, kv_reconnects_rx;
  kv_reconnects_tx.key   = "tx_byte_loss";
  kv_reconnects_tx.value = std::to_string(snap.tx_byte_loss_count);
  status.values.push_back(kv_reconnects_tx);

  kv_reconnects_rx.key   = "tx_byte_loss";
  kv_reconnects_rx.value = std::to_string(snap.rx_byte_loss_count);
  status.values.push_back(kv_reconnects_rx);

  array_msg.status.push_back(status);
  }

  // TODO: add motor error flags here when MCU error message is implemented

  error_state_pub_->publish(array_msg);
}