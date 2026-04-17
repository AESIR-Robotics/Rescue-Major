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
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "hardware/msg/joint_control.hpp"

#include "commands.hpp"
#include "protocol_handler.hpp"

#include "transport/bridge_transport.hpp"
#include "transport/serial_iface_manager.hpp"

#include "utils/diagnostics.hpp"
#include "utils/tuple_utils.hpp"
#include "utils/logger.hpp"

// Alias para el nuevo transporte por Mux Serial
template<typename ReadID = Cmd::ESP32::Read, typename WriteID = Cmd::ESP32::Write>
using Protocol_Handler_Bridge = Protocol_Handler<Bridge_Transport, ReadID, WriteID>;

// =============================================================================
// StepperState<N>
// =============================================================================
template <int N>
struct StepperState {
  std::array<double, N> position{};
  std::array<double, N> speed{};
  std::array<double, N> acceleration{M_PI};
  std::array<double, N> effort{};   

  static constexpr int FIELDS = 4;
  std::bitset<FIELDS> updated{};

  void setPosition(int motor, double rad) {
    if (motor < 0 || motor >= N) return;
    if (position[motor] != rad) { position[motor] = rad; updated.set(0); touch(); }
  }
  void setSpeed(int motor, double rad_s) {
    if (motor < 0 || motor >= N) return;
    if (speed[motor] != rad_s) { speed[motor] = rad_s; updated.set(1); touch(); }
  }
  void setAcceleration(int motor, double rad_s2) {
    if (motor < 0 || motor >= N) return;
    if (acceleration[motor] != rad_s2) { acceleration[motor] = rad_s2; updated.set(2); touch(); }
  }
  void setEffort(int motor, double eff) {
    if (motor < 0 || motor >= N) return;
    if (effort[motor] != eff) { effort[motor] = eff; updated.set(3); touch(); }
  }

  bool updatedPosition() const { return updated.test(0); }
  bool updatedSpeed() const    { return updated.test(1); }
  bool updatedAccel() const    { return updated.test(2); }
  bool anyUpdated() const      { return updated.any();   }

  void clearUpdated(std::bitset<FIELDS> mask) { updated &= ~mask; }
  void markAllUpdated() { updated.set(); }

  void touch() { feedback_received = true; }
  void stampFeedback(const rclcpp::Time &t) { last_feedback_time = t; feedback_received = true; }

  rclcpp::Time last_feedback_time{};
  bool         feedback_received{false};
};

// =============================================================================
// DCState
// =============================================================================
struct DCState {
  double linear{0.0};   // m/s
  double angular{0.0};  // rad/s
  std::bitset<1> updated{};

  void setLinear(double v)  { if (linear  != v) { linear  = v; updated.set(0); touch(); } }
  void setAngular(double w) { if (angular != w) { angular = w; updated.set(0); touch(); } }

  bool anyUpdated() const { return updated.any(); }
  void clearUpdated(std::bitset<1> mask) { updated &= ~mask; }
  void markAllUpdated() { updated.set(); }

  rclcpp::Time last_feedback_time{};
  bool         feedback_received{false};
  void touch()                          { feedback_received = true; }
  void stampFeedback(const rclcpp::Time &t) { last_feedback_time = t; feedback_received = true; }
};

using namespace Cmd;
namespace jsm  = sensor_msgs::msg;
namespace diag = diagnostic_msgs::msg;
namespace geom = geometry_msgs::msg;

// =============================================================================
// HardwareDriverNode
// =============================================================================
class HardwareDriverNode : public rclcpp::Node {
public:
  HardwareDriverNode();
  void tick();
  void tickI2C();
  void tickBridge();

private:
  int32_t radToSteps(double rad, int joint) const;

  template<typename T>
  double stepsToRad(T steps, int joint) const;     
  template<typename T>
  double stepsToRadRate(T steps, int joint) const;  
  std::pair<float, float> twistToMotorPct(double linear, double angular) const;

  void generateCallbacks();

  void cmdVelCallback(const geom::Twist::SharedPtr msg);
  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg);

  static constexpr int number_arms{4};
  void prepareI2CCommands();
  void prepareBridgeCommands(int arm_index);
  void enqueueFlipperInfo(const StepperState<4> &snap);
  void enqueueArmInfo(const StepperState<number_arms> &snap, int arm_index);
  void enqueueDCInfo(const DCState &snap);

  bool errorRecoveryI2C();
  bool errorRecoveryBridgeArm(int i);

  void deadmanStop();

  void publishJointFeedback();
  void publishDCFeedback();
  void publishDiagnostics();
  void diagTick();

  Logger logger{};

  Protocol_Handler_I2C<> stepper_micro;
  std::array<Protocol_Handler_Bridge<>, number_arms> stepper_arms;
  std::array<int, number_arms> credit;

  std::array<int, 11>    steps_per_rev_{};   
  std::array<double, 11> home_offset_{};     
  static constexpr int steppers{10};

  double track_width_m{1.25};    
  double velocity_scale{1.0};   

  std::vector<std::string> joint_names{
    "flipper_0", "flipper_1", "flipper_2", "flipper_3",
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"
  };

  Guarded<StepperState<4>>             in_flipper;   
  Guarded<StepperState<number_arms>>   in_arms;      
  Guarded<DCState>                     in_dc;

  StepperState<4>            feedback_flipper;
  StepperState<number_arms>  feedback_arms;

  static constexpr double joint_timeout_s_{2.0};
  DCState                feedback_dc;

  rclcpp::Time      last_cmd_time_{};
  bool              deadman_initialized_{ false };
  static constexpr double deadman_timeout_s_{ 0.5 }; 
  bool              deadman_fired_{ false };          

  int i2c_poll_ticks_{0};
  int bridge_poll_ticks_{0};
  static constexpr int feedback_poll_interval_ticks_{10};

  int ticks_since_feedback_publish_joint_{0};
  static constexpr int feedback_publish_joint_interval_ticks_{250};

  int ticks_since_feedback_publish_dc_{0};
  static constexpr int feedback_publish_dc_interval_ticks_{250};

  int i2c_update_ticks_{0};
  int bridge_update_ticks_{0};
  static constexpr int update_interval_ticks_{3};

  int i2c_wait_counter_{0};
  std::array<int, number_arms> bridge_wait_counter_{};
  static constexpr int max_wait_ticks_{50};

  static int constexpr max_budget{128};
  static int constexpr min_budget{-128};

  bool i2c_needs_resync_{true};
  bool bridge_needs_resync_{true};

  DiagnosticRegistry sysStats;

  int  diag_ticks_since_pub_{0};
  static constexpr int  diag_force_ticks_{500};
  static constexpr double diag_hz_{5.0};

  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::Subscription<geom::Twist>::SharedPtr   cmd_vel_sub_;
  rclcpp::Publisher<geom::Twist>::SharedPtr      vel_state_pub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;
  rclcpp::Publisher<jsm::JointState>::SharedPtr   joint_state_pub_;
  rclcpp::Publisher<diag::DiagnosticArray>::SharedPtr   error_state_pub_;
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

// =============================================================================
// Inline definitions
// =============================================================================

inline HardwareDriverNode::HardwareDriverNode() : Node("hardware_node") {
  this->declare_parameter<std::string>("i2c_port", "/dev/i2c-7");
  this->declare_parameter<int>("i2c_address", 0x30);
  
  // Nuevos parámetros para Serial Bridge
  this->declare_parameter<std::string>("bridge_serial_port", "/dev/ttyUSB0");
  this->declare_parameter<int>("bridge_serial_baud", 921600);

  this->declare_parameter<std::vector<int>>(
      "steps_per_rev",
      {40000, 40000, 40000, 40000, 40000, 40000, 40000, 5000, 5000, 5000, 400});
  this->declare_parameter<std::vector<double>>(
      "home_offset_rad",
      {3.141592653589793, 3.141592653589793, 3.141592653589793, 3.141592653589793, 3.141549976140265, 0.2791842593401257, 6.004033568448903, 1.5708634895150595, 1.5708819964673881, 3.141580759052331, 0.0});
  this->declare_parameter<std::vector<std::string>>(
      "joint_names", {"flipper_0", "flipper_1", "flipper_2", "flipper_3", 
        "joint_1", "joint_2", "joint_3", "joint_4"});
  this->declare_parameter<double>("track_width_m", 1.1);
  this->declare_parameter<double>("velocity_scale",95.0);

  std::string i2c_port_;
  int slave_addr_{};
  this->get_parameter("i2c_port",           i2c_port_);
  this->get_parameter("i2c_address",        slave_addr_);
  
  std::string bridge_port;
  int bridge_baud = 921600;
  this->get_parameter("bridge_serial_port", bridge_port);
  this->get_parameter("bridge_serial_baud", bridge_baud);

  {
    std::vector<int64_t> spr;
    this->get_parameter("steps_per_rev", spr);
    spr.resize(11, 40000);
    for (int j = 0; j < 11; ++j) steps_per_rev_[j] = spr[j] > 0 ? spr[j] : 40000;
  }
  {
    std::vector<double> ho;
    this->get_parameter("home_offset_rad", ho);
    ho.resize(11, 0.0);
    for (int j = 0; j < 11; ++j) home_offset_[j] = ho[j];
  }
  this->get_parameter("joint_names",        joint_names);
  this->get_parameter("track_width_m",      track_width_m);
  this->get_parameter("velocity_scale",     velocity_scale);

  logger.setLogger([this](const std::string &msg) { RCLCPP_INFO( get_logger(), "%s", msg.c_str()); },
        [this](const std::string &msg) { RCLCPP_WARN( get_logger(), "%s", msg.c_str()); },
       [this](const std::string &msg) { RCLCPP_ERROR(get_logger(), "%s", msg.c_str()); });

  stepper_micro.setLogger(logger);
  stepper_micro.init(&sysStats, i2c_port_, slave_addr_);

  // Inicialización del SerialIfaceManager
  auto bridge_mgr = std::make_shared<SerialIfaceManager>(&sysStats, bridge_port, bridge_baud);
  bridge_mgr->setLogger(logger);

  for (int i = 0; i < number_arms; ++i) {
      stepper_arms[i].setLogger(logger);
      stepper_arms[i].setIfaceManager(bridge_mgr);
      
      // La inicialización ahora omite el string del puerto físico (lo maneja el manager)
      stepper_arms[i].init(&sysStats, /*my=*/static_cast<uint8_t>(0x00 - i - 1),
                                      /*peer=*/static_cast<uint8_t>(i + 1), /*channel=*/0);
        
      using Cmd::ESP32::Read;
      const auto loggerLocal = [this](const char* fmt, auto... args) {
        RCLCPP_WARN(get_logger(), fmt, args...);
      };
      
      stepper_arms[i].read_callbacks.emplace(
        Read::POSITION,
        Cmd::make_callback<Read::POSITION>([this, i](const auto &info) {
          auto &[p0] = info;
          feedback_arms.setPosition(i, stepsToRad(p0, 4 + i));
          feedback_arms.stampFeedback(this->now());
        }, loggerLocal));


      stepper_arms[i].read_callbacks.emplace(
        Read::SPEED,
        Cmd::make_callback<Read::SPEED>([this, i](const auto &info) {
          auto &[s0] = info;
          feedback_arms.setSpeed(i, stepsToRadRate(s0, 4 + i));
        }, loggerLocal));

      stepper_arms[i].read_callbacks.emplace(
        Read::ACCEL,
        Cmd::make_callback<Read::ACCEL>([this, i](const auto &info) {
          auto &[a0] = info;
          feedback_arms.setAcceleration(i, stepsToRadRate(a0, 4 + i));
        }, loggerLocal));
  }

  generateCallbacks();

  cmd_vel_sub_ = this->create_subscription<geom::Twist>(
      "hardware_node/cmd_vel", qos_cmd_,
      std::bind(&HardwareDriverNode::cmdVelCallback, this, std::placeholders::_1));

  joint_cmd_sub_ = this->create_subscription<hardware::msg::JointControl>(
      "hardware_node/joint_command", qos_cmd_,
      std::bind(&HardwareDriverNode::jointCommandCallback, this, std::placeholders::_1));

  vel_state_pub_ = this->create_publisher<geom::Twist>(
      "hardware_node/state_vel", qos_feedback_);

  joint_state_pub_ = this->create_publisher<jsm::JointState>(
      "hardware_node/joint_states", qos_feedback_);

  error_state_pub_ = this->create_publisher<diag::DiagnosticArray>(
    "hardware_node/diagnostic_feed", qos_feedback_);

  diag_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / diag_hz_),
      std::bind(&HardwareDriverNode::diagTick, this));
}

inline void HardwareDriverNode::tick() {
  if (deadman_initialized_ && !deadman_fired_) {
    double age = (this->now() - last_cmd_time_).seconds();
    if (age > deadman_timeout_s_) {
      deadman_fired_ = true;
    }
  }

  tickI2C();
  tickBridge();
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::tickI2C() {
  if (!stepper_micro.connected() && !errorRecoveryI2C()) return;

  stepper_micro.readPending();
  stepper_micro.sendQueue();
  if (!stepper_micro.connected()) { i2c_wait_counter_ = 0; return; }

  if (i2c_poll_ticks_++ >= feedback_poll_interval_ticks_) {
    i2c_poll_ticks_ = 0;
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::POSITION>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::SPEED>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::ACCEL>>());
    stepper_micro.addCommand(std::make_unique<ReadInst<Cmd::Teensy::Read::DCVEL>>());
  }

  bool flipper_changed = in_flipper.with([](auto &d) { return d.anyUpdated(); });
  bool dc_changed      = in_dc.with(    [](auto &d) { return d.anyUpdated(); });
  bool periodic        = (i2c_update_ticks_++ >= update_interval_ticks_);

  if (i2c_needs_resync_ || periodic || flipper_changed || dc_changed) {
    i2c_update_ticks_ = 0;
    prepareI2CCommands();
  }

  if (feedback_flipper.anyUpdated() || feedback_arms.anyUpdated() ||
      ++ticks_since_feedback_publish_joint_ >= feedback_publish_joint_interval_ticks_) {
    ticks_since_feedback_publish_joint_ = 0;
    publishJointFeedback();
    feedback_flipper.clearUpdated(feedback_flipper.updated);
    feedback_arms.clearUpdated(feedback_arms.updated);
  }
  if (feedback_dc.anyUpdated() ||
      ++ticks_since_feedback_publish_dc_ >= feedback_publish_dc_interval_ticks_) {
    ticks_since_feedback_publish_dc_ = 0;
    publishDCFeedback();
    feedback_dc.clearUpdated(feedback_dc.updated);
  }
}

inline void HardwareDriverNode::tickBridge() {
  bool arms_changed = in_arms.with([](auto &d) { return d.anyUpdated(); });
  bool periodic     = (bridge_update_ticks_++ >= update_interval_ticks_);
  bool do_send      = bridge_needs_resync_ || periodic || arms_changed;
  if (do_send) bridge_update_ticks_ = 0;

  bool do_poll = (bridge_poll_ticks_++ >= feedback_poll_interval_ticks_);
  if (do_poll) bridge_poll_ticks_ = 0;

  std::array<int, number_arms> order;
  int count = 0;

  for (int i = 0; i < number_arms; ++i) {
      if (stepper_arms[i].connected() || errorRecoveryBridgeArm(i)) {
          credit[i] = std::min(credit[i] + 8, max_budget);
          order[count++] = i;
      }
  }

  std::sort(order.data(), order.data() + count, [&](int a, int b) {
      int sa = stepper_arms[a].msgQueued() * 8 + credit[a];
      int sb = stepper_arms[b].msgQueued() * 8 + credit[b];
      return sa > sb;
  });

  for (int k = 0; k < count; ++k) {
    int i = order[k];
    auto &arm = stepper_arms[i];
    
    arm.readPending();
    arm.sendQueue();

    credit[i] = std::max(credit[i] - arm.byteSentFromQueue(), min_budget);

    if (!arm.connected()) { 
        bridge_wait_counter_[i] = 0; 
        credit[i] = 0; 
        continue; 
    }

    if (do_poll) {
      arm.addCommand(std::make_unique<ReadInstESP<Cmd::ESP32::Read::POSITION>>());
      arm.addCommand(std::make_unique<ReadInstESP<Cmd::ESP32::Read::SPEED>>());
      arm.addCommand(std::make_unique<ReadInstESP<Cmd::ESP32::Read::ACCEL>>());
    }

    if (do_send) prepareBridgeCommands(i);
  }

  if (bridge_needs_resync_) bridge_needs_resync_ = false;

  if (feedback_arms.anyUpdated()) {
    if (!stepper_micro.connected()) {
      publishJointFeedback();
      feedback_arms.clearUpdated(feedback_arms.updated);
    }
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::prepareI2CCommands() {
  auto flipper_snap = in_flipper.snapshot();
  auto dc_snap      = in_dc.snapshot();

  if (i2c_needs_resync_) {
    flipper_snap.markAllUpdated();
    dc_snap.markAllUpdated();
    i2c_needs_resync_ = false;
  }

  enqueueFlipperInfo(flipper_snap);
  enqueueDCInfo(dc_snap);

  in_flipper.with([&](auto &d) { d.clearUpdated(flipper_snap.updated); });
  in_dc.with(    [&](auto &d) { d.clearUpdated(dc_snap.updated);      });
}

inline void HardwareDriverNode::prepareBridgeCommands(int i) {
  auto arms_snap = in_arms.snapshot();

  if (bridge_needs_resync_) arms_snap.markAllUpdated();

  enqueueArmInfo(arms_snap, i);

  if (i == number_arms - 1)
    in_arms.with([&](auto &d) { d.clearUpdated(arms_snap.updated); });
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
          radToSteps(snap.position[i], i));
      groups[key] |= static_cast<uint8_t>(1u << i);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<positionW>(positionW::packet{mask, key}));
  }

  if (snap.updatedSpeed()) {
    std::unordered_map<int32_t, uint8_t> groups;
    std::unordered_map<int32_t, float>   values;
    for (int i = 0; i < flippers; ++i) {
      int32_t key = snap.speed[i] / (2.0 * M_PI) * steps_per_rev_[i];
      float   val = static_cast<float>(snap.speed[i] / (2.0 * M_PI) * steps_per_rev_[i]);
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
      int32_t key = snap.acceleration[i] / (2.0 * M_PI) * steps_per_rev_[i];
      float   val = static_cast<float>(snap.acceleration[i] / (2.0 * M_PI) * steps_per_rev_[i]);
      groups[key] |= static_cast<uint8_t>(1u << i);
      values.insert_or_assign(key, val);
    }
    for (auto &[key, mask] : groups)
      stepper_micro.addCommand(std::make_unique<accelW>(accelW::packet{mask, values[key]}));
  }
}

inline void HardwareDriverNode::enqueueArmInfo(const StepperState<number_arms> &snap, int i) {
  using positionE = WriteInstESP<Cmd::ESP32::Write::POSITION>;
  using speedE    = WriteInstESP<Cmd::ESP32::Write::SPEED>;
  using accelE    = WriteInstESP<Cmd::ESP32::Write::ACCEL>;

  if (snap.updatedPosition()){
    stepper_arms[i].addCommand(
        std::make_unique<positionE>(positionE::packet{
            snap.position[i] < 0 ? -1 : static_cast<int32_t>(
                radToSteps(snap.position[i], 4 + i))}));
  }
  if (snap.updatedSpeed()){
    stepper_arms[i].addCommand(
        std::make_unique<speedE>(speedE::packet{
            static_cast<float>(snap.speed[i] / (2.0 * M_PI) * steps_per_rev_[4+i])}));
        }
  if (snap.updatedAccel()){
    stepper_arms[i].addCommand(
        std::make_unique<accelE>(accelE::packet{
            static_cast<float>(snap.acceleration[i] / (2.0 * M_PI) * steps_per_rev_[4+i])}));
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
  if(maxVel > 95.0){
    mult = 95.0 / maxVel;
  }

  return {left * mult, right * mult};
}

// -----------------------------------------------------------------------------
inline int32_t HardwareDriverNode::radToSteps(double rad, int joint) const {
  double angle = rad - home_offset_[joint];

  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;

  return static_cast<int32_t>(
      std::llround(angle / (2.0 * M_PI) * steps_per_rev_[joint]));
}

template<typename T>
inline double HardwareDriverNode::stepsToRad(T steps, int joint) const {

  double angle = static_cast<double>(steps) * (2.0 * M_PI) / steps_per_rev_[joint] + home_offset_[joint];

  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle < 0)
    angle += 2.0 * M_PI;

  return angle;   
}

template<typename T>
inline double HardwareDriverNode::stepsToRadRate(T steps, int joint) const {
  return static_cast<double>(steps) * (2.0 * M_PI) / steps_per_rev_[joint];
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::generateCallbacks() {
  using Cmd::Teensy::Read;

  const auto loggerLocal = [this](const char* fmt, auto... args) {
        RCLCPP_WARN(get_logger(), fmt, args...);
      };

  stepper_micro.read_callbacks.emplace(
      Read::BYTELOSS,
      Cmd::make_callback<Read::BYTELOSS>([&](const auto &info) {
        auto &[p0, p1] = info;
        (void)p0;
        (void)p1;
      }, loggerLocal));

  stepper_micro.read_callbacks.emplace(
      Read::POSITION,
      Cmd::make_callback<Read::POSITION>([&](const auto &info) {
        auto &[p0, p1, p2, p3] = info;
        feedback_flipper.setPosition(0, stepsToRad(p0, 0));
        feedback_flipper.setPosition(1, stepsToRad(p1, 1));
        feedback_flipper.setPosition(2, stepsToRad(p2, 2));
        feedback_flipper.setPosition(3, stepsToRad(p3, 3));
        feedback_flipper.stampFeedback(this->now());
      }, loggerLocal));

  stepper_micro.read_callbacks.emplace(
      Read::SPEED,
      Cmd::make_callback<Read::SPEED>([&](const auto &info) {
        auto &[s0, s1, s2, s3] = info;
        feedback_flipper.setSpeed(0, stepsToRadRate(s0, 0));
        feedback_flipper.setSpeed(1, stepsToRadRate(s1, 1));
        feedback_flipper.setSpeed(2, stepsToRadRate(s2, 2));
        feedback_flipper.setSpeed(3, stepsToRadRate(s3, 3));
      }, loggerLocal));

  stepper_micro.read_callbacks.emplace(
      Read::ACCEL,
      Cmd::make_callback<Read::ACCEL>([&](const auto &info) {
        auto &[a0, a1, a2, a3] = info;
        feedback_flipper.setAcceleration(0, stepsToRadRate(a0, 0));
        feedback_flipper.setAcceleration(1, stepsToRadRate(a1, 1));
        feedback_flipper.setAcceleration(2, stepsToRadRate(a2, 2));
        feedback_flipper.setAcceleration(3, stepsToRadRate(a3, 3));
      }, loggerLocal));

  stepper_micro.read_callbacks.emplace(
      Read::DCVEL,
      Cmd::make_callback<Read::DCVEL>([&](const auto &info) {
        auto &[left_pct, right_pct] = info;
        double scale = (velocity_scale > 0.0) ? velocity_scale : 1.0;
        double l = static_cast<double>(left_pct);
        double r = static_cast<double>(right_pct);
        feedback_dc.setLinear( (r + l) / (2.0 * scale));
        feedback_dc.setAngular((r - l) / (track_width_m * scale));
      }, loggerLocal));
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::cmdVelCallback(
    const geom::Twist::SharedPtr msg) {
  in_dc.with([&](DCState &d) {
    d.setLinear(msg->linear.x);
    d.setAngular(msg->angular.z);
  });
  last_cmd_time_     = this->now();
  deadman_initialized_ = true;
  deadman_fired_     = false;   
}

inline void HardwareDriverNode::jointCommandCallback(
    const hardware::msg::JointControl::SharedPtr msg) {
  in_flipper.with([&](StepperState<4> &d) {
    for (size_t i = 0; i < msg->joint_names.size(); ++i) {
      auto it = std::find(joint_names.begin(), joint_names.end(), msg->joint_names[i]);
      if (it == joint_names.end()) continue;
      int idx = static_cast<int>(std::distance(joint_names.begin(), it));
      if (idx >= 4) continue;  
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
      if (idx < 4 || idx >= steppers) continue;  
      int arm_idx = idx - 4;
      d.setPosition(arm_idx, msg->position[i]);
      d.setSpeed(arm_idx,    msg->velocity[i]);
      d.setAcceleration(arm_idx, msg->acceleration[i]);
    }
  });
  last_cmd_time_       = this->now();
  deadman_initialized_ = true;
  deadman_fired_       = false;
}


// -----------------------------------------------------------------------------
inline void HardwareDriverNode::deadmanStop() {
  in_dc.with([](DCState &d) {
    d.setLinear(0.0);
    d.setAngular(0.0);
  });
  in_flipper.with([](StepperState<4> &d) {
    for (int i = 0; i < 4; ++i) d.setSpeed(i, 0.0);
  });
  in_arms.with([](StepperState<number_arms> &d) {
    for (int i = 0; i < number_arms; ++i) d.setSpeed(i, 0.0);
  });
  RCLCPP_WARN(get_logger(), "Deadman triggered — all velocities zeroed");
}
// -----------------------------------------------------------------------------
inline bool HardwareDriverNode::errorRecoveryI2C() {
  if (++i2c_wait_counter_ < max_wait_ticks_) return false;
  i2c_wait_counter_ = 0;
  if (stepper_micro.reconnect()) {
    i2c_needs_resync_ = true;
    return true;
  }
  return false;
}

inline bool HardwareDriverNode::errorRecoveryBridgeArm(int i) {
  
  if (++bridge_wait_counter_[i] < max_wait_ticks_) return false;
  bridge_wait_counter_[i] = 0;
  if (stepper_arms[i].reconnect()) {
    bridge_needs_resync_ = true;
    logger.logInfo("Bridge serial reconnected (triggered by arm %d)", i);
    return true;
  }
  return false;
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::publishJointFeedback() {
  if (!feedback_flipper.feedback_received && !feedback_arms.feedback_received) return;

  const rclcpp::Time now = this->now();
  jsm::JointState msg;
  msg.header.stamp = now;

  if (feedback_flipper.feedback_received) {
    double age = (now - feedback_flipper.last_feedback_time).seconds();
    if (age <= joint_timeout_s_) {
      for (int i = 0; i < 4; ++i) {
        msg.name.push_back(joint_names[i]);
        msg.position.push_back(feedback_flipper.position[i]);
        msg.velocity.push_back(feedback_flipper.speed[i]);
        msg.effort.push_back(feedback_flipper.effort[i]);
      }
    }
  }

  if (feedback_arms.feedback_received) {
    double age = (now - feedback_arms.last_feedback_time).seconds();
    if (age <= joint_timeout_s_) {
      for (int i = 0; i < number_arms; ++i) {
        msg.name.push_back(joint_names[4 + i]);
        msg.position.push_back(feedback_arms.position[i]);
        msg.velocity.push_back(feedback_arms.speed[i]);
        msg.effort.push_back(feedback_arms.effort[i]);
      }
    }
  }

  if (msg.name.empty()) return;
  joint_state_pub_->publish(msg);
}

inline void HardwareDriverNode::publishDCFeedback() {
  geom::Twist msg;
  msg.linear.x  = feedback_dc.linear;
  msg.angular.z = feedback_dc.angular;
  vel_state_pub_->publish(msg);
}

inline void HardwareDriverNode::diagTick() {
  auto changed = sysStats.amountUpdated();
  bool force = (++diag_ticks_since_pub_ >= diag_force_ticks_);

  if (changed || force) {
    diag_ticks_since_pub_ = 0;
    publishDiagnostics();
  }
}

// -----------------------------------------------------------------------------
inline void HardwareDriverNode::publishDiagnostics() {
  diag::DiagnosticArray array_msg;
  array_msg.header.stamp = this->now();
  
  auto snap = sysStats.consume_updated();

  for(auto &stat : snap){
    diag::DiagnosticStatus instance;
    instance.level = stat.level;
    instance.hardware_id = std::move(stat.hardware_id);
    instance.message = std::move(stat.message);
    instance.name = std::move(stat.name);
    for(auto &kv : stat.values){
      diag::KeyValue kv_msg;
      kv_msg.key = std::move(kv.first);
      kv_msg.value = std::move(kv.second);
      instance.values.push_back(std::move(kv_msg));
    }
    array_msg.status.push_back(instance);
  }

  error_state_pub_->publish(array_msg);
}