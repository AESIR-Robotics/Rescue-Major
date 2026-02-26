#pragma once

#include <algorithm>
//#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include "hardware/msg/joint_control.hpp"

#include "commands.hpp"
#include "protocol_handler_i2c.hpp"

using namespace CommandsNC;

// =============================================================================
// Declaration
// =============================================================================

class HardwareDriverNode : public rclcpp::Node {
public:
  HardwareDriverNode();
  void tick();

private:
  int32_t radToSteps(double rad) const;
  double  stepsToRad(int32_t steps) const;

  void generateCallbacks();
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void jointCommandCallback(const hardware::msg::JointControl::SharedPtr msg);
  void publishJointFeedback();

  Protocol_Handler_I2C stepper_micro{this->get_logger()};

  int steps_per_revolution{40000};
  int steppers{4};

  std::vector<std::string> joint_names{"flipper_0", "flipper_1", "flipper_2",
                                       "flipper_3"};

  bool updated_pos{false};
  bool updated_vel{false};

  std::vector<double> feedback_joint_positions;
  std::vector<double> feedback_joint_velocities;
  std::vector<double> feedback_joint_efforts;

  std::vector<double> in_joint_positions;
  std::vector<double> in_joint_velocities;
  std::vector<double> in_joint_efforts; // reserved for future use

  // Tick state machine
  enum class TickState {
    IDLE,
    PREPARE_COMMANDS,
    WAIT_RESPONSE,
    PUBLISH_STATE,
    ERROR_RECOVERY
  };

  TickState tick_state_{TickState::IDLE};
  int wait_counter_{0};
  const int max_wait_ticks_{3};

  int ticks_since_poll_{0};
  const int poll_interval_ticks_{3};

  std::vector<int32_t> last_sent_pos_int_;
  std::vector<int32_t> last_sent_spd_int_;

  std::mutex joints_tick_mutex_;

  // ROS interfaces
  rclcpp::QoS qos_cmd_{rclcpp::QoS(1).best_effort()};
  rclcpp::QoS qos_feedback_{rclcpp::QoS(10).reliable()};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<hardware::msg::JointControl>::SharedPtr joint_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};


// =============================================================================
// Inline definitions
// =============================================================================

inline HardwareDriverNode::HardwareDriverNode() : Node("hardware_node") {
  // --- Parameters ------------------------------------------------------------
  this->declare_parameter<std::string>("i2c_port", "/dev/i2c-7");
  this->declare_parameter<int>("i2c_address", 0x30);
  this->declare_parameter<int>("flipper_revolution", 40000);
  this->declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"flipper_0", "flipper_1", "flipper_2", "flipper_3"});
  this->declare_parameter("steppers", 4);

  std::string i2c_port_;
  int slave_addr_{};

  this->get_parameter("i2c_port", i2c_port_);
  this->get_parameter("i2c_address", slave_addr_);
  this->get_parameter("flipper_revolution", steps_per_revolution);
  this->get_parameter("joint_names", joint_names);
  this->get_parameter("steppers", steppers);

  stepper_micro.init(i2c_port_, slave_addr_);

  // --- State vector init -----------------------------------------------------
  const size_t n = joint_names.size();
  feedback_joint_positions.assign(n, 0.0);
  feedback_joint_velocities.assign(n, 0.0);
  feedback_joint_efforts.assign(n, 0.0);

  in_joint_positions.assign(n, 0.0);
  in_joint_velocities.assign(n, 0.0);
  in_joint_efforts.assign(n, 0.0);

  last_sent_pos_int_.assign(n, std::numeric_limits<int32_t>::min());
  last_sent_spd_int_.assign(n, std::numeric_limits<int32_t>::min());

  generateCallbacks();

  // --- Subscriptions / publishers --------------------------------------------
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "hardware_node/cmd_vel", qos_cmd_,
      std::bind(&HardwareDriverNode::cmdVelCallback, this,
                std::placeholders::_1));

  joint_cmd_sub_ = this->create_subscription<hardware::msg::JointControl>(
      "hardware_node/joint_command", qos_cmd_,
      std::bind(&HardwareDriverNode::jointCommandCallback, this,
                std::placeholders::_1));

  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "hardware_node/joint_states", qos_feedback_);
}

inline void HardwareDriverNode::tick() {
  using positionW = WriteInst<WriteCommandsNC::POSITION>;
  using speedW    = WriteInst<WriteCommandsNC::SPEED>;

  stepper_micro.sendQueue();
  stepper_micro.readPending();

  switch (tick_state_) {

  case TickState::IDLE:
    if (!stepper_micro.connected()) {
      tick_state_ = TickState::ERROR_RECOVERY;
      break;
    }

    if (ticks_since_poll_++ >= poll_interval_ticks_) {
      ticks_since_poll_ = 0;
      tick_state_       = TickState::PREPARE_COMMANDS;
    } else {
      bool changed = false;
      {
        std::lock_guard<std::mutex> lock{joints_tick_mutex_};
        for (size_t i = 0;
             i < static_cast<size_t>(steppers) &&
             i < in_joint_positions.size();
             ++i) {
          if (radToSteps(in_joint_positions[i]) != last_sent_pos_int_[i] ||
              radToSteps(in_joint_velocities[i]) != last_sent_spd_int_[i]) {
            changed = true;
            break;
          }
        }
      }
      if (changed) {
        tick_state_ = TickState::PREPARE_COMMANDS;
      }
    }
    break;

  case TickState::PREPARE_COMMANDS: {
    std::unordered_map<int32_t, uint8_t> pos_groups;
    std::unordered_map<int32_t, uint8_t> spd_groups;
    std::unordered_map<int32_t, float>   spd_value;

    {
      std::lock_guard<std::mutex> lock{joints_tick_mutex_};
      for (size_t i = 0;
           i < static_cast<size_t>(steppers) &&
           i < in_joint_positions.size();
           ++i) {
        int32_t pos_key =
            static_cast<int32_t>(radToSteps(in_joint_positions[i]) %
                                 steps_per_revolution);
        int32_t spd_key = radToSteps(in_joint_velocities[i]);

        uint8_t bit = static_cast<uint8_t>(1u << i);
        pos_groups[pos_key] |= bit;
        spd_groups[spd_key] |= bit;

        float spd_f = static_cast<float>(in_joint_velocities[i] /
                                         (2.0 * M_PI) * steps_per_revolution);
        spd_value.insert_or_assign(spd_key, spd_f);
      }

      for (size_t i = 0;
           i < static_cast<size_t>(steppers) &&
           i < in_joint_positions.size();
           ++i) {
        last_sent_pos_int_[i] = radToSteps(in_joint_positions[i]);
        last_sent_spd_int_[i] = radToSteps(in_joint_velocities[i]);
      }
    }

    for (auto &p : pos_groups) {
      auto pkt = positionW::packet{p.second, p.first};
      stepper_micro.addCommand(std::make_unique<positionW>(pkt));
    }

    for (auto &s : spd_groups) {
      auto pkt = speedW::packet{s.second, spd_value[s.first]};
      stepper_micro.addCommand(std::make_unique<speedW>(pkt));
    }

    stepper_micro.addCommand(
        std::make_unique<ReadInst<ReadCommandsNC::POSITION>>());
    stepper_micro.addCommand(
        std::make_unique<ReadInst<ReadCommandsNC::SPEED>>());

    wait_counter_ = 0;
    tick_state_   = TickState::WAIT_RESPONSE;
  } break;

  case TickState::WAIT_RESPONSE:
    if (++wait_counter_ >= max_wait_ticks_) {
      tick_state_ = TickState::PUBLISH_STATE;
    }
    break;

  case TickState::PUBLISH_STATE:
    publishJointFeedback();
    tick_state_ = TickState::IDLE;
    break;

  case TickState::ERROR_RECOVERY:
    if (stepper_micro.connected()) {
      tick_state_ = TickState::IDLE;
      break;
    }
    if (++wait_counter_ >= max_wait_ticks_) {
      if (stepper_micro.reconnect()) {
        tick_state_ = TickState::IDLE;
      }
      wait_counter_ = 0;
    }
    break;
  }
}

inline int32_t HardwareDriverNode::radToSteps(double rad) const {
  return static_cast<int32_t>(
      std::llround(rad / (2.0 * M_PI) * steps_per_revolution));
}

inline double HardwareDriverNode::stepsToRad(int32_t steps) const {
  return static_cast<double>(steps) * (2.0 * M_PI) / steps_per_revolution;
}

inline void HardwareDriverNode::generateCallbacks() {
  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::ReadCommand::POSITION,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<
            ReadCommandsNC::ReadCommand::POSITION>(
            data, size, [&](const auto &info) {
              std::apply(
                  [&](const auto &...args) {
                    size_t i{0};
                    ((feedback_joint_positions[i++] =
                          stepsToRad(static_cast<int32_t>(args))),
                     ...);
                    updated_pos = true;
                  },
                  info);
            });
        if (!ok) {
          RCLCPP_WARN(this->get_logger(),
                      "Position return size mismatch: expected %zu, got %zu",
                      tuple_size(ReadCommandsNC::packetReturn<
                                 ReadCommandsNC::POSITION>::type{}),
                      size);
        }
      });

  stepper_micro.read_callbacks.emplace(
      ReadCommandsNC::ReadCommand::SPEED,
      [this](const uint8_t *data, size_t size) {
        bool ok = ReadCommandsNC::dispatch_one<ReadCommandsNC::SPEED>(
            data, size, [&](const auto &info) {
              std::apply(
                  [&](const auto &...args) {
                    size_t i{0};
                    ((feedback_joint_velocities[i++] =
                          static_cast<double>(args) * (2.0 * M_PI) /
                          steps_per_revolution),
                     ...);
                    updated_vel = true;
                  },
                  info);
            });
        if (!ok) {
          RCLCPP_WARN(this->get_logger(),
                      "Speed return size mismatch: expected %zu, got %zu",
                      tuple_size(ReadCommandsNC::packetReturn<
                                 ReadCommandsNC::SPEED>::type{}),
                      size);
        }
      });
}

inline void HardwareDriverNode::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  // TODO: implement cmd_vel → motor command mapping
  (void)msg;
}

inline void HardwareDriverNode::jointCommandCallback(
    const hardware::msg::JointControl::SharedPtr msg) {
  std::lock_guard<std::mutex> lock{joints_tick_mutex_};
  for (size_t i = 0; i < msg->joint_names.size(); ++i) {
    const std::string &name = msg->joint_names[i];
    auto it = std::find(joint_names.begin(), joint_names.end(), name);

    if (it == joint_names.end()) {
      RCLCPP_WARN(this->get_logger(), "Unknown joint name: %s", name.c_str());
      continue;
    }

    auto joint_index = std::distance(joint_names.begin(), it);
    if (joint_index < steppers) {
      in_joint_positions[joint_index]  = msg->position[i];
      in_joint_velocities[joint_index] = msg->velocity[i];
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid joint name format: %s",
                  msg->joint_names[i].c_str());
    }
  }
}

inline void HardwareDriverNode::publishJointFeedback() {
  // TODO: add staleness timeout for feedback
  if (updated_pos || updated_vel) {
    std::lock_guard<std::mutex> lock{joints_tick_mutex_};
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name         = joint_names;
    msg.position     = feedback_joint_positions;
    msg.velocity     = feedback_joint_velocities;
    msg.effort       = feedback_joint_efforts;
    joint_state_pub_->publish(msg);

    updated_pos = updated_vel = false;
  }
}