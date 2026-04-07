#pragma once

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "pluginlib/class_list_macros.hpp"

#include "hardware/msg/joint_control.hpp"

#include <vector>
#include <mutex>

namespace aesir_plugin
{

class TopicBridgeHardware : public hardware_interface::SystemInterface
{
private:
  rclcpp::Node::SharedPtr node_;

  // Publisher for the custom JointControl message
  rclcpp::Publisher<hardware::msg::JointControl>::SharedPtr command_pub_;

  // Subscriber to update per-joint acceleration at runtime
  // Topic: /hardware_bridge/set_acceleration
  // Message: std_msgs/Float64MultiArray (one value per joint, in order)
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr accel_sub_;

  // Memory for Position, Velocity, and Effort
  std::vector<double> hw_commands_;     // position commands
  std::vector<double> hw_vel_cmds_;     // velocity commands
  std::vector<double> hw_eff_cmds_;     // effort commands

  std::vector<double> hw_states_;       // position states
  std::vector<double> hw_vel_states_;   // velocity states
  std::vector<double> hw_eff_states_;   // effort states

  // Per-joint acceleration — updated at runtime via topic
  std::vector<double> hw_acc_cmds_;
  std::mutex acc_mutex_;  // Protect acceleration vector from concurrent access

  static constexpr double DEFAULT_ACCELERATION = 3.14159265;

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(TopicBridgeHardware)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override
  {
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    const size_t nr_joints = info_.joints.size();

    hw_states_.resize(nr_joints, 0.0);
    hw_vel_states_.resize(nr_joints, 0.0);
    hw_eff_states_.resize(nr_joints, 0.0);

    hw_commands_.resize(nr_joints, 0.0);
    hw_vel_cmds_.resize(nr_joints, 0.0);
    hw_eff_cmds_.resize(nr_joints, 0.0);

    // Default acceleration for every joint
    hw_acc_cmds_.resize(nr_joints, DEFAULT_ACCELERATION);

    // -----------------------------------------------------------------
    // Node setup
    // -----------------------------------------------------------------
    node_ = std::make_shared<rclcpp::Node>("hw_bridge_node");

    // Publisher: JointControl commands
    command_pub_ = node_->create_publisher<hardware::msg::JointControl>(
      "/commands_hardware", 10);

    // Subscriber: runtime acceleration update
    // Send a Float64MultiArray with exactly nr_joints values.
    accel_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/hardware_bridge/set_acceleration",
      10,
      [this, nr_joints](const std_msgs::msg::Float64MultiArray::SharedPtr msg)
      {
        if (msg->data.size() != nr_joints) {
          RCLCPP_WARN(
            node_->get_logger(),
            "set_acceleration: expected %zu values, got %zu — ignoring.",
            nr_joints, msg->data.size());
          return;
        }
        std::lock_guard<std::mutex> lock(acc_mutex_);
        for (size_t i = 0; i < nr_joints; ++i) {
          hw_acc_cmds_[i] = msg->data[i];
        }
        RCLCPP_INFO(node_->get_logger(), "Acceleration updated for all joints.");
      });

    RCLCPP_INFO(node_->get_logger(),
      "TopicBridgeHardware initialised with %zu joints. "
      "Publish Float64MultiArray to '/hw_bridge/set_acceleration' to update acceleration.",
      nr_joints);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ---------------------------------------------------------------------------
  // Export state interfaces: Position, Velocity, Effort per joint
  // ---------------------------------------------------------------------------
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_states_[i]);
      state_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_eff_states_[i]);
    }
    return state_interfaces;
  }

  // ---------------------------------------------------------------------------
  // Export command interfaces: Position, Velocity, Effort per joint
  // (Acceleration is internal — not a ros2_control command interface)
  // ---------------------------------------------------------------------------
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); ++i) {
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_vel_cmds_[i]);
      command_interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &hw_eff_cmds_[i]);
    }
    return command_interfaces;
  }

  // ---------------------------------------------------------------------------
  // Read: fake feedback — states mirror commands so MoveIt sees instant motion
  // ---------------------------------------------------------------------------
  hardware_interface::return_type read(
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/) override
  {
    for (size_t i = 0; i < hw_states_.size(); ++i) {
      hw_states_[i]     = hw_commands_[i];
      hw_vel_states_[i] = hw_vel_cmds_[i];
      hw_eff_states_[i] = hw_eff_cmds_[i];
    }

    // Spin the node once to process incoming acceleration messages
    rclcpp::spin_some(node_);

    return hardware_interface::return_type::OK;
  }

  // ---------------------------------------------------------------------------
  // Write: publish JointControl with position, velocity, acceleration, effort
  // ---------------------------------------------------------------------------
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & /*period*/) override
  {
    hardware::msg::JointControl msg;
    msg.header.stamp = time;

    std::vector<double> acc_snapshot;
    {
      std::lock_guard<std::mutex> lock(acc_mutex_);
      acc_snapshot = hw_acc_cmds_;
    }

    for (size_t i = 0; i < info_.joints.size(); ++i) {
      msg.joint_names.push_back(info_.joints[i].name);
      msg.position.push_back(hw_commands_[i]);
      msg.velocity.push_back(hw_vel_cmds_[i]);
      msg.acceleration.push_back(acc_snapshot[i]);
      msg.effort.push_back(hw_eff_cmds_[i]);
    }

    command_pub_->publish(msg);
    return hardware_interface::return_type::OK;
  }
};

}  // namespace aesir_plugin

PLUGINLIB_EXPORT_CLASS(aesir_plugin::TopicBridgeHardware, hardware_interface::SystemInterface)