#include <rclcpp/rclcpp.hpp>

#include "hardware_driver_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<HardwareDriverNode>();

  constexpr int LOOP_HZ = 500;
  rclcpp::Rate rate(LOOP_HZ);

  RCLCPP_INFO(node->get_logger(), "DC Motors node started");

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->tick();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}