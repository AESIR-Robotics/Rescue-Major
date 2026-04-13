#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sched.h>

#include "ros_node.hpp"

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  auto node = std::make_shared<HardwareDriverNode>();

  constexpr int LOOP_HZ = 250;
  rclcpp::Rate rate(LOOP_HZ);

  RCLCPP_INFO(node->get_logger(), "DC Motors node started");

  // Kernel priority
  struct sched_param sp{};
  sp.sched_priority = 80;
  if (sched_setscheduler(0, SCHED_FIFO, &sp) != 0) {
    perror("sched_setscheduler"); 
    RCLCPP_WARN(node->get_logger(), "Could not set kernel priority for system calls, i2c messaging might not meet timing garantees");
  }

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->tick();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}