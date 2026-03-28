cmake_minimum_required(VERSION 3.22)
project(aesir_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

add_executable(arm_command_node src/arm_command_node.cpp)

add_library(${PROJECT_NAME} SHARED src/topic_bridge_hardware.cpp)

ament_target_dependencies(arm_command_node
  rclcpp 
  geometry_msgs 
  sensor_msgs
  moveit_ros_planning_interface)

install(TARGETS arm_command_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware
  hardware_interface
  pluginlib
  sensor_msgs
)

install(PROGRAMS
  scripts/test_teleop.py
  scripts/test_teleop_joints.py
  DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(hardware_interface aesir_plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
