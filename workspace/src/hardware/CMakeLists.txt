cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy              REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(std_msgs           REQUIRED)
find_package(geometry_msgs      REQUIRED)
find_package(control_msgs       REQUIRED)
find_package(trajectory_msgs    REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ---------------------------------------------------------------------------
# Custom message generation
# ---------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointControl.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# ---------------------------------------------------------------------------
# dc_motors executable
# ---------------------------------------------------------------------------
add_executable(dc_motors src/dc_motors.cpp)

# Expose the include/ directory so headers are found without path prefixes
target_include_directories(dc_motors PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(dc_motors
  rclcpp
  std_msgs
  trajectory_msgs
  sensor_msgs
  geometry_msgs
  control_msgs
)

rosidl_target_interfaces(dc_motors
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  dc_motors
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motor_node.py
  scripts/jointjog_mux.py
  scripts/serial_sender.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers so other packages can consume them if needed
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()