cmake_minimum_required(VERSION 3.8)
project(vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# ADD THIS LINE for Python support
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)  
find_package(std_msgs REQUIRED)   
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(ament_cmake_core REQUIRED)

message(STATUS "ament_cmake_core version: ${ament_cmake_core_VERSION}")

if(ament_cmake_core_VERSION VERSION_GREATER_EQUAL "2.0.0")
    # Jazzy+
    add_compile_definitions(ROS2_JAZZY)
else()
    # Humble
    add_compile_definitions(ROS2_HUMBLE)
endif()


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Command.srv"
  DEPENDENCIES std_msgs sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()
endif()

add_executable(video_stream_publisher src/video_stream.cpp)
ament_target_dependencies(video_stream_publisher 
  rclcpp 
  sensor_msgs 
  std_srvs
  example_interfaces
  cv_bridge 
  OpenCV
  image_transport
)

install(TARGETS
  video_stream_publisher
  DESTINATION lib/${PROJECT_NAME})
  

# Use ament_python_install_package to install Python code
install(PROGRAMS
  scripts/sensors_master.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/best.pt
  DESTINATION lib/${PROJECT_NAME}
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(video_stream_publisher ${cpp_typesupport_target})

add_dependencies(video_stream_publisher ${PROJECT_NAME}__rosidl_generator_cpp)

install(DIRECTORY
  srv
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# This command must be at the end of the file
ament_package()
