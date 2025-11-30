cmake_minimum_required(VERSION 3.8)
project(teleoperation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

# Create executable for C++ command server
add_executable(command_server src/command_server.cpp)

target_link_libraries(command_server 
    ${Boost_LIBRARIES}
    pthread
)

# Include websocketpp headers (header-only library)
target_include_directories(command_server PRIVATE
    /usr/include/websocketpp
)

ament_target_dependencies(command_server
    rclcpp
    std_msgs
    sensor_msgs
)

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
  command_server
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/command_server.py
  scripts/server.py
  scripts/audio_reproducer.py
  scripts/test_audio.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  GUI 
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()