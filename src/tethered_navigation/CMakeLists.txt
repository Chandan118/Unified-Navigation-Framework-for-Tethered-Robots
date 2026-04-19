cmake_minimum_required(VERSION 3.8)
project(tethered_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find all necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ROS 2 message generation
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TetherState.msg"
  "msg/SwarmMetrics.msg"
  "msg/YieldingDecision.msg"
  DEPENDENCIES geometry_msgs std_msgs nav_msgs
)

# Define the C++ executable
add_executable(navigator_node src/navigator_node.cpp)

# Specify include directories for the executable
target_include_directories(navigator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link the executable against its dependencies
ament_target_dependencies(navigator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
  ${PROJECT_NAME}__rosidl_typesupport
)

# Link against rosidl_typesupport_cpp for message support
target_link_libraries(navigator_node
  rosidl_typesupport_cpp
)

# --- Installation Rules ---

# Install the C++ executable
install(TARGETS
  navigator_node
  DESTINATION lib/${PROJECT_NAME})

# Install all Python scripts as programs
install(PROGRAMS
  scripts/arduino_interface_node.py
  scripts/camera_obstacle_detector.py
  scripts/fuzzy_controller.py
  scripts/initial_pose_publisher.py
  scripts/jetson_camera_publisher.py
  scripts/rplidar_serial_reset.py
  scripts/rplidar_managed_runner.py
  scripts/safety_cmd_vel_filter.py
  scripts/small_robot_navigator.py
  scripts/train_obstacle_model.py
  scripts/swarm_coordinator.py
  scripts/tether_state_broadcaster.py
  scripts/swarm_simulator.py
  scripts/experiment_runner.py
  scripts/experiment_data_logger.py
  scripts/analyze_results.py
  scripts/enhanced_fuzzy_controller.py
  DESTINATION lib/${PROJECT_NAME})

# Install additional directories (worlds, swarm launch files)
install(DIRECTORY
  worlds
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME})

# This macro must be at the end
ament_package()