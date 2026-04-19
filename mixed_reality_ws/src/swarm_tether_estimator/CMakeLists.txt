cmake_minimum_required(VERSION 3.5)
project(swarm_tether_estimator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(jsoncpp REQUIRED)

# Include directories
include_directories(include)

# Build the tether estimator node
add_executable(tether_estimator_node
  src/tether_estimator_node.cpp
  src/tether_publisher.cpp
)

ament_target_dependencies(tether_estimator_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  rclcpp_components
  jsoncpp
)

# Install executables
install(TARGETS
  tether_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
