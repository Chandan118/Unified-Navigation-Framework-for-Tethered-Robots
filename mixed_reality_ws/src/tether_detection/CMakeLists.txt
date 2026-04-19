cmake_minimum_required(VERSION 3.5)
project(tether_detection)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

include_directories(include)

add_executable(tether_detector_node
  src/tether_detector_node.cpp
  src/line_extraction.cpp
)

ament_target_dependencies(tether_detector_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

install(TARGETS
  tether_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
