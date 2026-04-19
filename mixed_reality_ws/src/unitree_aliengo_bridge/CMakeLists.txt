cmake_minimum_required(VERSION 3.5)
project(unitree_aliengo_bridge)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(include)

add_executable(aliengo_bridge_node
  src/aliengo_bridge_node.cpp
  src/tether_collision_checker.cpp
  src/data_logger.cpp
  src/main.cpp
)

ament_target_dependencies(aliengo_bridge_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  rclcpp_components
  jsoncpp
)

install(TARGETS
  aliengo_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
