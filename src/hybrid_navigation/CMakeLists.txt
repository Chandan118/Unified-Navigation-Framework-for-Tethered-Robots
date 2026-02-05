cmake_minimum_required(VERSION 3.0.2)
project(hybrid_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  visualization_msgs
  tf
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  TetherStatus.msg
  SceneComplexity.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    rospy 
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    visualization_msgs
    tf
    tf2_ros
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/sensor_fusion_node.py
  nodes/scene_recognition_node.py
  nodes/hybrid_planner_node.py
  nodes/fuzzy_controller_node.py
  nodes/ga_optimizer_node.py
  nodes/tether_tension_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
