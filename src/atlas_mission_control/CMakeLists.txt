cmake_minimum_required(VERSION 3.0.2)
project(atlas_mission_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  hybrid_navigation
)

catkin_package()

catkin_install_python(PROGRAMS
  nodes/mission_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
