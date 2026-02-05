cmake_minimum_required(VERSION 3.0.2)
project(atlas_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    gazebo_ros
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

