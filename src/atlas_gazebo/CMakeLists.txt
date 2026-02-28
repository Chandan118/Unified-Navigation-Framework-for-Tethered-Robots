cmake_minimum_required(VERSION 3.8)
project(atlas_gazebo)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY launch worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
