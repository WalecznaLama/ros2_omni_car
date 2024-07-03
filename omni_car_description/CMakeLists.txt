cmake_minimum_required(VERSION 3.5)
project(omni_car_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and XACRO files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install meshes files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
