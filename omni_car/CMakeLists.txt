cmake_minimum_required(VERSION 3.16)
project(omni_car)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  kdl_parser
  rclcpp
  trajectory_msgs
  rcpputils
  controller_interface
  realtime_tools
  geometry_msgs

  gz-cmake3
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
# INSTALL
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/gazebo description/rviz description/meshes
            bringup/launch bringup/config 
            worlds
            maps
  DESTINATION share/omni_car
)

## GZ PLUGIN
gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

gz_find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

add_library(
  force_plugin 
  SHARED 
  gz_plugin/force_plugin.cpp
)

target_compile_features(force_plugin PUBLIC cxx_std_17)
target_include_directories(force_plugin PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/gz_plugin/include>
  $<INSTALL_INTERFACE:include/force_plugin>
)

ament_target_dependencies(
  force_plugin PUBLIC
  geometry_msgs
  rclcpp
)

target_link_libraries(force_plugin PUBLIC
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

install(
  TARGETS force_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## Test
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(omni_car_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_omni_car_launch test/test_view_robot_launch.py)
endif()

ament_package()
