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
  pluginlib
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
add_library(
  omni_car
  SHARED
  controller/omni_car_controller.cpp
)

target_compile_features(omni_car PUBLIC cxx_std_17)
target_include_directories(omni_car PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controller/include>
  $<INSTALL_INTERFACE:include/omni_car>
)

ament_target_dependencies(
  omni_car PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface omni_car.xml)

# INSTALL
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/gazebo description/rviz description/meshes
  DESTINATION share/omni_car
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/omni_car
)

install(TARGETS omni_car
  EXPORT export_omni_car
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

target_link_libraries(force_plugin
  PRIVATE gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  PRIVATE gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
)

## Test
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(omni_car_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_omni_car_launch test/test_view_robot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_omni_car HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
