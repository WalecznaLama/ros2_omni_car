cmake_minimum_required(VERSION 3.8)
project(omni_car_controller)

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  pluginlib
  rcpputils
  controller_interface
  realtime_tools
  geometry_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Define the library
add_library(
  omni_car_controller
  SHARED
  src/omni_car_controller.cpp
)

target_compile_features(omni_car_controller PUBLIC cxx_std_17)
target_include_directories(omni_car_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/omni_car_controller>
)

ament_target_dependencies(
  omni_car_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export controllers
pluginlib_export_plugin_description_file(controller_interface omni_car_controller.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(TARGETS omni_car_controller
  EXPORT export_omni_car_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(EXPORT export_omni_car_controller
  FILE omni_car_controllerTargets.cmake
  NAMESPACE omni_car_controller::
  DESTINATION lib/cmake/omni_car_controller
)

# Testing setup
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Exports
ament_export_targets(export_omni_car_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
