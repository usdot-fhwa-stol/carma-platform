cmake_minimum_required(VERSION 3.5)
project(system_controller)


find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Includes
include_directories(
  include
)

# Build
set(executable_name ${PROJECT_NAME})
set(library_name ${executable_name}_core)

ament_auto_add_library(${library_name} SHARED src/system_controller_node.cpp)
rclcpp_components_register_nodes(${library_name} "system_controller::SystemControllerNode")

ament_auto_add_executable(${executable_name} src/main.cpp)
target_link_libraries(${executable_name} ${library_name})

# Test
if(BUILD_TESTING)

  add_subdirectory(test)

endif()

# Install
ament_auto_package(
  INSTALL_TO_SHARE config
)