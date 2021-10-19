cmake_minimum_required(VERSION 3.5)
project(system_controller)


find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake REQUIRED)
find_package(carma_cmake_common REQUIRED)
find_package(carma_ros2_utils REQUIRED)
find_package(carma_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_lifecycle_manager REQUIRED)

set(executable_name ${PROJECT_NAME})
set(library_name ${executable_name}_core)

set(dependencies
  carma_cmake_common
  carma_ros2_utils
  rclcpp
  ros2_lifecycle_manager
  lifecycle_msgs
)

set(lib_cpp src/system_controller_node.cpp)
carma_define_ament_library(${library_name} "${dependencies}" "${lib_cpp}")

rclcpp_components_register_nodes(${library_name} "system_controller::SystemControllerNode")


set(exec_cpp src/main.cpp)
set(exec_deps
  ${dependencies}
  ${library_name}
)
carma_define_ament_executable(${executable_name} "${exec_deps}" "${exec_cpp}")

install(DIRECTORY config/
  DESTINATION config/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()