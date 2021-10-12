cmake_minimum_required(VERSION 3.5)
project(subsystem_controllers)


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

include_directories(
  include
  ${carma_ros2_utils_INCLUDE_DIRS}
  ${carma_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_lifecycle_manager_INCLUDE_DIRS}
)

set(base_dependencies
  carma_cmake_common
  carma_ros2_utils
  rclcpp
  ros2_lifecycle_manager
  lifecycle_msgs
)

add_library(base_subsystem_controller SHARED
  src/base_subsystem_controller/base_subsystem_controller.cpp
)
ament_target_dependencies(base_subsystem_controller
  ${base_dependencies}
)

set(dependency_libs
  ${carma_cmake_common_LIBRARIES}
  ${carma_ros2_utils_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${ros2_lifecycle_manager_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  base_subsystem_controller
)

add_subdirectory(src/v2x_controller)
add_subdirectory(src/localization_controller)
add_subdirectory(src/guidance_controller)
add_subdirectory(src/environment_perception_controller)
add_subdirectory(src/drivers_controller)


install(TARGETS base_subsystem_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY include/
  DESTINATION include/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_export_include_directories(include)
ament_export_libraries( # TODO can we put this export line in each subdirectory?
  base_subsystem_controller 
  v2x_controller
  localization_controller
  guidance_controller
  environment_perception_controller
  drivers_controller
)
ament_export_dependencies(${base_dependencies})
ament_package()