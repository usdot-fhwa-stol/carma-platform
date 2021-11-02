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

set(base_dependencies
  carma_cmake_common
  carma_ros2_utils
  rclcpp
  ros2_lifecycle_manager
  lifecycle_msgs
)

set(base_lib_cpp src/base_subsystem_controller/base_subsystem_controller.cpp)
carma_define_ament_library(base_subsystem_controller "${base_dependencies}" "${base_lib_cpp}")

set(dependency_libs
  ${base_dependencies}
  base_subsystem_controller
)

# V2X Subsystem
carma_define_ament_library(v2x_controller_core "${dependency_libs}" src/v2x_controller/v2x_controller_node.cpp)
set(v2x_controller_deps v2x_controller_core ${dependency_libs})
carma_define_ament_executable(v2x_controller "${v2x_controller_deps}" src/v2x_controller/main.cpp)
rclcpp_components_register_nodes(v2x_controller_core "subsystem_controllers::V2XControllerNode")

# Localization Subsystem
carma_define_ament_library(localization_controller_core "${dependency_libs}" src/localization_controller/localization_controller.cpp)
set(locz_controller_deps localization_controller_core ${dependency_libs})
carma_define_ament_executable(localization_controller "${locz_controller_deps}" src/localization_controller/main.cpp)
rclcpp_components_register_nodes(localization_controller_core "subsystem_controllers::LocalizationControllerNode")

# Guidance Subsystem
carma_define_ament_library(guidance_controller_core "${dependency_libs}" src/guidance_controller/guidance_controller.cpp)
set(guidance_controller_deps guidance_controller_core ${dependency_libs})
carma_define_ament_executable(guidance_controller "${guidance_controller_deps}" src/guidance_controller/main.cpp)
rclcpp_components_register_nodes(guidance_controller_core "subsystem_controllers::GuidanceControllerNode")

# Environment Subsystem
carma_define_ament_library(environment_perception_controller_core "${dependency_libs}" src/environment_perception_controller/environment_perception_controller.cpp)
set(environment_controller_deps environment_perception_controller_core ${dependency_libs})
carma_define_ament_executable(environment_perception_controller "${environment_controller_deps}" src/environment_perception_controller/main.cpp)
rclcpp_components_register_nodes(environment_perception_controller_core "subsystem_controllers::EnvironmentPerceptionControllerNode")

# Driver Subsystem
carma_define_ament_library(drivers_controller_core "${dependency_libs}" src/drivers_controller/drivers_controller_node.cpp)
set(driver_controller_deps drivers_controller_core ${dependency_libs})
carma_define_ament_executable(drivers_controller "${driver_controller_deps}" src/drivers_controller/main.cpp)
rclcpp_components_register_nodes(drivers_controller_core "subsystem_controllers::DriversControllerNode")

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

install(DIRECTORY config/
  DESTINATION config/
)

ament_package()