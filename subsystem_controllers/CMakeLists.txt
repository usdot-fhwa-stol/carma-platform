cmake_minimum_required(VERSION 3.5)
project(subsystem_controllers)


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

set(base_lib base_lib_cpp)

# Build
ament_auto_add_library(${base_lib} SHARED src/base_subsystem_controller/base_subsystem_controller.cpp)

# V2X Subsystem
ament_auto_add_library(v2x_controller_core SHARED src/v2x_controller/v2x_controller_node.cpp)
rclcpp_components_register_nodes(v2x_controller_core "subsystem_controllers::V2XControllerNode")
target_link_libraries(v2x_controller_core ${base_lib})
ament_auto_add_executable(v2x_controller src/v2x_controller/main.cpp)
target_link_libraries(v2x_controller v2x_controller_core ${base_lib})

# Localization Subsystem
ament_auto_add_library(localization_controller_core SHARED src/localization_controller/localization_controller.cpp)
rclcpp_components_register_nodes(localization_controller_core "subsystem_controllers::LocalizationControllerNode")
target_link_libraries(localization_controller_core ${base_lib})
ament_auto_add_executable(localization_controller src/localization_controller/main.cpp)
target_link_libraries(localization_controller localization_controller_core ${base_lib})

# Guidance Subsystem
ament_auto_add_library(guidance_controller_core SHARED src/guidance_controller/guidance_controller.cpp)
rclcpp_components_register_nodes(guidance_controller_core "subsystem_controllers::GuidanceControllerNode")
target_link_libraries(guidance_controller_core ${base_lib})
ament_auto_add_executable(guidance_controller src/guidance_controller/main.cpp)
target_link_libraries(guidance_controller guidance_controller_core ${base_lib})

# Environment Subsystem
ament_auto_add_library(environment_perception_controller_core SHARED src/environment_perception_controller/environment_perception_controller.cpp)
rclcpp_components_register_nodes(environment_perception_controller_core "subsystem_controllers::EnvironmentPerceptionControllerNode")
target_link_libraries(environment_perception_controller_core ${base_lib})
ament_auto_add_executable(environment_perception_controller src/environment_perception_controller/main.cpp)
target_link_libraries(environment_perception_controller environment_perception_controller_core ${base_lib})

# Driver Subsystem
ament_auto_add_library(drivers_controller_core SHARED src/drivers_controller/drivers_controller_node.cpp)
rclcpp_components_register_nodes(drivers_controller_core "subsystem_controllers::DriversControllerNode")
target_link_libraries(drivers_controller_core ${base_lib})
ament_auto_add_executable(drivers_controller src/drivers_controller/main.cpp)
target_link_libraries(drivers_controller drivers_controller_core ${base_lib})

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE config
)