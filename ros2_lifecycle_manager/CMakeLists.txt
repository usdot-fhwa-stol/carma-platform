cmake_minimum_required(VERSION 3.5)
project(ros2_lifecycle_manager)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(action_msgs REQUIRED)
find_package(carma_msgs REQUIRED)

set(dependencies
    action_msgs
    carma_msgs
    lifecycle_msgs
    rclcpp
    rclcpp_action
    rclcpp_lifecycle
    rcutils
    rclcpp_components
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/ros2_lifecycle_manager.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)


install(DIRECTORY include/
  DESTINATION include/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_pytest REQUIRED)
#   find_package(launch_testing_ament_cmake REQUIRED)
#   ament_lint_auto_find_test_dependencies()

#   find_package(ament_cmake_gtest REQUIRED)
# endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
