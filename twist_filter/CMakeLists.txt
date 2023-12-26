# Modifications:
# - Add unit testing for CARMA velocity limit code and adjust CMake to account
#   for refactoring required in that process
#    - Kyle Rush
#    - 9/13/2020
# - Refactored to use ament for ros2
#    - 11/16/2022

cmake_minimum_required(VERSION 3.5)
project(twist_filter)

find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

# Name build targets
set(node_exec twist_filter_node_exec)
set(node_lib twist_filter_node)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/velocity_limit.cpp
        src/accel_limit.cpp
        src/twist_filter.cpp
)

ament_auto_add_executable(${node_exec} 
        src/twist_filter_node.cpp 
)

# Register component
rclcpp_components_register_nodes(${node_lib} "twist_filter::TwistFilter")

# All locally created targets will need to be manually linked
# ament auto will handle linking of external dependencies
target_link_libraries(${node_exec}
        ${node_lib}
        ${Boost_LIBRARIES}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_twist_filter test/test_twist_filter.cpp)

  ament_target_dependencies(test_twist_filter ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})

  target_link_libraries(test_twist_filter ${node_lib})

endif()

# Install
ament_auto_package(
        INSTALL_TO_SHARE launch
)
