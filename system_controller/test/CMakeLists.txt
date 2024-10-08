find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_pytest REQUIRED)
find_package(ros2_lifecycle_manager REQUIRED)
find_package(std_srvs REQUIRED)

find_package(launch_testing_ament_cmake REQUIRED)

# These tests has been temporarily disabled to support Continuous Improvement (CI) processes.
# Related GitHub Issue: <https://github.com/usdot-fhwa-stol/carma-platform/issues/2335>

#add_launch_test(
#  launch_test_nominal.py
#)

#add_launch_test(
#  launch_test_node_crash.py
#)

set(dependencies ${dependencies} ros2_lifecycle_manager std_srvs carma_ros2_utils)


add_executable(test_carma_lifecycle_node
  test_node.cpp
)

ament_target_dependencies(test_carma_lifecycle_node ${dependencies} )

target_include_directories(test_carma_lifecycle_node
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  TARGETS test_carma_lifecycle_node
  # EXPORT should not be needed for unit tests
  LIBRARY DESTINATION lib/${PROJECT_NAME} # In Ament, Executables are installed to lib/${PROJECT_NAME} not bin
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)
