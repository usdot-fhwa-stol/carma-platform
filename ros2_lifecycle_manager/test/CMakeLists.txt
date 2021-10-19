add_executable(test_lifecycle_node
  test_node.cpp
)

target_link_libraries(test_lifecycle_node
  ${lifecycle_msgs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
)


ament_add_gtest_executable(test_lifecycle_gtest
  test_lifecycle_manager.cpp
)

target_link_libraries(test_lifecycle_gtest
  ${PROJECT_NAME}
)

ament_target_dependencies(test_lifecycle_gtest
  ${dependencies}
)

ament_add_test(test_lifecycle
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/launch_lifecycle_test.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 20
  ENV
    TEST_EXECUTABLE=$<TARGET_FILE:test_lifecycle_gtest>
)

install(
  TARGETS test_lifecycle_node
  DESTINATION lib/${PROJECT_NAME}
)

