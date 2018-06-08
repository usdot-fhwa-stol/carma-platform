cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(socketcan_bridge)

find_package(catkin REQUIRED
  COMPONENTS
    can_msgs
    roscpp
    socketcan_interface
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    socketcan_to_topic
    topic_to_socketcan
    socketcan_recover_ctrl
  CATKIN_DEPENDS
    can_msgs
    roscpp
    socketcan_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# socketcan_to_topic
add_library(socketcan_to_topic
  src/socketcan_to_topic.cpp
)
target_link_libraries(socketcan_to_topic
  ${catkin_LIBRARIES}
)
add_dependencies(socketcan_to_topic
  ${catkin_EXPORTED_TARGETS}
)

# topic_to_socketcan
add_library(topic_to_socketcan
  src/topic_to_socketcan.cpp
)
target_link_libraries(topic_to_socketcan
  ${catkin_LIBRARIES}
)
add_dependencies(topic_to_socketcan
  ${catkin_EXPORTED_TARGETS}
)

# socketcan_recover_ctrl
add_library(socketcan_recover_ctrl
  src/socketcan_recover_ctrl.cpp
)
target_link_libraries(socketcan_recover_ctrl
  ${catkin_LIBRARIES}
)
add_dependencies(socketcan_recover_ctrl
  ${catkin_EXPORTED_TARGETS}
)

# socketcan_to_topic_node
add_executable(socketcan_to_topic_node
  src/socketcan_to_topic_node.cpp
)
target_link_libraries(socketcan_to_topic_node
  socketcan_to_topic
  socketcan_recover_ctrl
  ${catkin_LIBRARIES}
)

# topic_to_socketcan_node
add_executable(topic_to_socketcan_node
  src/topic_to_socketcan_node.cpp
)
target_link_libraries(topic_to_socketcan_node
  topic_to_socketcan
  socketcan_recover_ctrl
  ${catkin_LIBRARIES}
)

# socketcan_bridge_node
add_executable(socketcan_bridge_node
  src/socketcan_bridge_node.cpp
)
target_link_libraries(socketcan_bridge_node
  topic_to_socketcan
  socketcan_to_topic
  socketcan_recover_ctrl
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    socketcan_bridge_node
    socketcan_recover_ctrl
    socketcan_to_topic
    socketcan_to_topic_node
    topic_to_socketcan
    topic_to_socketcan_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint REQUIRED)

  roslint_cpp()
  #for use with catkin_make roslint_socketcan_bridge

  # unit test for the can_msgs::Frame and can::Frame types.
  catkin_add_gtest(test_conversion
    test/test_conversion.cpp
  )
  target_link_libraries(test_conversion
    topic_to_socketcan
    socketcan_to_topic
    socketcan_recover_ctrl
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(test_to_socketcan
    test/to_socketcan.test
    test/to_socketcan_test.cpp
  )
  target_link_libraries(test_to_socketcan
    topic_to_socketcan
    ${catkin_LIBRARIES}
  )

  add_rostest_gtest(test_to_topic
    test/to_topic.test
    test/to_topic_test.cpp
  )
  target_link_libraries(test_to_topic
    socketcan_to_topic
    ${catkin_LIBRARIES}
  )

endif()
