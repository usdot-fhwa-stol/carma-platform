
# Copyright (C) 2022 LEIDOS.
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.5)
project(carma_wm)

# Declare carma package and check ROS version
find_package(carma_cmake_common REQUIRED)
carma_check_ros_version(2)
carma_package()

## Find dependencies using ament auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost REQUIRED)

# Name build targets
set(node_lib carma_wm_lib)

# Includes
include_directories(
  include
)

# Build
ament_auto_add_library(${node_lib} SHARED
        src/MapConformer.cpp
        src/CARMAWorldModel.cpp
        src/WMListener.cpp
        src/WMListenerWorker.cpp
        src/Geometry.cpp
        src/WorldModelUtils.cpp
        src/TrafficControl.cpp
        src/IndexedDistanceMap.cpp
        src/collision_detection.cpp
        src/SignalizedIntersectionManager.cpp
)

target_link_libraries(
        ${node_lib}
        ${Boost_LIBRARIES}
)

ament_auto_add_executable(map_update_logger_node 
        src/MapUpdateLoggerNode.cpp 
)
# Register component
rclcpp_components_register_nodes(${node_lib} "MapUpdateLogger")
target_link_libraries(map_update_logger_node
        ${node_lib}
        ${Boost_LIBRARIES}
)

# Testing
if(BUILD_TESTING)  

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies() # This populates the ${${PROJECT_NAME}_FOUND_TEST_DEPENDS} variable

  ament_add_gtest(test_carma_wm 
    test/GeometryTest.cpp
    test/CARMAWorldModelTest.cpp
    test/WMListenerWorkerTest.cpp
    test/SignalizedIntersectionManagerTest.cpp
    test/CollisionDetectionTest.cpp
    test/IndexedDistanceMapTest.cpp
    test/MapConformerTest.cpp
    test/TrafficControlTest.cpp
    test/WMTestLibForGuidanceTest.cpp
    test/WorldModelUtilsTest.cpp
  )
  ament_target_dependencies(test_carma_wm ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
  target_link_libraries(test_carma_wm ${node_lib})

  ament_add_gtest(segfault
        test/SegFaultTest.cpp
  )
  ament_target_dependencies(segfault ${${PROJECT_NAME}_FOUND_TEST_DEPENDS})
  target_link_libraries(segfault ${node_lib})

endif()


# Install
ament_auto_package(
        INSTALL_TO_SHARE 
)
