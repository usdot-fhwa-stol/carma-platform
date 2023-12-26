
# Copyright (C) 2020-2021 LEIDOS.
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

cmake_minimum_required(VERSION 3.5.1)
project(hardcoded_params)

find_package(carma_cmake_common REQUIRED)
carma_package()

find_package(ros_environment REQUIRED)

set(ROS_VERSION $ENV{ROS_VERSION})

if(${ROS_VERSION} EQUAL 2) # ROS 2
        ## Find dependencies using ament auto
        find_package(ament_cmake_auto REQUIRED)
        ament_auto_find_build_dependencies()

        # Includes
        # Currently this is a header only library so nothing else is needed here
        include_directories(
                include
        )

        # Install
        ament_auto_package()
        
else() # ROS 1

        find_package(catkin REQUIRED)

        catkin_package(
                INCLUDE_DIRS include
        )

        include_directories(
                include
                ${catkin_INCLUDE_DIRS}
        )

        install(DIRECTORY include/${PROJECT_NAME}/
                DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        )

endif()
