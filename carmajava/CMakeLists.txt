#
# Copyright (C) 2018-2020 LEIDOS.
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
#

##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(carma)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED
	rosjava_build_tools
        rosjava_messages #genjava was here
        cav_msgs
        cav_srvs
        j2735_msgs
        autoware_msgs
        derived_object_msgs
        radar_msgs
        rosjava_utils
)
# Enable integration tests using python
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/transform_server_integration.test)
#endif()

# Set the gradle targets you want catkin's make to run by default, e.g.
#   catkin_rosjava_setup(installApp)
# Note that the catkin_create_rosjava_xxx scripts will usually automatically
# add tasks to this for you when you create subprojects.
catkin_rosjava_setup(installApp publish)


# make sure messages get built first if genjava is enabled
add_dependencies(gradle-${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

catkin_package()

##############################################################################
# Installation
##############################################################################

# Change this to match the maven group name you have specified in the
# allprojects closure the root build.gradle
install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/gov/dot/fhwa/saxton/${PROJECT_NAME}/
        DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/gov/dot/fhwa/saxton/${PROJECT_NAME})

# Instal Binaries from Gradle Build folder
install(DIRECTORY
        launch
        routes
        scripts
        rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
        mock_drivers/build
        mock_drivers/config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mock_drivers)
install(DIRECTORY
        ui/config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ui)
install(DIRECTORY
        test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
