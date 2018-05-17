#
# Copyright (C) 2018 LEIDOS.
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
)
# Enable integration tests using python
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  add_rostest(test/transform_server_integration.test)
#endif()

generate_rosjava_messages(
  PACKAGES
    cav_msgs
    cav_srvs
)
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
        guidance/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/guidance)
install(DIRECTORY
        interfacemgr/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/interfacemgr)
install(DIRECTORY
        lateral_control_driver/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lateral_control_driver)
install(DIRECTORY
        message/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/message)
install(DIRECTORY
        mock_drivers/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mock_drivers)
install(DIRECTORY
        roadway/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/roadway)
install(DIRECTORY
        route/build
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/route)
install(DIRECTORY
        test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test)
