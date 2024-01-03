#!/bin/bash

#  Copyright (C) 2018-2023 LEIDOS.
#
#  Licensed under the Apache License, Version 2.0 (the "License"); you may not
#  use this file except in compliance with the License. You may obtain a copy of
#  the License at
#
#  http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
#  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
#  License for the specific language governing permissions and limitations under
#  the License.

# CARMA packages checkout script
# Optional argument to set the root checkout directory with no ending '/' default is '~'

set -exo pipefail

dir=~
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -d|--develop)
                  BRANCH=develop
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

cd ${dir}/src


# clone carma repos

if [[ "$BRANCH" = "develop" ]]; then
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch  $BRANCH
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch $BRANCH
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-messenger.git --branch $BRANCH
else
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch develop
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch develop
      git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-messenger.git --branch develop
fi

# Get humble branch of message filters which supports template Node arguments (foxy version supports rclcpp::Node only)
git clone https://github.com/usdot-fhwa-stol/carma-message-filters.git --branch develop

git clone https://github.com/usdot-fhwa-stol/multiple_object_tracking --branch develop

# add astuff messages
# NOTE: The ibeo_msgs package is ignored because on build the cmake files in that package run a sed command
#       which can make them incompatible with a new ros version after a source switch
git clone https://github.com/astuff/astuff_sensor_msgs

cd astuff_sensor_msgs
git checkout 41d5ef0c33fb27eb3c9ba808b51332bcce186a83

# Disable ibeo_msgs
cd ibeo_msgs
echo "" > COLCON_IGNORE
cd ../astuff_sensor_msgs
echo "" > COLCON_IGNORE

cd ../

# Clone the foxy branch of ros2_tracing in order to enable certain analyses of CARMA Platform
# made possible through collected trace data, such as analyzing ROS 2 callback durations.
git clone -b foxy https://github.com/ros2/ros2_tracing

#rosbridge_suite is a ROS meta-package including all the rosbridge packages.
# NOTE: clone -b flag is used instead of --branch to avoid hook rewriting it
git clone -b ros2 https://github.com/usdot-fhwa-stol/rosbridge_suite

# The feature/integrate-carma branch of rosbag2 includes improvements that were not possible to backport into the foxy branch
# of rosbag2. These rosbag2 packages will replace the originally built foxy rosbag2 packages.
# NOTE: Additional information regarding the rosbag2 improvements on this branch are included in the forked repository's README.
git clone -b carma-develop https://github.com/usdot-fhwa-stol/rosbag2

# Novatel OEM7 Driver
# NOTE: This is required since otherwise this image will not contain the novatel_oem7_msgs package, and a missing ROS 2 message package
#       can cause ROS 2 rosbag logging to fail in Foxy.
# Related GitHub discussion for fix that was not backported to Foxy: https://github.com/ros2/rosbag2/pull/858
git clone https://github.com/novatel/novatel_oem7_driver.git ${dir}/src/novatel_oem7_driver -b ros2-dev
# Checkout verified commit
cd ${dir}/src/novatel_oem7_driver
git checkout 3055e220bb9715b59c3ef53ab0aba05a495d9d5
# Ignore novatel_oem7_driver package; only novatel_oem7_msgs is required
cd ${dir}/src/novatel_oem7_driver/src/novatel_oem7_driver
echo "" > COLCON_IGNORE
cd ${dir}/src


cd ${dir}/src

# git clone --branch master --depth 1 https://github.com/nitroshare/qhttpengine.git
git clone -b master --depth 1 https://github.com/etherealjoy/qhttpengine.git
git clone -b develop --depth 1 https://github.com/usdot-fhwa-OPS/V2X-Hub.git
cd V2X-Hub
git config core.sparsecheckout true
git sparse-checkout init
git sparse-checkout set ext/ccserver
