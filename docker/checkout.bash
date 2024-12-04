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
BRANCH=develop  # The script will use this unless the -b flag updates it
while [[ $# -gt 0 ]]; do
      arg="$1"
      case $arg in
            -b|--branch)
                  BRANCH=$2
                  shift
                  shift
            ;;
            -r|--root)
                  dir=$2
                  shift
                  shift
            ;;
      esac
done

cd "${dir}"/src

git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-msgs.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-utils.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-messenger.git --branch "${BRANCH}"
# Get humble branch of message filters which supports template Node arguments (foxy version supports rclcpp::Node only)
git clone --depth=1 https://github.com/usdot-fhwa-stol/carma-message-filters.git --branch "${BRANCH}"
git clone --depth=1 https://github.com/usdot-fhwa-stol/multiple_object_tracking --branch "${BRANCH}"
git clone --depth=1 https://github.com/ros2/rosbag2 --branch humble

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

# Clone the humble branch of ros2_tracing in order to enable certain analyses of CARMA Platform
# made possible through collected trace data, such as analyzing ROS 2 callback durations.
git clone -b humble https://github.com/ros2/ros2_tracing

#rosbridge_suite is a ROS meta-package including all the rosbridge packages.
# NOTE: clone -b flag is used instead of --branch to avoid hook rewriting it
git clone -b ros2 https://github.com/usdot-fhwa-stol/rosbridge_suite

# TODO: Remove V2X-Hub Depedency (CAR-6029)
git clone -b master --depth 1 https://github.com/etherealjoy/qhttpengine.git

git clone -b 7.6.0 --depth 1 https://github.com/usdot-fhwa-OPS/V2X-Hub.git
cd V2X-Hub
git config core.sparsecheckout true
git sparse-checkout init
git sparse-checkout set ext/ccserver
