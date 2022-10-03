#!/bin/bash

#  Copyright (C) 2018-2021 LEIDOS.
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

#rosbridge_suite is a ROS meta-package including all the rosbridge packages.
git clone https://github.com/usdot-fhwa-stol/rosbridge_suite --branch ros2