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

set -ex

# Build the software and its dependencies

###
# ROS1 installation
###
# Source the autoware installation

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/autoware.ai/ros/install/setup.bash
fi

cd ~/carma_ws

sudo mkdir -p /opt/carma # Create install directory
sudo chown carma /opt/carma # Set owner to expose permissions for build
sudo chgrp carma /opt/carma # Set group to expose permissions for build

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    if [[ ! -z "$ROS1_PACKAGES" ]]; then
        echo "Incrementally building ROS1 packages: $ROS1_PACKAGES"
        colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS1_PACKAGES --allow-overriding $ROS1_PACKAGES 
    else
        echo "Build type is incremental but no ROS1 packages specified, skipping ROS1 build..."
    fi
else
    echo "Building all ROS1 CARMA Components"
    colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
echo "Build of ROS1 CARMA Components Complete"

###
# ROS2 installation
###
# Source the ROS2 autoware installation
source /home/carma/catkin/setup.bash
if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    echo "Sourcing previous build for incremental build start point..."
    source /opt/carma/install_ros2/setup.bash
else
    echo "Sourcing base image for full build..."
    source /opt/autoware.ai/ros/install_ros2/setup.bash
fi

cd ~/carma_ws

echo "Building ROS2 CARMA Components"

if [[ ! -z "$ROS1_PACKAGES$ROS2_PACKAGES" ]]; then
    if [[ ! -z "$ROS2_PACKAGES" ]]; then
        echo "Incrementally building ROS2 packages: $ROS2_PACKAGES"
        colcon build --install-base /opt/carma/install_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-above $ROS2_PACKAGES
    else
        echo "Build type is incremental but no ROS2 packages specified, skipping ROS2 build..."
    fi
else
    echo "Building all ROS2 components..."
    colcon build  --install-base /opt/carma/install_ros2 --build-base build_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
fi

echo "Build of ROS 2 CARMA Components Complete"
