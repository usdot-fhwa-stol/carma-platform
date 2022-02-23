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


# Build the software and its dependencies

set -ex
 
###
# ROS1 installation
###
# Source the autoware installation
source /opt/autoware.ai/ros/install/setup.bash

cd ~/carma_ws

sudo mkdir -p /opt/carma # Create install directory
sudo chown carma /opt/carma # Set owner to expose permissions for build
sudo chgrp carma /opt/carma # Set group to expose permissions for build

echo "Building ROS1 CARMA Components"

colcon build --install-base /opt/carma/install --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build of ROS1 CARMA Components Complete"

###
# ROS2 installation
###
# Source the ROS2 autoware installation
source /home/carma/catkin/setup.bash
source /opt/autoware.ai/ros/install_ros2/setup.bash

cd ~/carma_ws

echo "Building ROS2 CARMA Components"

colcon build --install-base /opt/carma/install_ros2 --build-base build_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "Build of ROS2 CARMA Components Complete"
