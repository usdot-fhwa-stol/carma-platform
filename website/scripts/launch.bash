#!/bin/bash

#  Copyright (C) 2018 LEIDOS.
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

# Kill all ROS related processes
pkill -f ros

# Source ROS (Must be called after ROS_HOME assignment)
# Assumes ROS Java was installed as package and included in this source
source /opt/ros/kinetic/setup.bash

# Source platform and set ros environment variables
source /opt/carma/app/bin/setup.bash

# Set ros to run from /opt/carma instead of home directory
export ROS_HOME=/opt/carma/.ros

# Remove bad launch.pid file if it exists
rm /opt/carma/launch.pid

# Launch platform
rosBagRecord=$1
roslaunch --pid=/opt/carma/launch.pid carma saxton_cav.launch use_rosbag:=$rosBagRecord
