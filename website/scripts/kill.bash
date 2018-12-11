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

# Get the Process Id of the roslaunch file as an argument
launch_pid=$1
echo $launch_pid

# Assumes ROS Java was installed as package and included in this source
source /opt/ros/kinetic/setup.bash

# Source platform and set ros environment variables
source /opt/carma/app/bin/setup.bash

# Publish system alert (-1 is once mode which adds a 3sec delay)
rostopic pub -1 /saxton_cav/system_alert cav_msgs/SystemAlert "type: 6
description: 'Shutdown requested by UI'"

# Wait for additional 3 seconds for roslaunch shutdown then kill process
sleep 3

#TODO make this more secure with somekind of validation on the target process

# Kill the roslaunch file
kill -INT $launch_pid
