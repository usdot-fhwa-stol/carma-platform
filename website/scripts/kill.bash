#!/bin/bash

# Get the Process Id of the roslaunch file as an argument
launch_pid=$1
echo $launch_pid

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
