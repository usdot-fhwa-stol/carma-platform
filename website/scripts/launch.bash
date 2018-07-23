#!/bin/bash

# Kill all ROS related processes
pkill -f ros

# Source ROS (Must be called after ROS_HOME assignment)
# Assumes ROS Java was installed as package and included in this source
source /opt/ros/kinetic/setup.bash

# Source platform and set ros environment variables
source /opt/carma/app/bin/setup.bash

# Ensure ROS_HOSTNAME is not set as we will be using ROS_IP
unset ROS_HOSTNAME

# Ensure ROS_IP is correct
export ROS_IP=192.168.0.4

# Ensure ROS_MASTER_URI is correct
export ROS_MASTER_URI=http://192.168.0.4:11311/

# Set ros to run from /opt/carma instead of home directory
export ROS_HOME=/opt/carma/.ros

# Remove bad launch.pid file if it exists
rm /opt/carma/launch.pid

# Launch platform
rosBagRecord=$1
roslaunch --pid=/opt/carma/launch.pid carma saxton_cav.launch use_rosbag:=$rosBagRecord
