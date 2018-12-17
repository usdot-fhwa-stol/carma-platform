#!/bin/bash

# Kill all ROS related processes
pkill -f ros

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
