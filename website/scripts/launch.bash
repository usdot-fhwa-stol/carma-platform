#!/bin/bash

# Set JAVA_HOME and update PATH
#export JAVA_HOME=/usr/local/java/jdk1.8.0_131
#export PATH=${PATH}:${JAVA_HOME}/bin

# Source ROS (Must be called after ROS_HOME assignment)
# Assumes ROS Java was installed as package and included in this source
source /opt/ros/kinetic/setup.bash

# Source platform and set ros environment variables
source /opt/carma/app/bin/setup.bash

# Ensure ROS_HOSTNAME is not set as we will be using ROS_IP
unset ROS_HOSTNAME

# Ensure ROS_IP is correct
export ROS_IP=192.168.88.10

# Ensure ROS_MASTER_URI is correct
export ROS_MASTER_URI=http://192.168.88.10:11311/

# Launch platform
roslaunch --pid=/opt/carma/launch.pid carma saxton_cav.launch
