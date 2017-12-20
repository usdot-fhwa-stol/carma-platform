#!/bin/bash

# Set JAVA_HOME and update PATH
export JAVA_HOME=/usr/local/java/jdk1.8.0_131
export PATH=${PATH}:${JAVA_HOME}/bin

# Source ROS (Must be called after ROS_HOME assignment)
source /opt/ros/kinetic/setup.bash

# Source ROSJava
source /home/mcconnelms/rosjava/devel/setup.bash

# Source platform
source /home/mcconnelms/to13_ws/devel/setup.bash

# Ensure ROS_HOSTNAME is not set as we will be using ROS_IP
unset ROS_HOSTNAME

# Ensure ROS_IP is correct
export ROS_IP=192.168.218.138

# Ensure ROS_MASTER_URI is correct
export ROS_MASTER_URI=http://192.168.218.138:11311/

# Launch platform
roslaunch --pid=/opt/carma/launch.pid carma saxton_cav.launch mock_dsrc:=true mock_can:=true mock_radar:=true mock_pinpoint:=true mock_srx_controller:=true

