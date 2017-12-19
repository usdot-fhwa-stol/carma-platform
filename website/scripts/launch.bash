#!/bin/bash

# Source platform
source ~/to13_ws/devel/setup.bash

# Ensure ROS_IP is correct
export ROS_IP=192.168.218.138

# Launch platform
roslaunch --pid=~/to13_ws/pid.txt carma saxton_cav.launch mock_dsrc:=true mock_can:=true mock_radar:=true mock_pinpoint:=true mock_srx_controller:=true

