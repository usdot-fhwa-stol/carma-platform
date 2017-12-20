#!/bin/bash

# Get the Process Id of the roslaunch file as an argument
launch_pid=$1
echo $launch_pid

#TODO make this more secure with somekind of validation on the target process

# Kill the roslaunch file
kill -INT $launch_pid
