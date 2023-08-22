#!/bin/bash

while true
    do
        # Get the flag that indiciates whether a ROS 1 rosbag should be recorded
        use_ros1_rosbag="$(rosparam get /use_ros1_rosbag)"
        if [ "$use_ros1_rosbag" == "false" ]
        then
            echo "A ROS 1 rosbag will not be recorded"
            exit 0
        fi

        # Get the regex of the excluded topics
        excluded_topics="$(rosparam get exclude_regex)"
        no_exclusions="$(rosparam get no_exclusions)"
        if [ ! -z "$excluded_topics" ] || [ "$no_exclusions" == "true" ]
        then 
            # Run the rosbag record command
            rosbag record -o /opt/carma/logs/ --lz4 -a -x "$excluded_topics"
            exit 0
        fi
    done