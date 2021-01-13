#!/bin/bash

while true
    do
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