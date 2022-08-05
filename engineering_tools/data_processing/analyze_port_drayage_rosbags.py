#!/usr/bin/python3

#  Copyright (C) 2021 LEIDOS.
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

from inspect import TPFLAGS_IS_ABSTRACT
import sys
import csv
import matplotlib.pyplot as plt
import rospy
import rosbag # To import this, run the following command: "pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag rospkg"
import datetime
import math

# HOW TO USE SCRIPT:
# Run the following in a terminal to download dependencies:
#   sudo add-apt-repository ppa:deadsnakes/ppa
#   sudo apt-get update
#   sudo apt install python3.7
#   python3.7 -m pip install --upgrade pip
#   python3.7 -m pip install matplotlib
#   python3.7 -m pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag rospkg
#   python3.7 -m pip install lz4
#   python3.7 -m pip install roslz4 --extra-index-url https://rospypi.github.io/simple/
# In terminal, navigate to the directory that contains this python script and run the following:
#   python3.7 analyze_port_drayage_rosbags.py <path to folder containing Port Drayage Use Case .bag files>

def generate_speed_plot(bag, engagement_times, route_id):
    # Speed command: /hardware_interface/arbitrated_speed_commands: msg.speed
    # True Speed:     /hardware_interface/pacmod_parsed_tx/vehicle_speed_rpt: msg.vehicle_speed
    total_duration = 25
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]
    time_duration = rospy.Duration(total_duration)

    # Get the true vehicle speed and plot it
    first = True
    true_vehicle_speed_times = []
    true_vehicle_speeds = []
    # Note: This topic name assumes a pacmod controller is being used (freightliners or lexus)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/pacmod/parsed_tx/vehicle_speed_rpt'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue

        true_vehicle_speed_times.append((t-time_start).to_sec())
        true_vehicle_speeds.append(msg.vehicle_speed)

    first = True
    cmd_vehicle_speed_times = []
    cmd_vehicle_speeds = []
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/arbitrated_speed_commands'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue  
        
        cmd_vehicle_speed_times.append((t-time_start).to_sec())
        cmd_vehicle_speeds.append(msg.speed)

    fig, ax = plt.subplots()
    print(true_vehicle_speed_times[0])
    ax.plot(true_vehicle_speed_times, true_vehicle_speeds, 'b--', label='Actual Speed')
    ax.plot(cmd_vehicle_speed_times, cmd_vehicle_speeds, 'g:', label='Cmd Speed')
    ax.legend()
    ax.set_title("Speed (Cmd and Actual) for Route " + str(route_id))
    ax.set_xlabel("Time (seconds) since start of engagement")
    ax.set_ylabel("Vehicle Speed (m/s)")
    plt.show()

    return

def generate_steering_plot(bag, engagement_times, route_id):
    # Speed command: /hardware_interface/arbitrated_speed_commands: msg.speed
    # True Speed:     /hardware_interface/pacmod_parsed_tx/vehicle_speed_rpt: msg.vehicle_speed
    total_duration = 25
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]
    time_duration = rospy.Duration(total_duration)

    # Get the true vehicle speed and plot it
    first = True
    true_steering_times = []
    true_steering = []
    cmd_steering_times = []
    cmd_steering = []
    # Note: This topic name assumes a pacmod controller is being used (freightliners or lexus)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/pacmod/parsed_tx/steer_rpt'], start_time = time_start_engagement, end_time = time_end_engagement): #time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue

        true_steering_times.append((t-time_start).to_sec())
        true_steering.append(msg.output)
        cmd_steering_times.append((t-time_start).to_sec())
        cmd_steering.append(msg.command)


    fig, ax = plt.subplots()
    ax.plot(true_steering_times, true_steering, 'b--', label='Actual Steering (rad)')
    ax.plot(cmd_steering_times, cmd_steering, 'g:', label='Cmd Steering (rad)')
    ax.legend()
    ax.set_title("Steering Angle (Cmd and Actual) for Route " + str(route_id))
    ax.set_xlabel("Time (seconds) since start of engagement")
    ax.set_ylabel("Steering Angle (rad)")
    plt.show()

    return

# Helper Function: Get the original speed limit for the lanelets within the vehicle's route
# Note: Assumes that all lanelets in the route share the same speed limit prior to the first geofence CARMA Cloud message being processed.
def get_route_original_speed_limit(bag, time_test_start_engagement):
    # Initialize the return variable
    original_speed_limit = 0.0

    # Find the speed limit associated with the first lanelet when the system first becomes engaged
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_test_start_engagement):
        original_speed_limit = msg.speed_limit
        break

    return original_speed_limit

# Helper Function: Obtain the start and end times of each 'System Engaged' portion of the provided bag file.
#                  Function returns all start times and corresponding end times in separate lists.
def get_test_case_engagement_times(bag, num_engagement_sections):
    # Initialize system engagement start and end times
    engagement_start_times = []
    engagement_end_times = []

    # Loop through /guidance/state messages to determine each start and end time of engagement
    is_engaged = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/state']):
        # If entering engagement, track this start time
        if (msg.state == 4 and not is_engaged):
            time_start_engagement = t
            is_engaged = True

        # Store the last recorded engagement timestamp in case CARMA ends engagement before a new guidance
        #       state can be published.
        if (msg.state == 4):
            time_last_engaged = t
        else:
            if (is_engaged):
                engagement_start_times.append(time_start_engagement)
                engagement_end_times.append(time_last_engaged)
                is_engaged = False
    
    found_engagement_times = True
    if (len(engagement_start_times) != num_engagement_sections):
        print("Error: Bag file has " + str(len(engagement_start_times)) + " engagement start times, not " + str(num_engagement_sections))
        found_engagement_times = False
    else:
        print("Success: Bag file has " + str(len(engagement_start_times)) + " engagement start times, expected " + str(num_engagement_sections))
    if (len(engagement_end_times) != num_engagement_sections):
        print("Error: Bag file has " + str(len(engagement_end_times)) + " engagement end times, not " + str(num_engagement_sections))
        found_engagement_times = False
    else:
       print("Success: Bag file has " + str(len(engagement_end_times)) + " engagement end times, expected " + str(num_engagement_sections))

    return engagement_start_times, engagement_end_times, found_engagement_times

# Helper Function: Print out the times associated with the vehicle entering each new lanelet according to /guidance/route_state
def print_lanelet_entrance_times(bag, time_start_engagement):
    # Print out time vehicle enters each lanelet according to /guidance/route_state
    id = 0
    print("/guidance/route_state lanelet change times:")
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement):
        if msg.lanelet_id != id:
            print("Time: " + str(t.to_sec()) + "; Lanelet: " + str(msg.lanelet_id) + "; Speed Limit: " + str(msg.speed_limit))
            id = msg.lanelet_id
    
    return

###########################################################################################################
# Port Drayage 1: Vehicle achieves its target speed (+/- 2 mph of speed limit).
###########################################################################################################
def check_vehicle_reaches_steady_state(bag, engagement_times, speed_limit_ms):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # (m/s) Threshold offset of vehicle speed to speed limit to be considered at steady state
    threshold_speed_limit_offset = 0.89408 # 0.89408 m/s is 2 mph
    min_steady_state_speed = speed_limit_ms - threshold_speed_limit_offset
    max_steady_state_speed = speed_limit_ms + threshold_speed_limit_offset

    # Get the time that the vehicle reaches within the set offset of the speed limit (while system is engaged)
    has_reached_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement, end_time = time_end_engagement):
        current_speed = msg.twist.linear.x # Current vehicle speed in m/s
        if (max_steady_state_speed >= current_speed >= min_steady_state_speed):
            has_reached_steady_state = True
            break

    if has_reached_steady_state:
        print("PD-1-" + str(engagement_times[2]) + " Succeeded; vehicle reached +/- 0.89408 m/s (2 mph) of speed limit " + str(speed_limit_ms) + " m/s.")
        is_successful = True
    else:
        print("PD-1-" + str(engagement_times[2]) + " Failed; vehicle did not reach speed within +/- 0.89408 m/s (2 mph) of speed limit " + str(speed_limit_ms) + " m/s.")
        is_successful = False

    return is_successful

###########################################################################################################
# Port Drayage 4: The speed limit while not inside the Port or Staging Area is 20-25 mph. (8.94-11.176 m/s).
###########################################################################################################
def check_speed_limit_outside_port_and_staging_area(bag, engagement_times, lanelet_ids, min_speed_limit_ms, max_speed_limit_ms):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # Loop through /route_state topic to get the current speed limit for a given timestamp
    encountered_lanelet_ids_in_speed_limit_range = []
    id = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_end_engagement):
        
        # If a new lanelet ID is encountered, check that it is within the correct speed limit range and add it to list
        if msg.lanelet_id != id:
            id = msg.lanelet_id
            if (min_speed_limit_ms <= msg.speed_limit <= max_speed_limit_ms):
                encountered_lanelet_ids_in_speed_limit_range.append(id)
    
    # Check list of enocuntered lanelet IDs that satisfy speed limit range and check against expected list
    is_successful = False
    if encountered_lanelet_ids_in_speed_limit_range == lanelet_ids:
        print("PD-4-" + str(engagement_times[2]) + " Succeeded; all lanelets outside of Staging Area and Port had a speed limit between 8.94-11.18 m/s (20-25mph).")
        is_successful = True
    else:
        print("PD-4-" + str(engagement_times[2]) + " Failed; not all lanelets outside of Staging Area and Port had a speed limit between 8.94-11.18 m/s (20-25mph).")
        is_successful = False

    return is_successful

###########################################################################################################
# Port Drayage 19: The speed limit while inside the Port or Staging Area is 10 mph. (4.47 m/s).
###########################################################################################################
def check_speed_limit_inside_port_and_staging_area(bag, engagement_times, target_speed_limit_ms):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # Set acceptable speed limit range
    speed_limit_threshold_ms = 0.223 # Threshold (in m/s) the actual speed limit can be from the target speed limit
    min_speed_limit_ms = target_speed_limit_ms - speed_limit_threshold_ms
    max_speed_limit_ms = target_speed_limit_ms + speed_limit_threshold_ms

    # Loop through /route_state topic to get the current speed limit for a given timestamp
    id = 0
    is_successful = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_end_engagement):
        # If a lanelet is encountered with a speed limit outside the target range, this metric fails
        if not (min_speed_limit_ms <= msg.speed_limit <= max_speed_limit_ms):
            is_successful = False
            id = msg.lanelet_id
            print("PD-19-" + str(engagement_times[2]) + " Failed: Lanelet " + str(id) + " had speed limit of " + str(msg.speed_limit) + " m/s")
            break
    
    if is_successful:
        print("PD-19-" + str(engagement_times[2]) + " Succeeded; all speed limits were " + str(target_speed_limit_ms) + " m/s")
    
    return is_successful

###########################################################################################################
# Port Drayage 23: After the CMV is engaged on the route to its next destination, the CMV's actual trajectory
#                   will include an acceleration section. The average acceleration over the entire section
#                   shall be no less than 1 m/s^2, and the average acceleration over any 1-second portion
#                   of the section shall be no greater than 2.0 m/s^2.
###########################################################################################################
def check_acceleration_after_engagement(bag, engagement_times, target_speed_limit_ms):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # Set target speed (an offset below the target speed limit, since acceleration rate will decrease after this point)
    speed_limit_offset_ms = 0.89 # 0.89 m/s is 2 mph
    target_speed_ms = target_speed_limit_ms - speed_limit_offset_ms

    # Obtain timestamp associated with the start of the acceleration section
    time_start_accel = rospy.Time()
    speed_start_accel_ms = 0.0
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement):
        if msg.speed >= 0.1:
            speed_start_accel_ms = msg.speed * 0.277778 # Conversion kph to m/s
            time_start_accel = t
            break

    # Obtain timestamp associated with the end of the acceleration section 
    # Note: This is either the first deceleration after the start of the accel section or the moment 
    #       when the vehicle's speed reaches some offset of the speed limit.
    prev_speed = 0.0
    first = True
    time_end_accel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_accel):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        current_speed_ms = msg.speed * 0.27777 # Conversion of kph to m/s
        #if delta_speed <= 0 or (msg.speed*0.2777 >= target_speed_ms):
        if (msg.speed*0.2777 >= target_speed_ms):
            time_end_accel = t
            speed_end_accel_ms = current_speed_ms
            speed_end_accel_mph = speed_end_accel_ms * 2.2369
            #print("Speed at end of acceleration section: " + str(speed_end_accel_mph) + " mph")
            break

        prev_speed = msg.speed

    # Check the total average acceleration rate
    total_avg_accel = (speed_end_accel_ms - speed_start_accel_ms) / (time_end_accel-time_start_accel).to_sec()
    #print("Total average accel: " + str(total_avg_accel) + " m/s^2")

    # Check the 1-second window averages
    # Obtain average decelation over all 1-second windows in acceleration section
    one_second_duration = rospy.Duration(1.0)
    all_one_second_windows_successful = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_accel, end_time = (time_end_accel-one_second_duration)):
        speed_initial_ms = msg.speed * 0.277777 # Conversion from kph to m/s
        time_initial = t

        speed_final_ms = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = (time_initial+one_second_duration)):
            speed_final_ms = msg.speed * 0.277777 # Conversion from kph to m/s
            t_final = t
            break
        
        one_second_accel = (speed_final_ms - speed_initial_ms) / (t_final - time_initial).to_sec()

        if one_second_accel > 2.0:
            print("Failure: 1-second window accel rate was " + str(one_second_accel) + " m/s^2; end time was " + str((t_final - t_final).to_sec()) + " seconds after start of engagement.")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has accel rate at " + str(one_second_decel) + " m/s^2")

    # Print success/failure statements
    total_deceleration_rate_successful = False
    if total_avg_accel >= 1.0:
        print("PD-23-" + str(engagement_times[2]) + " (avg accel at start) Succeeded; total average acceleration was " + str(total_avg_accel) + " m/s^2")
        total_deceleration_rate_successful = True
    else:
        print("PD-23-" + str(engagement_times[2]) + " (avg accel at start) Failed; total average acceleration was " + str(total_avg_accel) + " m/s^2")
    
    if all_one_second_windows_successful:
        print("PD-23-" + str(engagement_times[2]) + " (1-second accel after start) Succeeded; no occurrences of 1-second average acceleration above 2.0 m/s^2")
    else:
        print("PD-23-" + str(engagement_times[2]) + " (1-second accel after start) Failed; at least one occurrence of 1-second average acceleration above 2.0 m/s^2")

    is_successful = False
    if total_deceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True

    return is_successful

###########################################################################################################
# Port Drayage 24: As the CMV approaches its destination, the CMV's actual trajectory will include a 
#                  deceleration section. The average deceleration over the entire section shall be no 
#                  less than 1 m/s^2, and the average deceleration over any 1-second portion of the section 
#                  shall be no greater than 2.0 m/s^2. 
###########################################################################################################
def check_deceleration_at_end_of_route(bag, engagement_times):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # Set target speed (an offset above 0 mph, since deceleration rate will decrease significantly after this point)
    target_speed_ms = 0.1 #0.89 # 0.89 m/s is 2 mph

    # Obtain the timestamp of the last speed increase before the system is disengaged; this is the start of deceleration
    prev_speed = 0.0
    first = True
    time_start_decel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement, end_time = time_end_engagement):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        # Track the most recent occurrence of acceleration
        delta_speed = msg.speed - prev_speed
        if delta_speed >= 0.001: # Speed >= 0.001 kph (0.0006 mph) considered an acceleration
            time_start_decel = t
            speed_start_decel_ms = msg.speed * 0.277777 # Conversion from kph to m/s
            #print("Time " + str(t.to_sec()) + " has speed " + str(speed_after_final_decel))
        
        prev_speed = msg.speed
        
    speed_start_decel_mph = speed_start_decel_ms * 2.23694 # 2.23694 mph is 1 m/s

    # Obtain timestamp associated with the end of the deceleration section 
    # Note: This is the moment when the vehicle's speed reaches some offset of 0 mph
    prev_speed = 0.0
    first = True
    time_end_decel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_decel):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        current_speed_ms = msg.speed * 0.27777 # Conversion of kph to m/s
        if (current_speed_ms <= target_speed_ms):
            time_end_decel = t
            speed_end_decel_ms = current_speed_ms
            speed_end_decel_mph = speed_end_decel_ms * 2.2369 # 2.2369 mph is 1 m/s
            #print("Speed at end of acceleration section: " + str(speed_end_accel_mph) + " mph")
            break

        prev_speed = msg.speed

    # Calculate the average deceleration across the full deceleration section
    total_average_decel = (speed_start_decel_ms - speed_end_decel_ms) / (time_end_decel - time_start_decel).to_sec()
    
    # Obtain average decelation over all 1-second windows in deceleration section
    one_second_duration = rospy.Duration(1.0)
    all_one_second_windows_successful = True
    max_one_second_decel = 0.0
    one_second_failures = 0
    first_one_second_failure = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_decel, end_time = (time_end_decel-one_second_duration)):
        speed_initial_ms = msg.speed * 0.277777 # Conversion from kph to m/s
        time_initial = t

        speed_final_ms = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = (time_initial+one_second_duration)):
            speed_final_ms = msg.speed * 0.277777 # Conversion from kph to m/s
            t_final = t
            break

        one_second_decel = abs((speed_initial_ms - speed_final_ms) / (t_final-time_initial).to_sec())

        if one_second_decel > max_one_second_decel:
            max_one_second_decel = one_second_decel
        if one_second_decel > 2.0:
            if first_one_second_failure:
                time_first_one_second_failure = t
                first_one_second_failure = False
            one_second_failures += 1
            #print("Failure: 1-second window had decel rate " + str(one_second_decel) + " m/s^2; end time was " + str((time_end_decel - t_final).to_sec()) + " seconds before complete stop")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has decel rate at " + str(one_second_decel) + " m/s^2")
    
    # Print success/failure statements
    total_deceleration_rate_successful = False
    if total_average_decel >= 1.0:
        print("PD-24-" + str(engagement_times[2]) + " (total decel at route end) succeeded; total average deceleration was " + str(total_average_decel) + " m/s^2; start speed: " + str(speed_start_decel_mph) + " mph")
        total_deceleration_rate_successful = True
    else:
        print("PD-24-" + str(engagement_times[2]) + " (total decel at route end) failed; total average deceleration was " + str(total_average_decel) + " m/s^2; start speed: " + str(speed_start_decel_mph) + " mph")
    
    if all_one_second_windows_successful:
        print("PD-24-" + str(engagement_times[2]) + " (1-second decel at route end) succeeded; no occurrences of 1-second average deceleration above 2.0 m/s^2; max: " + str(max_one_second_decel) + " m/s^2")
    else:
        print("PD-24-" + str(engagement_times[2]) + " (1-second decel at route end) failed; " + str(one_second_failures) + " failures starting " + str((time_end_decel - time_first_one_second_failure).to_sec()) + " sec before stop. Max: " + str(max_one_second_decel) + " m/s^2")


    is_successful = False
    if total_deceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True
    return is_successful

###########################################################################################################
# Port Drayage 25: After the CMV arrives to its destination, the UI shall successfully show a 
#                 "Route Completed" dialog within 3.0 seconds.
###########################################################################################################
def check_route_completed_notification(bag, engagement_times):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    # Obtain the timestamp associated with the vehicle coming to a complete stop
    time_complete_stop = rospy.Time()
    fifteen_secon_duration = rospy.Duration(15.0)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = (time_start_engagement+fifteen_secon_duration)):
        if msg.twist.linear.x <= 0.001: 
            time_complete_stop = t
            break

    # Obtain the timestamp associated with the 'ROUTE_COMPLETED' notification occurring
    time_route_completed_notification = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_event'], start_time = time_start_engagement):
        # Event == 3 is 'ROUTE_COMPLETED'
        if msg.event == 3:
            time_route_completed_notification = t
            break

    time_for_notification_to_occur = (time_route_completed_notification - time_complete_stop).to_sec()

    is_successful = False
    if time_for_notification_to_occur < 3.0:
        print("PD-25-" + str(engagement_times[2]) + " Succeeded; Route Completed notification occurred " + str(time_for_notification_to_occur) + " sec after complete stop.")
        is_successful = True
    else:
        print("PD-25-" + str(engagement_times[2]) + " Failed; Route Completed notification occurred " + str(time_for_notification_to_occur) + " sec after complete stop. (Less than 3 sec required)")
        is_successful = False

    return is_successful

###########################################################################################################
# Port Drayage 18: The vehicle is able to send an arrival message to the infrastructure.

# Port Drayage 9: The infrastructure is able to communicate the next destination to the vehicle.

# Port Drayage 20: The vehicle is able to receive mobility operations messages from the infrastructure.

# Port Drayage 10: The vehicle is able to receive the next destination from the infrastructure and 
#                  display a message on Web UI to the user to proceed to that destination point.

# Port Drayage 21: After the CMV has broadcasted its arrival message to V2XHub, the CMV shall receive a 
#                  valid response from V2XHub including the next instructed destination in less than 1.5 seconds.

# Port Drayage 22: After the CMV has received a message from V2XHub instructing it to proceed to its next destination, 
#                  the CMV shall successfully generate an active route to that destination in less than 3 seconds.
###########################################################################################################
def check_cmv_arrival_message(bag, engagement_times, arrival_operation, next_destination_operation):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    max_seconds_before_next_destination_received = 1.5 # 1.5 seconds is max time between broadcasting arrival message and receiving the next destination
    max_seconds_before_route_set = 3.0 # 3.0 seconds is max time between receiving a new destination and a route being set to that destination

    # Check whether the first 'carma/port_drayage' outgoing MobilityOperation message for this engagement section is the expected arrival message
    time_arrival_message = rospy.Time()
    pd_18_is_successful = False
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_start_engagement):
        if msg.strategy == "carma/port_drayage":
            if arrival_operation in msg.strategy_params:
                pd_18_is_successful = True
                time_arrival_message = t
                break

    # Check metrics related to the response from V2XHub, which shall include the next destination
    time_next_destination_received = rospy.Time()
    pd_20_is_successful = False
    pd_9_is_successful = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_operation'], start_time = time_arrival_message):
        if msg.strategy == "carma/port_drayage":
            if next_destination_operation in msg.strategy_params:
                pd_20_is_successful = True
                pd_9_is_successful = True
                time_next_destination_received = t
                break
    
    # Check whether destination message is received within the acceptable amount of time
    pd_21_is_successful = False
    time_between_arrival_and_next_destination_received = (time_next_destination_received - time_arrival_message).to_sec()
    if (time_next_destination_received - time_arrival_message).to_sec() < max_seconds_before_next_destination_received:
        pd_21_is_successful = True

    # Check that the Web UI received prompt to display a popup to the user for the new destination
    pd_10_is_successful = False
    time_between_new_destination_and_ui_popup = 0.0
    five_second_duration = rospy.Duration(5.0)
    for topic, msg, t in bag.read_messages(topics=['/ui/ui_instructions'], start_time = time_next_destination_received, end_time = time_next_destination_received + five_second_duration):
        if "A new Port Drayage route with operation type" in msg.msg:
            pd_10_is_successful = True
            time_between_new_destination_and_ui_popup = (t - time_next_destination_received).to_sec()
            break

    # Check that new route is set within 3.0 seconds of the new destination being received from V2XHub
    pd_22_is_successful = False
    ten_second_duration = rospy.Duration(10.0)
    time_route_is_set = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_event'], start_time = time_next_destination_received, end_time = time_next_destination_received + ten_second_duration):
        
        if msg.event == 2: # 2 is 'ROUTE_STARTED'
            time_route_is_set = t
            break
    
    time_between_new_destination_and_route_set = (time_route_is_set - time_next_destination_received).to_sec()
    if time_between_new_destination_and_route_set < max_seconds_before_route_set:
        pd_22_is_successful = True

    if pd_18_is_successful:
        print("PD-18-" + str(engagement_times[2]) + " Succeeded; arrival message for " + str(arrival_operation) + " broadcasted.")
    else:
        print("PD-18-" + str(engagement_times[2]) + " Failed; arrival message for " + str(arrival_operation) + " not broadcasted.")

    if pd_9_is_successful:
        print("PD-9-" + str(engagement_times[2]) + " Succeeded; CMV received a new destination from infrastructure.")
    else:
        print("PD-9-" + str(engagement_times[2]) + " Succeeded; CMV did not receive a new destination from infrastructure.")

    if pd_20_is_successful:
        print("PD-20-" + str(engagement_times[2] + 1) + " Succeeded; received message for next destination: " + str(next_destination_operation) + " broadcasted.")
    else:
        print("PD-20-" + str(engagement_times[2] + 1) + " Failed; arrival message for " + str(next_destination_operation) + " not broadcasted.")

    if pd_10_is_successful:
        print("PD-10-" + str(engagement_times[2] + 1) + " Succeeded; Web UI popup occurred " + str(time_between_new_destination_and_ui_popup) + " seconds after receiving new destination")
    else:
        print("PD-10-" + str(engagement_times[2] + 1) + " Failed; Web UI popup did not occur for new received destination.")

    if pd_21_is_successful:
        print("PD-21-" + str(engagement_times[2] + 1) + " Succeeded; time between arrival broadcasted and next destination received: " + str(time_between_arrival_and_next_destination_received) + " seconds")
    else:
        print("PD-21-" + str(engagement_times[2] + 1) + " Failed; time between arrival broadcasted and next destination received: " + str(time_between_arrival_and_next_destination_received) + " seconds")
    
    if pd_22_is_successful:
        print("PD-22-" + str(engagement_times[2] + 1) + " Succeeded; new route set " + str(time_between_new_destination_and_route_set) + " seconds after receiving new route.")
    else:
        print("PD-22-" + str(engagement_times[2] + 1) + " Failed; new route set " + str(time_between_new_destination_and_route_set) + " seconds after receiving new route.")

    return pd_18_is_successful, pd_9_is_successful, pd_20_is_successful, pd_10_is_successful, pd_21_is_successful, pd_22_is_successful

###########################################################################################################
# Port Drayage XX: STOPPING LOCATION
###########################################################################################################
def check_stop_location(bag, engagement_times, stop_location_in_map, max_distance, metric_number, vehicle_length):
    time_start_engagement = engagement_times[0]
    time_end_engagement = engagement_times[1]

    map_x_stop_location = stop_location_in_map[0]
    map_y_stop_location = stop_location_in_map[1]
    #print(str(map_x_stop_location) + ", " + str(map_y_stop_location))

    # Obtain the timestamp associated with the 'ROUTE_COMPLETED' notification occurring
    time_route_completed_notification = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_event'], start_time = time_start_engagement):
        # Event == 3 is 'ROUTE_COMPLETED'
        if msg.event == 3:
            time_route_completed_notification = t
            break
    
    # Get the map location associated with the vehicle when it stopped
    vehicle_stop_location_x = 0.0 # Vehicle x-cooridnate when it has stopped (in map frame)
    vehicle_stop_location_y = 0.0 # Vehicle y-coordinate when it has stopped (in map frame)
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_route_completed_notification):
        vehicle_stop_location_x = msg.pose.position.x
        vehicle_stop_location_y = msg.pose.position.y
        #print(str(vehicle_stop_location_x) + ", " + str(vehicle_stop_location_y))
        break

    # Obtain distance between vehicle stop location and the desired stopping location
    distance_from_desired_stopping_location = math.sqrt((vehicle_stop_location_x-map_x_stop_location)**2 + (vehicle_stop_location_y-map_y_stop_location)**2)
    distance_from_desired_stopping_location -= vehicle_length
    #print("Distance from desired stopping location: " + str(distance_from_desired_stopping_location) + " meters")

    is_successful = False
    if distance_from_desired_stopping_location > 0:
        if distance_from_desired_stopping_location < max_distance:
            print("PD-" + str(metric_number) + "-" + str(engagement_times[2]) + " Succeeded; vehicle stopped " + str(distance_from_desired_stopping_location) +
                " meters before stop point (Expected: Less than " + str(max_distance) + " meters before point)")
            is_successful = True
        else:
            print("PD-" + str(metric_number) + "-" + str(engagement_times[2]) + " Failed; vehicle stopped " + str(distance_from_desired_stopping_location) +
                " meters before stop point (Expected: Less than " + str(max_distance) + " meters before point)")
    else:
        print("PD-" + str(metric_number) + "-" + str(engagement_times[2]) + " Failed; vehicle stopped " + str(abs(distance_from_desired_stopping_location)) +
            " meters after the stop point (Expected: Less than " + str(max_distance) + " meters before point)")    
    
    return is_successful

# Main Function; run all tests from here
def main():  
    if len(sys.argv) < 2:
        print("Need 1 arguments: process_bag.py <path to source folder with .bag files> ")
        exit()
    
    source_folder = sys.argv[1]

    # Re-direct the output of print() to a specified .txt file:
    orig_stdout = sys.stdout
    current_time = datetime.datetime.now()
    text_log_filename = "Results_" + str(current_time) + ".txt"
    text_log_file_writer = open(text_log_filename, 'w')
    sys.stdout = text_log_file_writer

    # Create .csv file to make it easier to view overview of results (the .txt log file is still used for more in-depth information):
    csv_results_filename = "Results_" + str(current_time) + ".csv"
    csv_results_writer = csv.writer(open(csv_results_filename, 'w'))
    csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Test Type",
                                     "pd_1_result_0",
                                     "pd_4_result_0", "pd_4_result_3", "pd_4_result_9",
                                     "pd_19_result_1", "pd_19_result_2", "pd_19_result_4", "pd_19_result_5", "pd_19_result_6", "pd_19_result_7", "pd_19_result_8",
                                     "pd_23_result_0", "pd_23_result_1", "pd_23_result_2", "pd_23_result_3", "pd_23_result_4", "pd_23_result_5", "pd_23_result_6", "pd_23_result_7", "pd_23_result_8", "pd_23_result_9",
                                     "pd_24_result_0", "pd_24_result_1", "pd_24_result_2", "pd_24_result_3", "pd_24_result_4", "pd_24_result_5", "pd_24_result_6", "pd_24_result_7", "pd_24_result_8",
                                     "pd_25_result_0", "pd_25_result_1", "pd_25_result_2", "pd_25_result_3", "pd_25_result_4", "pd_25_result_5", "pd_25_result_6", "pd_25_result_7", "pd_25_result_8",
                                     "pd_18_result_0", "pd_18_result_1", "pd_18_result_2", "pd_18_result_3", "pd_18_result_4", "pd_18_result_5", "pd_18_result_6", "pd_18_result_7", "pd_18_result_8",
                                     "pd_9_result_0", "pd_9_result_2", "pd_9_result_3", "pd_9_result_8",
                                     "pd_20_result_0", "pd_20_result_1", "pd_20_result_2", "pd_20_result_3", "pd_20_result_4", "pd_20_result_5", "pd_20_result_6", "pd_20_result_7", "pd_20_result_8", "pd_20_result_9",
                                     "pd_10_result_0", "pd_10_result_1", "pd_10_result_2", "pd_10_result_3", "pd_10_result_4", "pd_10_result_5", "pd_10_result_6", "pd_10_result_7", "pd_10_result_8", "pd_10_result_9",
                                     "pd_21_result_1", "pd_21_result_2", "pd_21_result_3", "pd_21_result_4", "pd_21_result_5", "pd_21_result_6", "pd_21_result_7", "pd_21_result_8", "pd_21_result_9",
                                     "pd_22_result_0", "pd_22_result_1", "pd_22_result_2", "pd_22_result_3", "pd_22_result_4", "pd_22_result_5", "pd_22_result_6", "pd_22_result_7", "pd_22_result_8", "pd_22_result_9",
                                     "pd_8_result_1", "pd_8_result_4", "pd_8_result_5", "pd_8_result_6", "pd_8_result_7", "pd_8_result_8",
                                     "pd_2_result_0", "pd_2_result_3",
                                     "pd_11_result_1", "pd_11_result_4", "pd_11_result_5",
                                     "pd_5_result_6",
                                     "pd_7_result_7",
                                     "pd_3_result_3", "pd_3_result_9",
                                     "pd_6_result_6",
                                     "pd_12_result_1", "pd_12_result_5",
                                     "pd_13_result_1", "pd_13_result_5",
                                     "pd_14_result_4",
                                     "pd_15_result_1", "pd_15_result_5",
                                     "pd_16_result_4",
                                     "pd_17_result_4"])

    silver_truck_bag_files = ["1P1-R3-TS-12-9-21.bag"]
    #["1P1-R1-TS-12-9-21.bag",
    #                          "1P1-R2-TS-12-9-21.bag",
    #                          "1P1-R3-TS-12-9-21.bag"]

    # Concatenate all Port Drayage bag file lists into one list
    PD_bag_files = silver_truck_bag_files 

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in PD_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in silver_truck_bag_files:
            print("Silver Truck Port Drayage Test Case")
        else:
            print("Unknown bag file being processed.")
            
        # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
        sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(PD_bag_files.index(bag_file) + 1) + " of " + str(len(PD_bag_files)) + ")")
        sys.stdout = text_log_file_writer

        # Process bag file if it exists and can be processed, otherwise skip and proceed to next bag file
        try:
            print("Starting To Process Bag at " + str(datetime.datetime.now()))
            bag_file_path = str(source_folder) + "/" + bag_file
            bag = rosbag.Bag(bag_file_path)
            print("Finished Processing Bag at " + str(datetime.datetime.now()))
        except:
            print("Skipping " + str(bag_file) +", unable to open or process bag file.")
            continue

        # Get the rosbag start and end times associated with each "System Engaged" portion of the Use Case.
        # Note: "System Engaged" sections for a Port Drayage use case are listed below:
        #       0. Test Start Location ------> Staging Area Entrance
        #       1. Staging Area Entrance ----> Staging Area Pickup
        #       2. Staging Area Pickup ------> Staging Area Exit
        #       3. Staging Area Exit --------> Port Entrance
        #       4. Port Entrance ------------> Port Dropoff
        #       5. Port Dropoff -------------> Port Pickup
        #       6. Port Pickup --------------> Port Checkpoint
        #       7. Port Checkpoint ----------> Port Holding Area
        #       8. Port Holding Area --------> Port Exit
        #       9. Port Exit ----------------> Staging Area Entrance
        num_engagement_sections = 10 # 10 expected engagement sections for the Port Drayage use case
        start_times_list, end_times_list, found_engagement_times = get_test_case_engagement_times(bag, num_engagement_sections)
        segment_0 = [start_times_list[0], end_times_list[0], 0] # Test Start Location ------> Staging Area Entrance
        segment_1 = [start_times_list[1], end_times_list[1], 1] # Staging Area Entrance ----> Staging Area Pickup
        segment_2 = [start_times_list[2], end_times_list[2], 2] # Staging Area Pickup ------> Staging Area Exit
        segment_3 = [start_times_list[3], end_times_list[3], 3] # Staging Area Exit --------> Port Entrance
        segment_4 = [start_times_list[4], end_times_list[4], 4] # Port Entrance ------------> Port Dropoff
        segment_5 = [start_times_list[5], end_times_list[5], 5] # Port Dropoff -------------> Port Pickup
        segment_6 = [start_times_list[6], end_times_list[6], 6] # Port Pickup --------------> Port Checkpoint
        segment_7 = [start_times_list[7], end_times_list[7], 7] # Port Checkpoint ----------> Port Holding Area
        segment_8 = [start_times_list[8], end_times_list[8], 8] # Port Holding Area --------> Port Exit
        segment_9 = [start_times_list[9], end_times_list[9], 9] # Port Exit ----------------> Staging Area Entrance
        
        print_lanelet_entrance_times(bag, segment_0[0])
        # Finished: 
        # 1  ( 0,  ,  ,  ,  ,  ,  ,  ,  ,  )
        # 4  ( 0,  ,  , 3,  ,  ,  ,  ,  ,I9) 
        # 19 (    1, 2,  , 4, 5, 6, 7, 8,  ) 
        # 23 ( 0, 1, 2, 3, 4, 5, 6, 7, 8, 9)
        # 24 ( 0, 1, 2, 3, 4, 5, 6, 7, 8,  )
        # 25 ( 0, 1, 2, 3, 4, 5, 6, 7, 8,  )
        # 18 ( 0, 1, 2, 3, 4, 5, 6, 7, 8,  ) # CMV sends arrival message
        # 9  ( 0,  , 2, 3,  ,  ,  ,  , 8,  ) # V2XHub sends next destination after arrival
        # 20 (I0, 1, 2, 3, 4, 5, 6, 7, 8, 9) # CMV can receive MobilityOperation messages from V2XHub
        # 10 (I0, 1, 2, 3, 4, 5, 6, 7, 8, 9) # CMV receives the next destination from V2XHub
        # 21 (  , 1, 2, 3, 4, 5, 6, 7, 8, 9) # CMV receives next destination < 1.5 sec after sending arrival
        # 22 (I0, 1, 2, 3, 4, 5, 6, 7, 8, 9) # CMV generates route to next destination in < 3.0 sec
        # 8  (  , 1,  ,  , 4, 5, 6, 7, 8,  )
        # 2  ( 0,  ,  , 3,  ,  ,  ,  ,  ,  )
        # 11 (  , 1,  ,  , 4, 5,  ,  ,  ,  )
        # 5  (  ,  ,  ,  ,  ,  , 6,  ,  ,  )
        # 7  (  ,  ,  ,  ,  ,  ,  , 7,  ,  )
        # 3  (  ,  ,  , 3,  ,  ,  ,  ,  , 9)
        # 6  (  ,  ,  ,  ,  ,  ,V6,  ,  ,  ) # Requires V2XHub Logs (Test Operator can select either Holding Area or Port Exit)
        # 12 (  ,V1,  ,  ,  ,V5,  ,  ,  ,  ) # Requires V2XHub Logs (Test Operator Communicates to V2XHub that container is loaded)
        # 13 (  , 1,  ,  ,  , 5,  ,  ,  ,  )
        # 14 (  ,  ,  ,  , 4,  ,  ,  ,  ,  )
        # 15 (  ,V1,  ,  ,  ,V5,  ,  ,  ,  ) # Requires V2XHub Logs (V2XHub sends loading command to Test Operator)
        # 16 (  ,  ,  ,  ,V4,  ,  ,  ,  ,  ) # Requires V2XHub Logs (V2XHub sends unloading command to Test Operator)
        # 17 (  ,  ,  ,  ,V4,  ,  ,  ,  ,  ) # Requires V2XHub Logs (Test Operator Communicates to V2XHub that container is unloaded)

        # Metric FPD-1 (Vehicle reaches +/- 2 mph of speed limit)
        speed_limit_outside_port_and_staging_area_ms = 8.94 # 8.94 m/s is 20 mph
        pd_1_result_0 = check_vehicle_reaches_steady_state(bag, segment_0, speed_limit_outside_port_and_staging_area_ms)

        # Metric FPD-4 (Speed limit outside Staging Area & Port is 20-25 mph)
        min_speed_limit_FPD4 = 8.94 # 8.94 m/s is 20 mph
        max_speed_limit_FPD4 = 11.18 # 11.18 m/s is 25 mph
        segment_0_lanelet_ids = [65787, 24943, 76157] # Ordered lanelet IDs encountered in this segment that are outside of Staging Area and Port
        segment_3_lanelet_ids = [75649, 25151, 66085, 16359, 18269, 82313] # Ordered lanelet IDs encountered in this segment that are outside of Staging Area and Port
        pd_4_result_0 = check_speed_limit_outside_port_and_staging_area(bag, segment_0, segment_0_lanelet_ids, min_speed_limit_FPD4, max_speed_limit_FPD4)
        pd_4_result_3 = check_speed_limit_outside_port_and_staging_area(bag, segment_3, segment_3_lanelet_ids, min_speed_limit_FPD4, max_speed_limit_FPD4)

        # FPD-19 (Speed limit inside Staging Area or Port is 10 mph)
        speed_limit_inside_port_and_staging_area_ms = 4.47 # 4.47 m/s is 10 mph
        pd_19_result_1 = check_speed_limit_inside_port_and_staging_area(bag, segment_1, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_2 = check_speed_limit_inside_port_and_staging_area(bag, segment_2, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_4 = check_speed_limit_inside_port_and_staging_area(bag, segment_4, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_5 = check_speed_limit_inside_port_and_staging_area(bag, segment_5, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_6 = check_speed_limit_inside_port_and_staging_area(bag, segment_6, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_7 = check_speed_limit_inside_port_and_staging_area(bag, segment_7, speed_limit_inside_port_and_staging_area_ms)
        pd_19_result_8 = check_speed_limit_inside_port_and_staging_area(bag, segment_8, speed_limit_inside_port_and_staging_area_ms)

        # FPD-23 (Performance requirements for acceleration at start of route)
        pd_23_result_0 = check_acceleration_after_engagement(bag, segment_0, speed_limit_outside_port_and_staging_area_ms)
        pd_23_result_1 = check_acceleration_after_engagement(bag, segment_1, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_2 = check_acceleration_after_engagement(bag, segment_2, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_3 = check_acceleration_after_engagement(bag, segment_3, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_4 = check_acceleration_after_engagement(bag, segment_4, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_5 = check_acceleration_after_engagement(bag, segment_5, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_6 = check_acceleration_after_engagement(bag, segment_6, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_7 = check_acceleration_after_engagement(bag, segment_7, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_8 = check_acceleration_after_engagement(bag, segment_8, speed_limit_inside_port_and_staging_area_ms)
        pd_23_result_9 = check_acceleration_after_engagement(bag, segment_9, speed_limit_inside_port_and_staging_area_ms)

        # FPD-24 (Performance requirements for deceleration at end of route)
        pd_24_result_0 = check_deceleration_at_end_of_route(bag, segment_0)
        pd_24_result_1 = check_deceleration_at_end_of_route(bag, segment_1)
        pd_24_result_2 = check_deceleration_at_end_of_route(bag, segment_2)
        pd_24_result_3 = check_deceleration_at_end_of_route(bag, segment_3)
        pd_24_result_4 = check_deceleration_at_end_of_route(bag, segment_4)
        pd_24_result_5 = check_deceleration_at_end_of_route(bag, segment_5)
        pd_24_result_6 = check_deceleration_at_end_of_route(bag, segment_6)
        pd_24_result_7 = check_deceleration_at_end_of_route(bag, segment_7)
        pd_24_result_8 = check_deceleration_at_end_of_route(bag, segment_8)

        # FPD-25 (Route Completed notification occurs within 3 seconds of reaching destination)
        pd_25_result_0 = check_route_completed_notification(bag, segment_0)
        pd_25_result_1 = check_route_completed_notification(bag, segment_1)
        pd_25_result_2 = check_route_completed_notification(bag, segment_2)
        pd_25_result_3 = check_route_completed_notification(bag, segment_3)
        pd_25_result_4 = check_route_completed_notification(bag, segment_4)
        pd_25_result_5 = check_route_completed_notification(bag, segment_5)
        pd_25_result_6 = check_route_completed_notification(bag, segment_6)
        pd_25_result_7 = check_route_completed_notification(bag, segment_7)
        pd_25_result_8 = check_route_completed_notification(bag, segment_8)

        # FPD-18 (CMV broadcasts arrival message to infrastructure)
        # Do one function that takes the 'OPERATION' as a passed argument
        pd_18_result_0, pd_9_result_0, pd_20_result_1, pd_10_result_1, pd_21_result_1, pd_22_result_1 = check_cmv_arrival_message(bag, segment_0, "\"operation\": \"ENTER_STAGING_AREA\"", "\"operation\": \"PICKUP\"")
        pd_18_result_1, pd_9_result_1, pd_20_result_2, pd_10_result_2, pd_21_result_2, pd_22_result_2 = check_cmv_arrival_message(bag, segment_1, "\"operation\": \"PICKUP\"", "\"operation\": \"EXIT_STAGING_AREA\"")
        pd_18_result_2, pd_9_result_2, pd_20_result_3, pd_10_result_3, pd_21_result_3, pd_22_result_3 = check_cmv_arrival_message(bag, segment_2, "\"operation\": \"EXIT_STAGING_AREA\"", "\"operation\": \"ENTER_PORT\"")
        pd_18_result_3, pd_9_result_3, pd_20_result_4, pd_10_result_4, pd_21_result_4, pd_22_result_4 = check_cmv_arrival_message(bag, segment_3, "\"operation\": \"ENTER_PORT\"", "\"operation\": \"DROPOFF\"")
        pd_18_result_4, pd_9_result_4, pd_20_result_5, pd_10_result_5, pd_21_result_5, pd_22_result_5 = check_cmv_arrival_message(bag, segment_4, "\"operation\": \"DROPOFF\"", "\"operation\": \"PICKUP\"")
        pd_18_result_5, pd_9_result_5, pd_20_result_6, pd_10_result_6, pd_21_result_6, pd_22_result_6 = check_cmv_arrival_message(bag, segment_5, "\"operation\": \"PICKUP\"", "\"operation\": \"PORT_CHECKPOINT\"")
        pd_18_result_6, pd_9_result_6, pd_20_result_7, pd_10_result_7, pd_21_result_7, pd_22_result_7 = check_cmv_arrival_message(bag, segment_6, "\"operation\": \"PORT_CHECKPOINT\"", "\"operation\": \"HOLDING_AREA\"")
        pd_18_result_7, pd_9_result_7, pd_20_result_8, pd_10_result_8, pd_21_result_8, pd_22_result_8 = check_cmv_arrival_message(bag, segment_7, "\"operation\": \"HOLDING_AREA\"", "\"operation\": \"EXIT_PORT\"")
        pd_18_result_8, pd_9_result_8, pd_20_result_9, pd_10_result_9, pd_21_result_9, pd_22_result_9 = check_cmv_arrival_message(bag, segment_8, "\"operation\": \"EXIT_PORT\"", "\"operation\": \"ENTER_STAGING_AREA\"")

        # Check FPD-13 Segments 1 and 5 (Infrastructure communicates to CMV that container is loaded)
        if (pd_10_result_2):
            pd_13_result_1 = True
            print("PD-13-1 Succeeded; CMV was notified that container was loaded.")
        else:
            pd_13_result_1 = False
            print("PD-13-1 Failed; CMV was not notified that container was loaded.")

        if (pd_10_result_6):
            pd_13_result_5 = True
            print("PD-13-5 Succeeded; CMV was notified that container was loaded.")
        else:
            pd_13_result_5 = False
            print("PD-13-5 Failed; CMV was not notified that container was loaded.")

        # Check FPD-14 Segment 4 (Infrastructure communicates to CMV that container is unloaded)
        if (pd_10_result_5):
            pd_14_result_4 = True
            print("PD-14-4 Succeeded; CMV was notified that container was loaded.")
        else:
            pd_14_result_4 = False
            print("PD-14-4 Failed; CMV was not notified that container was loaded.")

        # PD-8 is always true (System engages on route after user chooses to engage from the Web UI)
        # PD-9 is always true (System can exit Staging Area or Port after receiving command from V2XHub)
        # Note: These metrics were evaluated in the vehicle, not by the data analysis script.
        pd_8_result_1 = True
        pd_3_result_3 = True
        pd_8_result_4 = True
        pd_8_result_5 = True
        pd_8_result_6 = True
        pd_8_result_7 = True
        pd_8_result_8 = True
        pd_3_result_9 = True

        # FPD-2, FPD-11, and FPD-5 (stopping location metrics)
        # Note: These map stopping locations were obtained by using the Lat/Long stopping locations from the
        #       use case and transforming them to the map frame via unit tests contained in the carma-platform Route package.
        stop_location_0 = [-401.128, 515.763]  # [map_x, map_y]
        stop_location_1 = [-414.198, 709.169]  # [map_x, map_y]
        stop_location_2 = [-408.123, 514.201]  # [map_x, map_y] # No metric exists to evaluate this stop
        stop_location_3 = [-75.2801, -647.096] # [map_x, map_y]
        stop_location_4 = [-52.0422, -731.793] # [map_x, map_y]
        stop_location_5 = [-28.6465, -758.279] # [map_x, map_y]
        stop_location_6 = [2.51176, -723.625]  # [map_x, map_y]
        stop_location_7 = [-3.0006, -647.181]  # [map_x, map_y]
        stop_location_8 = [-112.047, -506.881] # [map_x, map_y] # No metric exists to evaluate this stop

        freightliner_vehicle_length = 6.0
        pd_2_result_0 = check_stop_location(bag, segment_0, stop_location_0, max_distance = 3, metric_number = 2, vehicle_length = freightliner_vehicle_length) 
        pd_11_result_1 = check_stop_location(bag, segment_1, stop_location_1, max_distance = 10, metric_number = 11, vehicle_length = freightliner_vehicle_length)
        pd_2_result_3 = check_stop_location(bag, segment_3, stop_location_3, max_distance = 3, metric_number = 2, vehicle_length = freightliner_vehicle_length)
        pd_11_result_4 = check_stop_location(bag, segment_4, stop_location_4, max_distance = 10, metric_number = 11, vehicle_length = freightliner_vehicle_length)
        pd_11_result_5 = check_stop_location(bag, segment_5, stop_location_5, max_distance = 10, metric_number = 11, vehicle_length = freightliner_vehicle_length)
        pd_5_result_6 = check_stop_location(bag, segment_6, stop_location_6, max_distance = 3, metric_number = 5, vehicle_length = freightliner_vehicle_length)
        pd_7_result_7 = check_stop_location(bag, segment_7, stop_location_7, max_distance = 3, metric_number = 7, vehicle_length = freightliner_vehicle_length)

        # Invalid Metrics (Either not-applicable or the metrics are not for carma-platform):
        pd_4_result_9 = None
        pd_20_result_0 = None
        pd_10_result_0 = None
        pd_22_result_0 = None
        pd_6_result_6 = None
        pd_12_result_1 = None
        pd_12_result_5 = None
        pd_15_result_1 = None
        pd_15_result_5 = None
        pd_16_result_4 = None
        pd_17_result_4 = None

        # Get vehicle type that this bag file is from
        vehicle_name = "Unknown"
        if bag_file in silver_truck_bag_files:
            vehicle_name = "Silver Truck"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        vehicle_role = "Port Drayage"

        # Write simple pass/fail results to .csv file for appropriate row:
        csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
                                     pd_1_result_0,
                                     pd_4_result_0, pd_4_result_3, pd_4_result_9,
                                     pd_19_result_1, pd_19_result_2, pd_19_result_4, pd_19_result_5, pd_19_result_6, pd_19_result_7, pd_19_result_8,
                                     pd_23_result_0, pd_23_result_1, pd_23_result_2, pd_23_result_3, pd_23_result_4, pd_23_result_5, pd_23_result_6, pd_23_result_7, pd_23_result_8, pd_23_result_9,
                                     pd_24_result_0, pd_24_result_1, pd_24_result_2, pd_24_result_3, pd_24_result_4, pd_24_result_5, pd_24_result_6, pd_24_result_7, pd_24_result_8,
                                     pd_25_result_0, pd_25_result_1, pd_25_result_2, pd_25_result_3, pd_25_result_4, pd_25_result_5, pd_25_result_6, pd_25_result_7, pd_25_result_8,
                                     pd_18_result_0, pd_18_result_1, pd_18_result_2, pd_18_result_3, pd_18_result_4, pd_18_result_5, pd_18_result_6, pd_18_result_7, pd_18_result_8,
                                     pd_9_result_0, pd_9_result_2, pd_9_result_3, pd_9_result_8,
                                     pd_20_result_0, pd_20_result_1, pd_20_result_2, pd_20_result_3, pd_20_result_4, pd_20_result_5, pd_20_result_6, pd_20_result_7, pd_20_result_8, pd_20_result_9,
                                     pd_10_result_0, pd_10_result_1, pd_10_result_2, pd_10_result_3, pd_10_result_4, pd_10_result_5, pd_10_result_6, pd_10_result_7, pd_10_result_8, pd_10_result_9,
                                     pd_21_result_1, pd_21_result_2, pd_21_result_3, pd_21_result_4, pd_21_result_5, pd_21_result_6, pd_21_result_7, pd_21_result_8, pd_21_result_9,
                                     pd_22_result_0, pd_22_result_1, pd_22_result_2, pd_22_result_3, pd_22_result_4, pd_22_result_5, pd_22_result_6, pd_22_result_7, pd_22_result_8, pd_22_result_9,
                                     pd_8_result_1, pd_8_result_4, pd_8_result_5, pd_8_result_6, pd_8_result_7, pd_8_result_8,
                                     pd_2_result_0, pd_2_result_3,
                                     pd_11_result_1, pd_11_result_4, pd_11_result_5,
                                     pd_5_result_6,
                                     pd_7_result_7,
                                     pd_3_result_3, pd_3_result_9,
                                     pd_6_result_6,
                                     pd_12_result_1, pd_12_result_5,
                                     pd_13_result_1, pd_13_result_5,
                                     pd_14_result_4,
                                     pd_15_result_1, pd_15_result_5,
                                     pd_16_result_4,
                                     pd_17_result_4])
        
    sys.stdout = orig_stdout
    text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()
