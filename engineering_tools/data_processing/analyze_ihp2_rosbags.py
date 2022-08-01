#!/usr/bin/python3

#  Copyright (C) 2022 LEIDOS.
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
#   python3.7 analyze_ihp2_rosbags.py <path to folder containing IHP2 .bag files> 

def generate_two_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, time_starting_vehicle_start_engagement, time_starting_vehicle_end_engagement, veh_1_speed_limit_zone_change_times, test_type, test_num):
    if test_type == "SH-MID" or test_type == "SH-HIGH":
        # First vehicle to engage is vehicle 2

        # Get the true vehicle speed (m/s) and the associated time with each data point
        first = True
        vehicle_1_speed_times = []
        vehicle_1_speeds = []
        for topic, msg, t in vehicle_1_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            if first:
                time_start = t
                first = False
                continue

            vehicle_1_speed_times.append((t-time_start).to_sec())
            vehicle_1_speeds.append(msg.twist.linear.x) # Current speed in m/s

        vehicle_2_speed_times = []
        vehicle_2_speeds = []
        for topic, msg, t in vehicle_2_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            vehicle_2_speed_times.append((t-time_start).to_sec())
            vehicle_2_speeds.append(msg.twist.linear.x) # Current speed in m/s

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot vehicle 1 speed (m/s) vs. time
    ax.plot(vehicle_1_speed_times, vehicle_1_speeds, 'g:', label='Vehicle 1 Speed (m/s)')

    # Plot vehicle 2 speed (m/s) vs. time
    ax.plot(vehicle_2_speed_times, vehicle_2_speeds, 'b:', label='Vehicle 2 Speed (m/s)')

    # Optional: Plot a vertical bar at the time of each speed limit change time
    if test_type == "SH-MID":
        for i in range(0, len(veh_1_speed_limit_zone_change_times)):
            if i == 0:
                ax.axvline(x = (veh_1_speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r', label = 'Leader Speed Zone Entrance Times (In Order: 15.6, 11.3, 10.5, 9.7, 9.4, 15.6 in m/s)')
            else:
                ax.axvline(x = (veh_1_speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r')
    elif test_type == "SH-HIGH":
        for i in range(0, len(veh_1_speed_limit_zone_change_times)):
            if i == 0:
                ax.axvline(x = (veh_1_speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r', label = 'Leader Speed Zones Entrance Times (In Order: 15.6, 9.1, 8.3, 7.5, 7.2, 15.6 in m/s)')
            else:
                ax.axvline(x = (veh_1_speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(test_type) + " Test Case Run " + str(test_num) + " Vehicle Speeds") # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement for Starting Vehicle") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title

    # Option 1: Save the plot
    filename = str(test_type) + "_Run_" + str(test_num) + "_Vehicle_Speeds.png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()
    
    # Option 2: Display the plot
    #plt.show()

    return

def generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_starting_vehicle_start_engagement, time_starting_vehicle_end_engagement, test_type, test_num):
    if test_type == "SLR" or test_type == "ALR":
        # First vehicle to engage is vehicle 1

        # Get the true vehicle speed (m/s) and the associated time with each data point
        first = True
        vehicle_1_speed_times = []
        vehicle_1_speeds = []
        for topic, msg, t in vehicle_1_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            if first:
                time_start = t
                first = False
                continue

            vehicle_1_speed_times.append((t-time_start).to_sec())
            vehicle_1_speeds.append(msg.twist.linear.x) # Current speed in m/s


        vehicle_2_speed_times = []
        vehicle_2_speeds = []
        for topic, msg, t in vehicle_2_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            vehicle_2_speed_times.append((t-time_start).to_sec())
            vehicle_2_speeds.append(msg.twist.linear.x) # Current speed in m/s

        vehicle_3_speed_times = []
        vehicle_3_speeds = []
        for topic, msg, t in vehicle_3_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            vehicle_3_speed_times.append((t-time_start).to_sec())
            vehicle_3_speeds.append(msg.twist.linear.x) # Current speed in m/s
    
    elif test_type == "SLF" or test_type == "ALF":
        # First vehicle to engage is vehicle 2

        # Get the true vehicle speed (m/s) and the associated time with each data point
        first = True
        vehicle_2_speed_times = []
        vehicle_2_speeds = []
        for topic, msg, t in vehicle_2_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            if first:
                time_start = t
                first = False
                continue
           
            vehicle_2_speed_times.append((t-time_start).to_sec())
            vehicle_2_speeds.append(msg.twist.linear.x) # Current speed in m/s

        vehicle_1_speed_times = []
        vehicle_1_speeds = []
        for topic, msg, t in vehicle_1_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            vehicle_1_speed_times.append((t-time_start).to_sec())
            vehicle_1_speeds.append(msg.twist.linear.x) # Current speed in m/s

        vehicle_3_speed_times = []
        vehicle_3_speeds = []
        for topic, msg, t in vehicle_3_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = time_starting_vehicle_end_engagement): # time_start_engagement+time_duration):
            vehicle_3_speed_times.append((t-time_start).to_sec())
            vehicle_3_speeds.append(msg.twist.linear.x) # Current speed in m/s

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot vehicle 1 speed (m/s) vs. time
    ax.plot(vehicle_1_speed_times, vehicle_1_speeds, 'g:', label='Vehicle 1 Speed (m/s)')

    # Plot vehicle 2 speed (m/s) vs. time
    ax.plot(vehicle_2_speed_times, vehicle_2_speeds, 'b:', label='Vehicle 2 Speed (m/s)')

    # Plot vehicle 2 speed (m/s) vs. time
    ax.plot(vehicle_3_speed_times, vehicle_3_speeds, 'r:', label='Vehicle 3 Speed (m/s)')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(test_type) + " Test Case Run " + str(test_num) + " Vehicle Speeds") # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement for Starting Vehicle") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title

    # Option 1: Save the plot
    filename = str(test_type) + "_Run_" + str(test_num) + "_Vehicle_Speeds.png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()

    # Option 2: Display the plot
    #plt.show()

    return

def generate_speed_plot(bag, time_start_engagement, time_end_engagement, bag_file_name, speed_limit_zone_change_times, test_type, vehicle_number):
    # Speed command: /hardware_interface/arbitrated_speed_commands: msg.speed (m/s)
    # True Speed:    /hardware_interface/vehicle/twist: msg.twist.linear.x (m/s)

    # Get the true vehicle speed (m/s) and the associated time with each data point
    first = True
    true_vehicle_speed_times = []
    true_vehicle_speeds = []
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue

        true_vehicle_speed_times.append((t-time_start).to_sec())
        true_vehicle_speeds.append(msg.twist.linear.x) # Current speed in m/s

    # TODO: UPDATE THE TOPIC USED FOR THE LEXUS/PACMOD SPEED COMMANDS
    # Get the commanded vehicle speed (m/s) and the associated time with each data point
    first = True
    cmd_vehicle_speed_times = []
    cmd_vehicle_speeds = []
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/arbitrated_speed_commands'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue  
        
        cmd_vehicle_speed_times.append((t-time_start).to_sec())
        cmd_vehicle_speeds.append(msg.speed) # Commanded speed in m/s

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot commanded vehicle speed (m/s) vs. time
    ax.plot(cmd_vehicle_speed_times, cmd_vehicle_speeds, 'g:', label='Commanded Speed (m/s)')

    # Plot true vehicle speed (m/s) vs. time
    ax.plot(true_vehicle_speed_times, true_vehicle_speeds, 'b--', label='Actual Speed (m/s)')

    # Optional: Plot a vertical bar at the time of each speed limit change time
    if test_type == "SH-MID":
        for i in range(0, len(speed_limit_zone_change_times)):
            if i == 0:
                ax.axvline(x = (speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r', label = 'Speed Zone Start Times (In Order: 15.6, 11.3, 10.5, 9.7, 9.4, 15.6 in m/s)')
            else:
                ax.axvline(x = (speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r')
    elif test_type == "SH-HIGH":
        for i in range(0, len(speed_limit_zone_change_times)):
            if i == 0:
                ax.axvline(x = (speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r', label = 'Speed Zones Start Times (In Order: 15.6, 9.1, 8.3, 7.5, 7.2, 15.6 in m/s)')
            else:
                ax.axvline(x = (speed_limit_zone_change_times[i] - time_start).to_sec(), color = 'r')


    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(bag_file_name) + " Speed (Commanded and Actual) -- Vehicle " + str(vehicle_number)) # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title

    # Option 1: Save the plot
    filename = "Speed_" + bag_file_name + ".png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()

    # Option 2: Display the plot
    #plt.show() 

    return

def generate_platooning_plot(bag, time_start_platooning, bag_file_name, test_type, vehicle_number):

    actual_gaps = []
    desired_gaps = []
    times = []
    first = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_start_platooning):
        if first:
            time_start = t
            first = False
            continue 

        actual_gaps.append(msg.actual_gap)
        desired_gaps.append(msg.desired_gap)
        times.append((t-time_start).to_sec())

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot actual gaps (meters) vs. time
    ax.plot(times, actual_gaps, 'g:', label='Actual Gap (meters)')

    # Plot desired gaps (meters) vs. time
    ax.plot(times, desired_gaps, 'r--', label='Desired Gap (meters)')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(bag_file_name) + " Platooning Information -- Vehicle " + str(vehicle_number)) # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Following in Platoon") # Plot X Title
    ax.set_ylabel("Meters") # Plot Y Title

    # Option 1: Save the plot
    filename = "Platooning_Information_" + bag_file_name + ".png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()
    
    # Option 2: Display the plot
    #plt.show() 

    return

def generate_crosstrack_plot(bag, time_start_engagement, time_end_engagement, bag_file_name):
    # Crosstrack Error: /message/route_state (meters))

    # Get the true vehicle speed (m/s) and the associated time with each data point
    first = True
    crosstrack_times = []
    crosstrack_values = []
    # Note: This topic name assumes a pacmod controller is being used (freightliners or lexus)
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_end_engagement):
        if first:
            time_start = t
            first = False
            continue

        crosstrack_times.append((t-time_start).to_sec())
        crosstrack_values.append(msg.cross_track) # Crosstrack error in meters

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot crosstrack error (meters) vs. time
    ax.plot(crosstrack_times, crosstrack_values, 'g:', label='Crosstrack Error (meters)')

    # Optional: Plot a horizontal bar at a positive value for reference
    ax.axhline(y = 0.95, color = 'r', label = '+/- 0.95 Meters (For Reference)')

    # Optional: Plot a horizontal bar at a negative value for reference
    ax.axhline(y = -0.95, color = 'r')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(bag_file_name) + " Crosstrack Error") # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Plot X Title
    ax.set_ylabel("Crosstrack Error (meters)") # Plot Y Title

    # Option 1: Save the plot
    filename = "Crosstrack_" + bag_file_name + ".png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()
    
    # Option 2: Display the plot
    #plt.show() 

    return

# Helper Function: Get start and end times of the period of engagement that includes the in-geofence section
def get_test_case_engagement_times(bag):
    # Initialize system engagement start and end times
    time_start_engagement = rospy.Time()
    time_stop_engagement = rospy.Time()

    # Loop through /guidance/state messages to determine start and end times of engagement that include the in-geofence section
    is_engaged = False
    found_starting_engagement_time = False
    found_ending_engagement_time = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/state']):
        # If entering engagement, track this start time
        if (msg.state == 4 and not is_engaged):
            found_starting_engagement_time = True
            time_start_engagement = t
            is_engaged = True

        # Store the last recorded engagement timestamp in case CARMA ends engagement before a new guidance
        #       state can be published.
        if (msg.state == 4):
            time_last_engaged = t
        
        # Log time that engagements ends
        elif (msg.state != 4 and is_engaged):
            found_ending_engagement_time = True
            is_engaged = False
            time_stop_engagement = t
    
    # If CARMA ended engagement before guidance state could be updated, update time_stop_engagement
    if found_starting_engagement_time and not found_ending_engagement_time:
        time_stop_engagement = time_last_engaged
    
    found_engagement_times = False
    if found_starting_engagement_time:
        found_engagement_times = True

    return time_start_engagement, time_stop_engagement, found_engagement_times

def analyze_route_speed_limits(bag):
    print("New route analyzed: ")
    current_lanelet = None
    current_speed_limit = None
    lanelet_speed_limits = []
    speed_limit_zone_change_times = []
    speed_limit_zone_values = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state']):
        if msg.lanelet_id != current_lanelet:
            lanelet_speed_limits.append(msg.speed_limit)
            current_lanelet = msg.lanelet_id

        if current_speed_limit is None:
            print("New speed limit " + str(msg.speed_limit) + " at time " + str(t.to_sec()))
            speed_limit_zone_values.append(msg.speed_limit)
            speed_limit_zone_change_times.append(t)
            current_speed_limit = msg.speed_limit
        else:
            if abs(current_speed_limit - msg.speed_limit) > 0.1:
                print("New speed limit " + str(msg.speed_limit) + " at time " + str(t.to_sec()))
                speed_limit_zone_values.append(msg.speed_limit)
                speed_limit_zone_change_times.append(t)
                current_speed_limit = msg.speed_limit

    # Remove first element for zero speed limit
    lanelet_speed_limits = lanelet_speed_limits[1:]
    speed_limit_zone_change_times = speed_limit_zone_change_times[1:]
    speed_limit_zone_values = speed_limit_zone_values[1:]

    print("Route speed limits in m/s (1 entry for each shortest-path lanelet in order): " + str(lanelet_speed_limits[1:]))

    return speed_limit_zone_values,speed_limit_zone_change_times


def leader_platoon_formation_analysis(bag, run_type, leader_id, follower_id):

    if run_type == "SLR":
        has_received_initial_request = False
        time_received_initial_request = rospy.Time()
        for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_request']):
            if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
                if msg.plan_type.type == 3:
                    has_received_initial_request = True
                    time_received_initial_request = t
                    print("Received initial request at " + str(t.to_sec()))

        has_sent_initial_ack = False
        time_sent_initial_ack = rospy.Time()
        for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_response']):
            if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
                if msg.plan_type.type == 3:
                    has_sent_initial_ack = True
                    time_sent_initial_ack = t
                    print("Sent initial ACK at " + str(t.to_sec()))


        has_received_second_request = False
        time_received_second_request = rospy.Time()
        for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_request']):
            if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
                if msg.plan_type.type == 4:
                    has_received_second_request = True
                    time_received_second_request = t
                    print("Received second request at " + str(t.to_sec()))


        has_sent_second_ack = False
        time_sent_second_ack = rospy.Time()
        for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_response']):
            if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
                if msg.plan_type.type == 4:
                    has_sent_second_ack = True
                    time_sent_second_ack = t
                    print("Sent second ACK at " + str(t.to_sec()))


        print("DEBUG: Time between initial request and ack: " + str((time_sent_initial_ack - time_received_initial_request).to_sec()) + " sec")
        print("DEBUG: Time between initial request and ack: " + str((time_sent_second_ack - time_received_second_request).to_sec()) + " sec")
        print("DEBUG: Time between initial request and second ack: " + str((time_sent_second_ack - time_received_initial_request).to_sec()) + " sec")
        return

def joiner_platoon_formation_analysis(bag, run_type, leader_id, follower_id, joiner_vehicle_number):

    has_sent_initial_request = False
    time_sent_initial_request = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request']):
        if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
            if run_type == "SLR" or run_type == "SH-MID" or run_type == "SH-HIGH" or (run_type == "SLF" and joiner_vehicle_number == 3) \
            or (run_type == "ALR" and joiner_vehicle_number == 2) or (run_type == "ALF" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 3:
                    has_sent_initial_request = True
                    time_sent_initial_request = t
                    #print("DEBUG: Sent initial request at " + str(t.to_sec()))

            elif (run_type == "SLF" and joiner_vehicle_number == 1):
                print("DEBUG: " + str(msg.m_header.sender_id) + " sending " + str(msg.m_header.recipient_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()))
                if msg.plan_type.type == 5:
                    has_sent_initial_request = True
                    time_sent_initial_request = t
                    #print("DEBUG: Sent initial request at " + str(t.to_sec()))

            elif (run_type == "ALR" and joiner_vehicle_number == 3) or (run_type == "ALF" and joiner_vehicle_number == 1):
                print("DEBUG: " + str(msg.m_header.sender_id) + " sending " + str(msg.m_header.recipient_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()) + " with params " + str(msg.strategy_params))
                if msg.plan_type.type == 8:
                    has_sent_initial_request = True
                    time_sent_initial_request = t

    has_received_initial_ack = False
    time_received_initial_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response']):
        if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
            if run_type == "SLR" or run_type == "SH-MID" or run_type == "SH-HIGH" or (run_type == "SLF" and joiner_vehicle_number == 3) \
            or (run_type == "ALR" and joiner_vehicle_number == 2) or (run_type == "ALF" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 3 and msg.is_accepted:
                    has_received_initial_ack = True
                    time_received_initial_ack = t
                    #print("DEBUG: Received initial ACK at " + str(t.to_sec()))

            elif (run_type == "SLF" and joiner_vehicle_number == 1):
                print("DEBUG: " + str(msg.m_header.recipient_id) + " receiving from " + str(msg.m_header.sender_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()) + " with " + str(msg.is_accepted))
                if msg.plan_type.type == 5 and msg.is_accepted:
                    has_received_initial_ack = True
                    time_received_initial_ack = t
                    #print("DEBUG: Received initial ACK at " + str(t.to_sec()))

            elif (run_type == "ALR" and joiner_vehicle_number == 3) or (run_type == "ALF" and joiner_vehicle_number == 1):
                print("DEBUG: " + str(msg.m_header.recipient_id) + " receiving from " + str(msg.m_header.sender_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()) + " with " + str(msg.is_accepted))
                if msg.plan_type.type == 8 and msg.is_accepted:
                    has_received_initial_ack = True
                    time_received_initial_ack = t

    has_sent_second_request = False
    time_sent_second_request = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request']):
        if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
            if run_type == "SLR" or run_type == "SH-MID" or run_type == "SH-HIGH" or (run_type == "SLF" and joiner_vehicle_number == 3) \
            or (run_type == "ALR" and joiner_vehicle_number == 2) or (run_type == "ALF" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 4:
                    has_sent_second_request = True
                    time_sent_second_request = t
                    #print("DEBUG: Sent second request at " + str(t.to_sec()))

            elif (run_type == "ALR" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 11:
                    has_sent_second_request = True
                    time_sent_second_request = t

            elif (run_type == "ALF" and joiner_vehicle_number == 1):
                if msg.plan_type.type == 10:
                    has_sent_second_request = True
                    time_sent_second_request = t

    has_received_second_ack = False
    time_received_second_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response']):
        if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
            if run_type == "SLR" or run_type == "SH-MID" or run_type == "SH-HIGH" or (run_type == "SLF" and joiner_vehicle_number == 3) \
            or (run_type == "ALR" and joiner_vehicle_number == 2) or (run_type == "ALF" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 4 and msg.is_accepted:
                    has_received_second_ack = True
                    time_received_second_ack = t
                    #print("DEBUG: Received second ACK at " + str(t.to_sec()))

            elif (run_type == "ALR" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 11 and msg.is_accepted:
                    has_received_second_ack = True
                    time_received_second_ack = t

            elif (run_type == "ALF" and joiner_vehicle_number == 1):
                if msg.plan_type.type == 10 and msg.is_accepted:
                    has_received_second_ack = True
                    time_received_second_ack = t

    has_sent_third_request = False
    time_sent_third_request = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request']):
        if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
            if (run_type == "ALR" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 4:
                    has_sent_third_request = True
                    time_sent_third_request = t

    has_received_third_ack = False
    time_received_third_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response']):
        if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
            if (run_type == "ALR" and joiner_vehicle_number == 3):
                if msg.plan_type.type == 4 and msg.is_accepted:
                    has_received_third_ack = True
                    time_received_third_ack = t

    has_received_second_request = False
    time_received_second_request = rospy.Time()
    has_received_third_request = False
    time_received_third_request = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_request']):
        print("DEBUG: " + str(msg.m_header.recipient_id) + " receiving from " + str(msg.m_header.sender_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()))
        if msg.m_header.recipient_id == follower_id and msg.m_header.sender_id == leader_id:
            if (run_type == "SLF" and joiner_vehicle_number == 1):
                if msg.plan_type.type == 6:
                    has_received_second_request = True
                    time_received_second_request = t

            elif (run_type == "ALF" and joiner_vehicle_number == 1):
                print("DEBUG: " + str(msg.m_header.sender_id) + " sending " + str(msg.m_header.recipient_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()))
                if msg.plan_type.type == 6:
                    has_received_third_request = True
                    time_received_third_request = t

    has_sent_second_ack = False
    time_sent_second_ack = rospy.Time()
    has_sent_third_ack = False
    time_has_sent_third_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_response']):
        if (run_type == "SLF" and joiner_vehicle_number == 1):
            if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
                print("DEBUG: " + str(msg.m_header.sender_id) + " sending " + str(msg.m_header.recipient_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()) + " with " + str(msg.is_accepted))
                if msg.plan_type.type == 6 and msg.is_accepted:
                    has_sent_second_ack = True
                    time_sent_second_ack = t

        elif (run_type == "ALF" and joiner_vehicle_number == 1):
            if msg.m_header.recipient_id == leader_id and msg.m_header.sender_id == follower_id:
                print("DEBUG: " + str(msg.m_header.sender_id) + " sending " + str(msg.m_header.recipient_id) + " plan_type " + str(msg.plan_type.type) + " at " + str(t.to_sec()) + " with " + str(msg.is_accepted))
                if msg.plan_type.type == 6 and msg.is_accepted:
                    has_sent_third_ack = True
                    time_sent_third_ack = t

    total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
    #print("DEBUG: Time between sending initial request and receiving ack: " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
    #print("DEBUG: Time between sending second request and receiving ack: " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec")
    #print("DEBUG: Time between sending initial request and receiving second ack: " + str(total_formation_duration) + " sec")
        
    if run_type == "SLR" or run_type == "SH-MID" or run_type == "SH-HIGH":
        total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
        if joiner_vehicle_number == 2:
            if has_received_initial_ack:
                print("IHP2-1 Succeeded - Vehicle 2 received initial ACK from leader for JOIN_PLATOON_AT_REAR at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-1 Failed - Vehicle 2 did not receive initial ACK from leader for JOIN_PLATOON_AT_REAR")
                
            if has_received_second_ack:
                print("IHP2-3 Succeeded - Vehicle 2 received second ACK from leader for PLATOON_FOLLOWER_JOIN and joins platoon at " + str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-3 Failed - Vehicle 2 did not receive second ACK from leader for PLATOON_FOLLOWER_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-2 Succeeded: Time between Vehicle 2 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-2 Succeeded: Time between Vehicle 2 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack]

        elif joiner_vehicle_number == 3:
            if has_received_initial_ack:
                print("IHP2-7 Succeeded - Vehicle 3 received initial ACK from leader for JOIN_PLATOON_AT_REAR at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-7 Failed - Vehicle 3 did not receive initial ACK from leader for JOIN_PLATOON_AT_REAR")
             
            if has_received_second_ack:
                print("IHP2-9 Succeeded - Vehicle 3 received second ACK from leader for PLATOON_FOLLOWER_JOIN and joins platoon at " + str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9 Failed - Vehicle 3 did not receive second ACK from leader for PLATOON_FOLLOWER_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-8 Succeeded: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-8 Failed: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack]
        
    elif run_type == "SLF":
        if joiner_vehicle_number == 3:
            total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-1 Succeeded - Vehicle 3 received initial ACK from leader for JOIN_PLATOON_AT_REAR at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-1 Failed - Vehicle 3 did not receive initial ACK from leader for JOIN_PLATOON_AT_REAR")
                
            if has_received_second_ack:
                print("IHP2-3 Succeeded - Vehicle 3 received second ACK from leader for PLATOON_FOLLOWER_JOIN and joins platoon at " + str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-3 Failed - Vehicle 3 did not receive second ACK from leader for PLATOON_FOLLOWER_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-2 Succeeded: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-2 Succeeded: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack]

        elif joiner_vehicle_number == 1:
            total_formation_duration = (time_sent_second_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-7 Succeeded - Vehicle 1 received initial ACK from leader for JOIN_PLATOON_FROM_FRONT at " + str(time_received_initial_ack.to_sec())+ " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-7 Failed - Vehicle 1 did not receive initial ACK from leader for JOIN_PLATOON_FROM_FRONT")
             
            if has_sent_second_ack:
                print("IHP2-9 Succeeded - Vehicle 1 sent second ACK to leader for PLATOON_FRONT_JOIN and joins platoon at " +str(time_sent_second_ack.to_sec()) + " in " + str((time_sent_second_ack - time_received_second_request).to_sec()) + " sec; " + str((time_sent_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9 Failed - Vehicle 1 did not send second ACK to leader for PLATOON_FRONT_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-8 Succeeded: Time between Vehicle 1 sending initial request and sending second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-8 Failed: Time between Vehicle 1 sending initial request and sending second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_sent_second_ack]

    elif run_type == "ALR":
        if joiner_vehicle_number == 2:
            total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-1 Succeeded - Vehicle 2 received initial ACK from leader for JOIN_PLATOON_AT_REAR at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-1 Failed - Vehicle 2 did not receive initial ACK from leader for JOIN_PLATOON_AT_REAR")
                
            if has_received_second_ack:
                print("IHP2-3 Succeeded - Vehicle 2 received second ACK from leader for PLATOON_FOLLOWER_JOIN and joins platoon at " +str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-3 Failed - Vehicle 2 did not receive second ACK from leader for PLATOON_FOLLOWER_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-2 Succeeded: Time between Vehicle 2 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-2 Succeeded: Time between Vehicle 2 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack]
        elif joiner_vehicle_number == 3:
            total_formation_duration = (time_received_third_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-7 Succeeded (ALR): Vehicle 3 received initial ACK from leader for PLATOON_CUT_IN_JOIN at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-7 Failed (ALR): Vehicle 3 did not receive initial ACK from leader for PLATOON_CUT_IN_JOIN ")

            if has_received_second_ack:
                print("IHP2-9 Succeeded (ALR): Vehicle 3 received second ACK from leader for CUT_IN_MID_OR_REAR_DONE at " +str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9 Failed (ALR): Vehicle 3 did not receive second ACK from leader for CUT_IN_MID_OR_REAR_DONE")
            
            if has_received_third_ack:
                print("IHP2-9.5 Succeeded (ALR): Vehicle 3 received third ACK from leader for PLATOON_FOLLOWER_JOIN at " + str(time_received_third_ack.to_sec()) + " in " + str((time_received_third_ack - time_sent_third_request).to_sec()) + " sec; " + str((time_received_third_ack-time_received_second_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9.5 Failed (ALR): Vehicle 3 did not receive third ACK from leader for PLATOON_FOLLOWER_JOIN")

            if total_formation_duration <= 2.0:
                print("IHP2-8 Succeeded (ALR): Time between Vehicle 3 sending initial request and receiving third ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-8 Succeeded (ALR): Time between Vehicle 3 sending initial request and receiving third ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack, time_received_third_ack]

    elif run_type == "ALF":
        if joiner_vehicle_number == 3:
            total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-1 Succeeded - Vehicle 3 received initial ACK from leader for JOIN_PLATOON_AT_REAR at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-1 Failed - Vehicle 3 did not receive initial ACK from leader for JOIN_PLATOON_AT_REAR")
                
            if has_received_second_ack:
                print("IHP2-3 Succeeded - Vehicle 3 received second ACK from leader for PLATOON_FOLLOWER_JOIN and joins platoon at " + str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-3 Failed - Vehicle 3 did not receive second ACK from leader for PLATOON_FOLLOWER_JOIN and did not join the platoon")
                
            if total_formation_duration <= 2.0:
                print("IHP2-2 Succeeded: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-2 Succeeded: Time between Vehicle 3 sending initial request and receiving second ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack]
        elif joiner_vehicle_number == 1:
            total_formation_duration = (time_sent_third_ack - time_sent_initial_request).to_sec()
            if has_received_initial_ack:
                print("IHP2-7 Succeeded (ALF): Vehicle 1 received initial ACK from leader for PLATOON_CUT_IN_JOIN at " + str(time_received_initial_ack.to_sec()) + " in " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
            else:
                print("IHP2-7 Failed (ALF): Vehicle 1 did not receive initial ACK from leader for PLATOON_CUT_IN_JOIN ")

            if has_received_second_ack:
                print("IHP2-9 Succeeded (ALF): Vehicle 1 received second ACK from leader for CUT_IN_FRONT_DONE  at " +str(time_received_second_ack.to_sec()) + " in " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec; " + str((time_received_second_ack-time_received_initial_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9 Failed (ALF): Vehicle 1 did not receive second ACK from leader for CUT_IN_FRONT_DONE ")
            
            if has_sent_third_ack:
                print("IHP2-9.5 Succeeded (ALF): Vehicle 1 sent third ACK to leader for PLATOON_FRONT_JOIN at " + str(time_sent_third_ack.to_sec()) + " in " + str((time_sent_third_ack - time_received_third_request).to_sec()) + " sec; " + str((time_sent_third_ack-time_received_second_ack).to_sec()) + " sec after prev ack")
            else:
                print("IHP2-9.5 Failed (ALF): Vehicle 1 did not send third ACK to leader for PLATOON_FRONT_JOIN ")

            if total_formation_duration <= 2.0:
                print("IHP2-8 Succeeded (ALF): Time between Vehicle 1 sending initial request and sending third ACK was " + str(total_formation_duration) + " sec")
            else:
                print("IHP2-8 Succeeded (ALF): Time between Vehicle 1 sending initial request and sending third ACK was " + str(total_formation_duration) + " sec")

            return [time_received_initial_ack, time_received_second_ack, time_sent_third_ack]

def received_tcm_analysis(bag, time_start_platooning, test_type, vehicle_number, expected_tcm_count):

    # List to store the receive times of each TCM msgnum [time_receive_msgnum1, time_receive_msgnum2, etc.]
    tcm_receive_times = [None for i in range(0, expected_tcm_count)]

    # Store all transmitted TCR reqids so that only corresponding received TCMs are analyzed
    tcr_reqids = []
    tcr_reqids_times = []
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_geofence_request']): 
        tcr_reqids.append(msg.tcr_v01.reqid.id)
        tcr_reqids_times.append(t)

    # Only analyze TCMs with reqids matching a transmitted TCR reqid. Populate the tcm_receive_times list
    has_received_any_tcms = False
    has_received_all_tcms = False
    has_received_correct_tcms = True
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']): 
        reqid = msg.tcm_v01.reqid.id
        if reqid in tcr_reqids:
            has_received_any_tcms = True
            msgnum = int(msg.tcm_v01.msgnum) - 1
            if tcm_receive_times[msgnum] is None:
                time_received_after_platoon_start = (t-time_start_platooning).to_sec()
                tcm_receive_times[msgnum] = time_received_after_platoon_start

                tcm_type = msg.tcm_v01.params.detail.choice

                if tcm_type == 12:
                    max_speed = msg.tcm_v01.params.detail.maxspeed
                    print("DEBUG: Received TCM msgnum " + str(msgnum) + " of " + str(msg.tcm_v01.msgtot) + " with type " + str(tcm_type) + " " + str(time_received_after_platoon_start) + " sec after platooning -- max speed " + str(max_speed))
                else:
                    print("Received non-maxspeed TCM")

        if None not in tcm_receive_times:
            has_received_all_tcms = True
            break

    if not has_received_any_tcms:
        print("IHP2-4 Failed for vehicle " + str(vehicle_number) + ": No TCMs were received matching its TCR reqids")
        print("IHP2-5 Failed for vehicle " + str(vehicle_number) + ": No TCMs were received matching its TCR reqids")
        return
    
    #if has_received_all_tcms:
    #    time_received_first_tcm = min(tcm_receive_times)
    #    print("DEBUG: Vehicle " + str(vehicle_number) + " received first applicable TCM " + str(time_received_first_tcm) + " sec after beginning platooning")
    
    if min(tcm_receive_times) > 0:
        print("IHP2-4 Succeeded for vehicle " + str(vehicle_number) + ": all TCMs were received after the vehicle began platooning")
    else:
        print("IHP2-4 Failed for vehicle " + str(vehicle_number) + ": at least one TCM was received before vehicle began platooning")

    if has_received_all_tcms and has_received_correct_tcms:
        print("IHP2-5 Succeeded: CMV received all correct TCMs for test type " + str(test_type))
    elif not has_received_correct_tcms and not has_received_correct_tcms:
        print("IHP2-5 Failed: CMV didn't receive all TCMs and didn't receive correct TCMs for test type " + str(test_type))
    elif not has_received_correct_tcms:
        print("IHP2-5 Failed: CMV did not receive all correct TCMs for test type " + str(test_type))
    elif not has_received_all_tcms:
        print("IHP2-5 Failed: CMV did not receive all TCM msgnums for test type " + str(test_type))


def crosstrack_distance_analysis(join_vehicle_rosbag, other_vehicle_rosbag, run_type, join_vehicle_number, other_vehicle_number, time):
    # Get other vehicle cross track at provided time (negative is to the left; positive is to the right)
    for topic, msg, t in other_vehicle_rosbag.read_messages(topics=['/guidance/route_state'], start_time = time):
        other_vehicle_cross_track = msg.cross_track
        break

    # Get joining vehicle pose at provided time (negative is to the left; positive is to the right)
    for topic, msg, t in join_vehicle_rosbag.read_messages(topics=['/guidance/route_state'], start_time = time):
        join_vehicle_cross_track = msg.cross_track
        break

    total_cross_track = abs(other_vehicle_cross_track - join_vehicle_cross_track)
    print("DEBUG: Join vehicle " + str(join_vehicle_number) + " crosstrack " + str(join_vehicle_cross_track) + ", veh " + str(other_vehicle_number) + " crosstrack " + str(other_vehicle_cross_track) + ", total " + str(total_cross_track) + " meters at time " + str(time.to_sec()))

    if run_type == "ALR":
        if total_cross_track < 1.5:
            print("IHP2-20 Succeeded: crosstrack between joining vehicle " + str(join_vehicle_number) + " and other vehicle " + str(other_vehicle_number) + " was " + str(total_cross_track) + " meters (< 1.5 desired) at PLATOON_FOLLOWER_JOIN")
        else:
            print("IHP2-20 Failed: crosstrack between joining vehicle " + str(join_vehicle_number) + " and other vehicle " + str(other_vehicle_number) + " was " + str(total_cross_track) + " meters (< 1.5 desired) at PLATOON_FOLLOWER_JOIN")
    elif run_type == "ALF":
        if total_cross_track < 1.5:
            print("IHP2-22 Succeeded: crosstrack between joining vehicle " + str(join_vehicle_number) + " and other vehicle " + str(other_vehicle_number) + " was " + str(total_cross_track) + " meters (< 1.5 desired) at PLATOON_FRONT_JOIN")
        else:
            print("IHP2-22 Failed: crosstrack between joining vehicle " + str(join_vehicle_number) + " and other vehicle " + str(other_vehicle_number) + " was " + str(total_cross_track) + " meters (< 1.5 desired) at PLATOON_FRONT_JOIN")

    return total_cross_track

def downtrack_distance_analysis(join_vehicle_rosbag, other_vehicle_rosbag, run_type, join_vehicle_number, other_vehicle_number, time):
    # Get other vehicle pose at provided time
    for topic, msg, t in other_vehicle_rosbag.read_messages(topics=['/localization/current_pose'], start_time = time):
        other_vehicle_pose = [msg.pose.position.x, msg.pose.position.y]
        break

    # Get joining vehicle pose at provided time
    for topic, msg, t in join_vehicle_rosbag.read_messages(topics=['/localization/current_pose'], start_time = time):
        join_vehicle_pose = [msg.pose.position.x, msg.pose.position.y]
        break
    
    # Get joining vehicle speed
    for topic, msg, t in join_vehicle_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time):
        join_vehicle_speed = msg.twist.linear.x
        break

    distance_between_vehicles = ((other_vehicle_pose[0] - join_vehicle_pose[0])**2 + (other_vehicle_pose[1] - join_vehicle_pose[1])**2) ** 0.5
    time_gap_between_vehicles = distance_between_vehicles / join_vehicle_speed

    print("DEBUG: Joining vehicle " + str(join_vehicle_number) + " speed is " + str(join_vehicle_speed) + " m/s")
    print("DEBUG: Distance between joining vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " at time " + str(time.to_sec()) + " is " + str(distance_between_vehicles) + " meters")
    print("DEBUG: Time Gap between joining vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " at time " + str(time.to_sec()) + " is " + str(time_gap_between_vehicles) + " seconds")

    if run_type == "SLR":
        # Values are the same for same-lane front join and same-lane rear join
        if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
            print("IHP2-17 (SLR) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
        else:
            print("IHP2-17 (SLR) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
    
    elif run_type == "SH-MID" or run_type == "SH-HIGH":
        # This test is for a same-lane rear join
        if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
            print("IHP2-17 (SH-MID or SH-HIGH) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
        else:
            print("IHP2-17 (SH-MID or SH-HIGH) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")

    elif run_type == "SLF":
        # Values are the same for same-lane front join and same-lane rear join
        if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
            print("IHP2-18 (SLF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_FRONT")
        else:
            print("IHP2-18 (SLF) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_FRONT")
    
    elif run_type == "ALR":
        if join_vehicle_number == 3:
            # No time gap thresholds for adjacent lane rear join vehicle
            if distance_between_vehicles <= 100.0:
                print("IHP2-19 (SLF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and platoon rear vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=100 desired) at PLATOON_CUT_IN_JOIN")
            else:
                print("IHP2-19 (SLF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and platoon rear vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=100 desired) at PLATOON_CUT_IN_JOIN")
        elif join_vehicle_number == 2:
            if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
                print("IHP2-17/18 (SLR in ALR) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
            else:
                print("IHP2-17/18 (SLR in ALR) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
    
    elif run_type == "ALF":
        if join_vehicle_number == 1:
            # No time gap thresholds for adjacent lane front join vehicle
            if distance_between_vehicles <= 150.0:
                print("IHP2-21 (ALF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and platoon lead vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=150 desired) at PLATOON_CUT_IN_JOIN")
            else:
                print("IHP2-21 (ALF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and platoon lead vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=150 desired) at PLATOON_CUT_IN_JOIN")
        elif join_vehicle_number == 3:
            if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
                print("IHP2-17/18 (SLR in ALF) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
            else:
                print("IHP2-17/18 (SLR in ALF) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
    
    return distance_between_vehicles

# Main Function; run all tests from here
def main():  
    if len(sys.argv) < 2:
        print("Need 1 arguments: process_bag.py <path to source folder with .bag files> ")
        exit()
    
    source_folder = sys.argv[1]

    # # Re-direct the output of print() to a specified .txt file:
    # orig_stdout = sys.stdout
    # current_time = datetime.datetime.now()
    # text_log_filename = "Results_" + str(current_time) + ".txt"
    # text_log_file_writer = open(text_log_filename, 'w')
    # sys.stdout = text_log_file_writer

    # # Create .csv file to make it easier to view overview of results (the .txt log file is still used for more in-depth information):
    # csv_results_filename = "Results_" + str(current_time) + ".csv"
    # csv_results_writer = csv.writer(open(csv_results_filename, 'w'))
    # csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Test Type",
    #                                  "IHP2-1", "IHP2-2", "IHP2-3", "IHP2-4", "IHP2-5",
    #                                  "IHP2-6", "IHP2-7", "IHP2-8", "IHP2-9", "IHP2-10",
    #                                  "IHP2-11", "IHP2-12", "IHP2-13", "IHP2-14", "IHP2-15",
    #                                  "IHP2-16", "IHP2-17", "IHP2-18", "IHP2-19"])

    # Constants
    BLUE_LEXUS = "DOT-45244"
    BLACK_PACIFICA = "DOT-45245"
    WHITE_PACIFICA = "DOT-45246"

    # SLR Test 5 removed (BP didn't engage), Test 6 removed (incorrect BL rosbag)
    slr_bag_files = [[["SLR_BL_1_2022-07-13-19-43-10.bag",
                           "SLR_WP_1_2022-07-13-19-43-51.bag",
                           "SLR_BP_01.bag"], 
                           [BLUE_LEXUS, WHITE_PACIFICA, BLACK_PACIFICA]],
                     [["SLR_BL_2_2022-07-13-19-57-25.bag",
                           "SLR_WP_2_2022-07-13-19-57-54.bag",
                           "SLR_BP_02.bag"], 
                           [BLUE_LEXUS, WHITE_PACIFICA, BLACK_PACIFICA]],
                     [["SLR_BP_3_2022-07-13-20-24-52.bag",
                           "SLR_WP_3_2022-07-13-20-24-27.bag",
                           "SLR_BL_3_2022-07-13-20-24-30.bag"], 
                           [BLACK_PACIFICA, WHITE_PACIFICA, BLUE_LEXUS]],
                     [["SLR_BP_4_2022-07-13-20-32-58.bag",
                           "SLR_WP_4_2022-07-13-20-34-47.bag",
                           "SLR_BL_4_2022-07-13-20-34-41.bag"], 
                           [BLACK_PACIFICA, WHITE_PACIFICA, BLUE_LEXUS]]]

    slf_bag_files = [[["SLF_BL_1_2022-07-14-13-23-46.bag",
                           "SLF_BP_01_2022-07-14-13-27-17.bag",
                           "SLF_WP_1_2022-07-14-13-26-23.bag"], 
                           [BLUE_LEXUS, BLACK_PACIFICA, WHITE_PACIFICA]],
                     [["SLF_BL_2_2022-07-14-13-32-51.bag",
                           "SLF_BP_2_2022-07-14-13-32-23.bag",
                           "SLF_WP_2_2022-07-14-13-32-08.bag"], 
                           [BLUE_LEXUS, BLACK_PACIFICA, WHITE_PACIFICA]],
                     [["SLF_BL_3_2022-07-14-13-37-47.bag",
                           "SLF_WP_3_2022-07-14-13-37-49.bag",
                           "SLF_BP_3_2022-07-14-13-37-53.bag"], 
                           [BLUE_LEXUS, WHITE_PACIFICA, BLACK_PACIFICA]],
                     [["SLF_BP_4_2022-07-14-13-46-28.bag",
                           "SLF_WP_4_2022-07-14-13-46-17.bag",
                           "SLF_BL_4_2022-07-14-13-46-30.bag"], 
                           [BLACK_PACIFICA, WHITE_PACIFICA, BLUE_LEXUS]],
                     [["SLF_WP_5_2022-07-14-13-53-29.bag",
                           "SLF_BP_5_2022-07-14-13-53-37.bag",
                           "SLF_BL_5_2022-07-14-13-53-08.bag"], 
                           [WHITE_PACIFICA, BLACK_PACIFICA, BLUE_LEXUS]],
                     [["SLF_WP_6_2022-07-14-13-59-05.bag",
                           "SLF_BP_6_2022-07-14-13-58-35.bag",
                           "SLF_BL_6_2022-07-14-13-57-48.bag"], 
                           [WHITE_PACIFICA, BLACK_PACIFICA, BLUE_LEXUS]]]

    alr_bag_files = [[["ALR2_WP_1_2022-07-14-18-21-10.bag",
                           "ALR2_BL_1_2022-07-14-18-25-00.bag",
                           "ALR2_BP_1_2022-07-14-18-19-30.bag"],
                           [WHITE_PACIFICA, BLUE_LEXUS, BLACK_PACIFICA]],
                      [["ALR2_WP_2_2022-07-14-18-31-06.bag",
                           "ALR2_BL_2_2022-07-14-18-30-53.bag",
                           "ALR2_BP_2_2022-07-14-18-31-31.bag"],
                           [WHITE_PACIFICA, BLUE_LEXUS, BLACK_PACIFICA]],
                      [["ALR2_WP_3_2022-07-14-18-36-16.bag",
                           "ALR2_BL_3_2022-07-14-18-36-43.bag",
                           "ALR2_BP_3_2022-07-14-18-36-40.bag"],
                           [WHITE_PACIFICA, BLUE_LEXUS, BLACK_PACIFICA]],   
                      [["ALR2_BP_4_2022-07-14-18-45-55.bag",
                           "ALR2_WP_4_2022-07-14-18-45-03.bag",
                           "ALR2_BL_4_2022-07-14-18-46-06.bag"],
                           [BLACK_PACIFICA, WHITE_PACIFICA, BLUE_LEXUS]],  
                      [["ALR2_BP_5_2022-07-14-18-54-27.bag",
                           "ALR2_WP_5_2022-07-14-18-53-32.bag",
                           "ALR2_BL_5_2022-07-14-18-54-23.bag"],
                           [BLACK_PACIFICA, WHITE_PACIFICA, BLUE_LEXUS]]]

    # Note: Test 3 data was removes since it could not be processed
    alf_bag_files = [[["ALF_BP_1_2022-07-14-19-04-30.bag",
                           "ALF_BL_1_2022-07-14-19-07-10.bag",
                           "ALF_WP_1_2022-07-14-19-07-16.bag"],
                           [BLACK_PACIFICA, BLUE_LEXUS, WHITE_PACIFICA]],
                     [["ALF_BP_2_2022-07-14-19-13-35.bag",
                           "ALF_BL_2_2022-07-14-19-13-14.bag",
                           "ALF_WP_2_2022-07-14-19-13-17.bag"],
                           [BLACK_PACIFICA, BLUE_LEXUS, WHITE_PACIFICA]],        
                     [["ALF_BL_4_2022-07-14-19-35-27.bag",
                           "ALF_WP_4_2022-07-14-19-35-12.bag",
                           "ALF_BP_4_2022-07-14-19-35-26.bag"],
                           [BLUE_LEXUS, WHITE_PACIFICA, BLACK_PACIFICA]], 
                     [["ALF_BL_5_2022-07-14-19-42-11.bag",
                           "ALF_BP_5_2022-07-14-19-41-13.bag",
                           "ALF_WP_5_2022-07-14-19-41-35.bag"],
                           [BLUE_LEXUS, BLACK_PACIFICA, WHITE_PACIFICA]]]
    
    sh_mid_bag_files = [[["SH_BP_01.bag",
                           "SH_WP_1_2022-07-13.bag"],
                           [BLACK_PACIFICA, WHITE_PACIFICA]]]
    
    # Test 2 failed; Not Black Pacifica bag for Test 7
    sh_high_bag_files = [[["SH_BP_03.bag",
                           "SH_WP_3_2022-07-13-16-06-49.bag"],
                           [BLACK_PACIFICA, WHITE_PACIFICA]]]

    # Combine each grouping of test case data files
    ihp2_bag_files = slr_bag_files + slf_bag_files + alr_bag_files + alf_bag_files + sh_mid_bag_files + sh_high_bag_files

    # Loop to conduct data analysis on each grouping of test run rosbags:
    test_num = 0
    for test_case_files in ihp2_bag_files:
        test_num += 1
        print("*****************************************************************")
        if test_case_files in slr_bag_files:
            test_type = "SLR"
            print("IHP2 - SLR Test Case")
        elif test_case_files in slf_bag_files:
            test_type = "SLF"
            print("IHP2 - SLF Test Case")
        elif test_case_files in alr_bag_files:
            test_type = "ALR"
            print("IHP2 - ALR Test Case")
        elif test_case_files in alf_bag_files:
            test_type = "ALF"
            print("IHP2 - ALF Test Case")
        elif test_case_files in sh_mid_bag_files:
            test_type = "SH-MID"
            print("IHP2 - SH MID Test Case")
        elif test_case_files in sh_high_bag_files:
            test_type = "SH-HIGH"
            print("IHP2 - SH High Test Case")
        else:
            print("Unknown test run type being processed.")
            continue
        
        # Store the rosbag and vehicle type for each vehicle in the lineup (Vehicle 1, Vehicle 2, Vehicle 3)
        bag_files_list = test_case_files[0]
        vehicle_types = test_case_files[1]
        for file_number in range(0, len(bag_files_list)):
            print("--")
            bag_file = bag_files_list[file_number]
            if file_number == 0:
                vehicle_number = 1
                vehicle_1_rosbag_name = bag_files_list[file_number]
                vehicle_1_type = vehicle_types[0]
            elif file_number == 1:
                vehicle_number = 2
                vehicle_2_rosbag_name = bag_files_list[file_number]
                vehicle_2_type = vehicle_types[1]
            elif file_number == 2:
                vehicle_number = 3
                vehicle_3_rosbag_name = bag_files_list[file_number]
                vehicle_3_type = vehicle_types[2]

            print("Processing new bag for vehicle " + str(file_number+1) + ": " + str(bag_file) + " for " + str(vehicle_types[file_number]))
                
            # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
            # sys.stdout = orig_stdout
            print("Processing bag file " + str(bag_file) + " (" + str(ihp2_bag_files.index(test_case_files) + 1) + " of " + str(len(ihp2_bag_files)) + ")")
            # sys.stdout = text_log_file_writer

            # Process bag file if it exists and can be processed, otherwise skip and proceed to next bag file
            try:
                print("Starting To Process Bag at " + str(datetime.datetime.now()))
                bag_file_path = str(source_folder) + "/" + bag_file

                if vehicle_number == 1:
                    vehicle_1_rosbag = rosbag.Bag(bag_file_path)
                elif vehicle_number == 2:
                    vehicle_2_rosbag = rosbag.Bag(bag_file_path)
                elif vehicle_number == 3:
                    vehicle_3_rosbag = rosbag.Bag(bag_file_path)

                print("Finished Processing Bag at " + str(datetime.datetime.now()))
            except:
                print("Skipping " + str(bag_file) +", unable to open or process bag file.")
                continue

            # Get the rosbag times associated with the starting engagement and ending engagement for the Basic Travel use case test
            print("Getting engagement times at " + str(datetime.datetime.now()))
            if vehicle_number == 1:
                time_start_engagement, time_end_engagement, found_test_times = get_test_case_engagement_times(vehicle_1_rosbag)
                time_1_start_engagement = time_start_engagement
                time_1_end_engagement = time_end_engagement
            if vehicle_number == 2:
                time_start_engagement, time_end_engagement, found_test_times = get_test_case_engagement_times(vehicle_2_rosbag)
                time_2_start_engagement = time_start_engagement
                time_2_end_engagement = time_end_engagement
            if vehicle_number == 3:
                time_start_engagement, time_end_engagement, found_test_times = get_test_case_engagement_times(vehicle_3_rosbag)
                time_3_start_engagement = time_start_engagement
                time_3_end_engagement = time_end_engagement
            print("Got engagement times at " + str(datetime.datetime.now()))

            if (not found_test_times):
                print("Could not find test case engagement start and end times in bag file.")
                #continue
            else:
                print("Began engagement at " + str(time_start_engagement.to_sec()))
                print("Ended engagement at " + str(time_end_engagement.to_sec()))
                print("Time spent engaged: " + str((time_end_engagement - time_start_engagement).to_sec()) + " seconds")
            
            if vehicle_number == 1:
                speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_1_rosbag)
                veh_1_speed_limit_zone_change_times = speed_limit_zone_change_times
                #generate_speed_plot(vehicle_1_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
                #generate_crosstrack_plot(vehicle_1_rosbag, time_start_engagement, time_end_engagement, bag_file)
            if vehicle_number == 2:
                speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_2_rosbag)
                #generate_speed_plot(vehicle_2_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
                #generate_crosstrack_plot(vehicle_2_rosbag, time_start_engagement, time_end_engagement, bag_file)
            if vehicle_number == 3:
                speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_3_rosbag)
                #generate_speed_plot(vehicle_3_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
                #generate_crosstrack_plot(vehicle_3_rosbag, time_start_engagement, time_end_engagement, bag_file)
        
        if test_type == "SLR":
            # Vehicle 2 joins Vehicle 1 from rear; then Vehicle 3 joins 1/2 from Rear

            # Platoon Formation - Joiner Perspective (IHP2 metrics 1,2,3,7,8,9)
            vehicle_2_platooning_times = joiner_platoon_formation_analysis(vehicle_2_rosbag, test_type, vehicle_1_type, vehicle_2_type, 2)
            vehicle_3_platooning_times = joiner_platoon_formation_analysis(vehicle_3_rosbag, test_type, vehicle_1_type, vehicle_3_type, 3)
            time_vehicle_1_platooning_start = vehicle_2_platooning_times[1]

            print("DEBUG: Vehicle 1 Platoon Start Time: " + str(time_vehicle_1_platooning_start.to_sec()))
            print("DEBUG: Vehicle 2 Platoon Start Time: " + str(vehicle_2_platooning_times[1].to_sec()))
            print("DEBUG: Vehicle 3 Platoon Start Time: " + str(vehicle_3_platooning_times[1].to_sec()))

            received_tcm_analysis(vehicle_1_rosbag, time_vehicle_1_platooning_start, test_type, 1, expected_tcm_count=7)
            received_tcm_analysis(vehicle_2_rosbag, vehicle_2_platooning_times[1], test_type, 2, expected_tcm_count=7)
            received_tcm_analysis(vehicle_3_rosbag, vehicle_3_platooning_times[1], test_type, 3, expected_tcm_count=7)

            print("IHP2-6 N/A for SLR Test; no platoon-specific TCMs")
            print("IHP2-10 N/A for SLR Test; TCM speed limit matches original speed limit")
            print("IHP2-11 N/A for SLR Test; no headway-specific TCMs")
            print("IHP2-12 N/A for SLR Test; TCM speed limit matches original speed limit")
            print("IHP2-13 N/A for SLR Test; no headway-specific TCMs")

            downtrack_distance_analysis(vehicle_2_rosbag, vehicle_1_rosbag, test_type, 2, 1, vehicle_2_platooning_times[0])
            downtrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[0])

            #generate_platooning_plot(vehicle_2_rosbag, vehicle_2_platooning_times[1], vehicle_2_rosbag_name, test_type, 2)
            #generate_platooning_plot(vehicle_3_rosbag, vehicle_3_platooning_times[1], vehicle_3_rosbag_name, test_type, 3)

            #generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_1_start_engagement, time_1_end_engagement, test_type, test_num)
        
        elif test_type == "SLF":
            # Vehicle 3 joins Vehicle 2 from rear; then Vehicle 1 joins 2/3 from Front

            vehicle_1_platooning_times = joiner_platoon_formation_analysis(vehicle_1_rosbag, test_type, vehicle_2_type, vehicle_1_type, 1)
            vehicle_3_platooning_times = joiner_platoon_formation_analysis(vehicle_3_rosbag, test_type, vehicle_2_type, vehicle_3_type, 3)
            
            downtrack_distance_analysis(vehicle_1_rosbag, vehicle_2_rosbag, test_type, 1, 2, vehicle_1_platooning_times[0])
            downtrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[0])

            #generate_platooning_plot(vehicle_3_rosbag, vehicle_3_platooning_times[1], vehicle_3_rosbag_name, test_type, 3)
            #generate_platooning_plot(vehicle_2_rosbag, vehicle_1_platooning_times[1], vehicle_2_rosbag_name, test_type, 2)

            #generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_2_start_engagement, time_2_end_engagement, test_type, test_num)

        elif test_type == "ALR":
            # Vehicle 2 joins Vehicle 1 from rear; then Vehicle 3 joins 1/2 from adjacent rear

            vehicle_2_platooning_times = joiner_platoon_formation_analysis(vehicle_2_rosbag, test_type, vehicle_1_type, vehicle_2_type, 2)
            vehicle_3_platooning_times = joiner_platoon_formation_analysis(vehicle_3_rosbag, test_type, vehicle_1_type, vehicle_3_type, 3)

            downtrack_distance_analysis(vehicle_2_rosbag, vehicle_1_rosbag, test_type, 2, 1, vehicle_2_platooning_times[0])

            downtrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[0])
            crosstrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[2])
            
            #generate_platooning_plot(vehicle_3_rosbag, vehicle_3_platooning_times[2], vehicle_3_rosbag_name, test_type, 3)
            #generate_platooning_plot(vehicle_2_rosbag, vehicle_2_platooning_times[1], vehicle_2_rosbag_name, test_type, 2)

            #generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_1_start_engagement, time_1_end_engagement, test_type, test_num)
        
        elif test_type == "ALF":
            # Vehicle 3 joins Vehicle 2 from rear; then Vehicle 1 joins 2/3 from adjacent front

            vehicle_3_platooning_times = joiner_platoon_formation_analysis(vehicle_3_rosbag, test_type, vehicle_2_type, vehicle_3_type, 3)
            vehicle_1_platooning_times = joiner_platoon_formation_analysis(vehicle_1_rosbag, test_type, vehicle_2_type, vehicle_1_type, 1)

            downtrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[0])

            downtrack_distance_analysis(vehicle_1_rosbag, vehicle_2_rosbag, test_type, 1, 2, vehicle_1_platooning_times[0])
            crosstrack_distance_analysis(vehicle_1_rosbag, vehicle_2_rosbag, test_type, 1, 2, vehicle_1_platooning_times[2])

            #generate_platooning_plot(vehicle_3_rosbag, vehicle_3_platooning_times[1], vehicle_3_rosbag_name, test_type, 3)
            #generate_platooning_plot(vehicle_2_rosbag, vehicle_1_platooning_times[2], vehicle_2_rosbag_name, test_type, 2)

            #generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_2_start_engagement, time_2_end_engagement, test_type, test_num)
        
        elif test_type == "SH-MID":
            # Vehicle 2 joins vehicle 1 from rear

            vehicle_2_platooning_times = joiner_platoon_formation_analysis(vehicle_2_rosbag, test_type, vehicle_1_type, vehicle_2_type, 2)
            downtrack_distance_analysis(vehicle_2_rosbag, vehicle_1_rosbag, test_type, 2, 1, vehicle_2_platooning_times[0])
            received_tcm_analysis(vehicle_1_rosbag, vehicle_2_platooning_times[1], test_type, 1, expected_tcm_count=10)
            received_tcm_analysis(vehicle_2_rosbag, vehicle_2_platooning_times[1], test_type, 2, expected_tcm_count=10)

            #generate_platooning_plot(vehicle_2_rosbag, vehicle_2_platooning_times[1], vehicle_2_rosbag_name, test_type, 2)
            #generate_two_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, time_1_start_engagement, time_1_end_engagement, veh_1_speed_limit_zone_change_times, test_type, test_num)
        
        elif test_type == "SH-HIGH":
            # Vehicle 2 joins vehicle 1 from rear

            vehicle_2_platooning_times = joiner_platoon_formation_analysis(vehicle_2_rosbag, test_type, vehicle_1_type, vehicle_2_type, 2)
            downtrack_distance_analysis(vehicle_2_rosbag, vehicle_1_rosbag, test_type, 2, 1, vehicle_2_platooning_times[0])
            received_tcm_analysis(vehicle_1_rosbag, vehicle_2_platooning_times[1], test_type, 1, expected_tcm_count=10)
            received_tcm_analysis(vehicle_2_rosbag, vehicle_2_platooning_times[1], test_type, 2, expected_tcm_count=10)

            #generate_platooning_plot(vehicle_2_rosbag, vehicle_2_platooning_times[1], vehicle_2_rosbag_name, test_type, 2)
            #generate_two_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, time_1_start_engagement, time_1_end_engagement, veh_1_speed_limit_zone_change_times, test_type, test_num)

    print("Analysis complete")


if __name__ == "__main__":
    main()