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
# Run the following in a terminal:
#   sudo add-apt-repository ppa:deadsnakes/ppa
#   sudo apt-get update
#   sudo apt install python3.7
#   python3.7 -m pip install --upgrade pip
#   python3.7 -m pip install matplotlib
#   python3.7 -m pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag rospkg
#   python3.7 -m pip install lz4
#   python3.7 -m pip install roslz4 --extra-index-url https://rospypi.github.io/simple/
# In terminal, navigate to the directory that contains this python script and run the following:
#   python3.7 analyze_voices_sit1_rosbags.py <path to folder containing SIT1 .bag files> 

def generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_starting_vehicle_start_engagement, vehicle_1_platooning_times, vehicle_2_platooning_times, vehicle_3_platooning_times, test_type, test_num):
    # vehicle 2 and 3 platooning times: [time_veh1_engaged, time_received_initial_ack, time_received_second_ack, time_veh1_left_platoon]
    # vehicle 1 platooning times: [time_began_platooning_with_veh2, time_began_platooning_with_veh3, time_leader_end_engagement]
    
    if test_type == "VOICES-SIT1":
        # First vehicle to engage is vehicle 1

        # Get the true vehicle speed (m/s) and the associated time with each data point
        first = True
        vehicle_1_speed_times = []
        vehicle_1_speeds = []
        for topic, msg, t in vehicle_1_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_starting_vehicle_start_engagement, end_time = vehicle_1_platooning_times[2]): # time_start_engagement+time_duration):
            if first:
                time_1_start = t
                first = False
                continue

            vehicle_1_speed_times.append((t-time_1_start).to_sec())
            vehicle_1_speeds.append(msg.twist.linear.x) # Current speed in m/s

        first = True
        vehicle_2_speed_times = []
        vehicle_2_speeds = []
        for topic, msg, t in vehicle_2_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = vehicle_2_platooning_times[0], end_time = vehicle_2_platooning_times[3]): # time_start_engagement+time_duration):
            if first:
                time_2_start = t
                first = False
                continue

            vehicle_2_speed_times.append((t-time_2_start).to_sec())
            vehicle_2_speeds.append(msg.twist.linear.x) # Current speed in m/s

        first = True
        vehicle_3_speed_times = []
        vehicle_3_speeds = []
        for topic, msg, t in vehicle_3_rosbag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = vehicle_3_platooning_times[0], end_time = vehicle_3_platooning_times[3]): # time_start_engagement+time_duration):
            if first:
                time_3_start = t
                first = False
                continue

            vehicle_3_speed_times.append((t-time_3_start).to_sec())
            vehicle_3_speeds.append(msg.twist.linear.x) # Current speed in m/s

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot vehicle 1 speed (m/s) vs. time
    ax.plot(vehicle_1_speed_times, vehicle_1_speeds, 'g:', label='Vehicle 1 Speed (m/s)')

    # Plot vehicle 2 speed (m/s) vs. time
    ax.plot(vehicle_2_speed_times, vehicle_2_speeds, 'b:', label='Vehicle 2 Speed (m/s)')

    # Plot vehicle 2 speed (m/s) vs. time
    ax.plot(vehicle_3_speed_times, vehicle_3_speeds, 'r:', label='Vehicle 3 Speed (m/s)')

    # Optional: Plot a vertical bar at the time that vehicle 2 joins the platoon
    #ax.axvline(x = (vehicle_2_platooning_times[2] - time_2_start).to_sec(), color = 'b', label = 'Time Vehicle 2 Joined the Platoon')

    # Optional: Plot a vertical bar at the time that vehicle 3 joins the platoon
    #ax.axvline(x = (vehicle_3_platooning_times[2] - time_3_start).to_sec(), color = 'r', label = 'Time Vehicle 3 Joined the Platoon')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(test_type) + " Run " + str(test_num) + " Vehicle Speeds") # Plot Title
    ax.set_xlabel("Time (seconds) Since Lead Vehicle Became Engaged") # Plot X Title
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

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(bag_file_name) + " Speed (Commanded and Actual) -- Vehicle " + str(vehicle_number)) # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title

    # Option 1: Save the plot
    filename = str(test_type) + "_" + bag_file_name + ".png"
    plt.savefig(filename, bbox_inches='tight')
    plt.close()

    # Option 2: Display the plot
    #plt.show() 

    return

def generate_platooning_plot(bag, time_start_platooning, time_end_platooning, bag_file_name, test_type, vehicle_number):

    actual_gaps = []
    desired_gaps = []
    times = []
    first = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_start_platooning, end_time = time_end_platooning):
        if first:
            time_start = t
            first = False
            continue 
        
        if msg.actual_gap < 150 and msg.actual_gap > -150:
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
    ax.set_xlabel("Time (seconds) Since Joining Platoon") # Plot X Title
    ax.set_ylabel("Meters") # Plot Y Title

    # Option 1: Save the plot
    filename = str(test_type) + "_Platooning_Information_" + bag_file_name + ".png"
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
    #plt.savefig(filename, bbox_inches='tight')
    #plt.close()
    
    # Option 2: Display the plot
    plt.show() 

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


def leader_platoon_formation_analysis(bag, run_type, leader_id, second_vehicle_id, third_vehicle_id, time_leader_end_engagement):

    if run_type == "VOICES-SIT1":
        has_began_platooning_with_veh2 = False
        time_began_platooning_with_veh2 = rospy.Time()
        has_began_platooning_with_veh3 = False
        time_began_platooning_with_veh3 = rospy.Time()
        ack_num = 0
        for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_response']):
            if msg.header.sender_id == leader_id and msg.header.recipient_id == second_vehicle_id:
                if msg.is_accepted:
                    ack_num += 1
                    
                    if ack_num == 2:
                        has_began_platooning_with_veh2 = True
                        time_began_platooning_with_veh2 = t
                        print("Leader began platooning with veh2 at " + str(t.to_sec()))
            elif msg.header.sender_id == leader_id and msg.header.recipient_id == third_vehicle_id:
                if msg.is_accepted:
                    ack_num += 1

                    if ack_num == 4:
                        has_began_platooning_with_veh3 = True
                        time_began_platooning_with_veh3 = t
                        print("Leader began platooning with veh3 at " + str(t.to_sec()))
                        break


        return [time_began_platooning_with_veh2, time_began_platooning_with_veh3, time_leader_end_engagement]

def joiner_platoon_formation_analysis(bag, run_type, leader_id, follower_id, joiner_vehicle_number):

    has_sent_initial_request = False
    time_sent_initial_request = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request']):
        if msg.header.recipient_id == leader_id and msg.header.sender_id == follower_id:
            if run_type == "VOICES-SIT1":
                if msg.plan_type.type == 3:
                    has_sent_initial_request = True
                    time_sent_initial_request = t
                    print("DEBUG: Vehicle " + str(joiner_vehicle_number) + " sent initial request at " + str(t.to_sec()))

    has_received_initial_ack = False
    time_received_initial_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response']):
        if msg.header.recipient_id == follower_id and msg.header.sender_id == leader_id:
            if run_type == "VOICES-SIT1":
                #if msg.plan_type.type == 3 and msg.is_accepted:
                if msg.is_accepted:
                    has_received_initial_ack = True
                    time_received_initial_ack = t
                    print("DEBUG: Vehicle " + str(joiner_vehicle_number) + " received ACK at " + str(t.to_sec()))
                    break

    has_sent_second_request = False
    time_sent_second_request = rospy.Time()
    first = True
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request']):
        if msg.header.recipient_id == leader_id and msg.header.sender_id == follower_id:
            if run_type == "VOICES-SIT1":
                if msg.plan_type.type == 4:
                    if first:
                        has_sent_second_request = True
                        time_sent_second_request = t
                        first = False
                    print("DEBUG: Vehicle " + str(joiner_vehicle_number) + " sent second request at " + str(t.to_sec()))

    has_received_second_ack = False
    time_received_second_ack = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response']):
        if msg.header.recipient_id == follower_id and msg.header.sender_id == leader_id:
            if run_type == "VOICES-SIT1":
                if msg.is_accepted:
                    has_received_second_ack = True
                    time_received_second_ack = t
                    print("DEBUG: Vehicle " + str(joiner_vehicle_number) + " received ACK at " + str(t.to_sec()))

    total_formation_duration = (time_received_second_ack - time_sent_initial_request).to_sec()
    #print("DEBUG: Time between sending initial request and receiving ack: " + str((time_received_initial_ack - time_sent_initial_request).to_sec()) + " sec")
    #print("DEBUG: Time between sending second request and receiving ack: " + str((time_received_second_ack - time_sent_second_request).to_sec()) + " sec")
    #print("DEBUG: Time between sending initial request and receiving second ack: " + str(total_formation_duration) + " sec")
        
    if run_type == "VOICES-SIT1":
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

    if run_type == "VOICES-SIT1":
        # Values are the same for same-lane front join and same-lane rear join
        if distance_between_vehicles <= 90.0 or time_gap_between_vehicles <= 15.0:
            print("IHP2-17 (SLR) Succeeded: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
        else:
            print("IHP2-17 (SLR) Failed: Downtrack between join vehicle " + str(join_vehicle_number) + " and vehicle " + str(other_vehicle_number) + " was " + str(distance_between_vehicles) + " meters (<=90 desired), time gap was " + str(time_gap_between_vehicles) + " seconds (<=15 desired) at JOIN_PLATOON_AT_REAR")
    
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
    SILVER_LEXUS = "DOT-45243"
    SPR = "CARMA-SPR"
    AUG = "CARMA-AUG"

    bag_file = "_2022-07-28-18-35-40.bag"
    bag_file_path = str(source_folder) + "/" + bag_file
    bag = rosbag.Bag(bag_file_path)  
    num = 0 
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        print("***********************************************")
        print(msg)
        num += 1
        if num == 5:
            break

    sit1_bag_files = [[["veh1_run2_2022-07-28-16-05-55-run2-veh1.bag",
                           "veh2_run2_2022-07-28-16-08-05.bag",
                           "veh3_run2_2022-07-28-16-07-02.bag"], 
                           [SILVER_LEXUS, AUG, SPR]]]

    # Loop to conduct data analysis on each grouping of test run rosbags:
    test_num = 0
    for test_case_files in sit1_bag_files:
        test_num += 1
        print("*****************************************************************")
        if test_case_files in sit1_bag_files:
            test_type = "VOICES-SIT1"
            print("VOICES - SIT1 Test Case")
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
            print("Processing bag file " + str(bag_file) + " (" + str(sit1_bag_files.index(test_case_files) + 1) + " of " + str(len(sit1_bag_files)) + ")")
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
            
            # if vehicle_number == 1:
            #     speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_1_rosbag)
            #     veh_1_speed_limit_zone_change_times = speed_limit_zone_change_times
            #     #generate_speed_plot(vehicle_1_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
            #     #generate_crosstrack_plot(vehicle_1_rosbag, time_start_engagement, time_end_engagement, bag_file)
            # if vehicle_number == 2:
            #     speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_2_rosbag)
            #     #generate_speed_plot(vehicle_2_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
            #     #generate_crosstrack_plot(vehicle_2_rosbag, time_start_engagement, time_end_engagement, bag_file)
            # if vehicle_number == 3:
            #     speed_limit_zone_values, speed_limit_zone_change_times = analyze_route_speed_limits(vehicle_3_rosbag)
            #     #generate_speed_plot(vehicle_3_rosbag, time_start_engagement, time_end_engagement, bag_file, speed_limit_zone_change_times, test_type, vehicle_number)
            #     #generate_crosstrack_plot(vehicle_3_rosbag, time_start_engagement, time_end_engagement, bag_file)
        
        if test_type == "VOICES-SIT1":
            # Vehicle 2 joins Vehicle 1 from rear; then Vehicle 3 joins 1/2 from Rear

            # Platoon Formation - Joiner Perspective (IHP2 metrics 1,2,3,7,8,9)
            vehicle_2_platooning_times = joiner_platoon_formation_analysis(vehicle_2_rosbag, test_type, vehicle_1_type, vehicle_2_type, 2) # [time_received_initial_ack, time_received_second_ack]
            vehicle_3_platooning_times = joiner_platoon_formation_analysis(vehicle_3_rosbag, test_type, vehicle_1_type, vehicle_3_type, 3) # [time_received_initial_ack, time_received_second_ack]
            vehicle_1_platooning_times = leader_platoon_formation_analysis(vehicle_1_rosbag, test_type, vehicle_1_type, vehicle_2_type, vehicle_3_type, time_1_end_engagement) # [time_began_platooning_with_veh2, time_began_platooning_with_veh3, time_leader_end_engagement]

            vehicle_1_platoon_duration = (vehicle_1_platooning_times[2] - vehicle_1_platooning_times[0])
            vehicle_2_platoon_duration = (vehicle_1_platooning_times[2] - vehicle_1_platooning_times[0])
            vehicle_3_platoon_duration = (vehicle_1_platooning_times[2] - vehicle_1_platooning_times[1])
            vehicle_2_platooning_times.append(vehicle_2_platooning_times[1] + vehicle_2_platoon_duration) # Now [time_received_initial_ack, time_received_second_ack, time_veh1_left_platoon]
            vehicle_3_platooning_times.append(vehicle_3_platooning_times[1] + vehicle_3_platoon_duration) # Now [time_received_initial_ack, time_received_second_ack, time_veh1_left_platoon]


            print("Vehicle 1 platoon duration: " + str(vehicle_1_platoon_duration.to_sec()))
            print("Vehicle 2 platoon duration: " + str(vehicle_2_platoon_duration.to_sec()))
            print("Vehicle 3 platoon duration: " + str(vehicle_3_platoon_duration.to_sec()))

            print("DEBUG: Vehicle 1 Platoon Start Time: " + str(vehicle_1_platooning_times[1].to_sec()))
            print("DEBUG: Vehicle 1 Platoon End Time: " + str(vehicle_1_platooning_times[2].to_sec()))

            print("DEBUG: Vehicle 2 Platoon Start Time: " + str(vehicle_2_platooning_times[1].to_sec()))
            print("DEBUG: Vehicle 2 Platoon End Time: " + str(vehicle_2_platooning_times[2].to_sec()))

            print("DEBUG: Vehicle 3 Platoon Start Time: " + str(vehicle_3_platooning_times[1].to_sec()))
            print("DEBUG: Vehicle 3 Platoon End Time: " + str(vehicle_3_platooning_times[2].to_sec()))

            vehicle_1_engagement_duration = time_1_end_engagement - time_1_start_engagement
            vehicle_2_platooning_times.insert(0, vehicle_2_platooning_times[2] - vehicle_1_engagement_duration) # Now [time_veh1_engaged, time_received_initial_ack, time_received_second_ack, time_veh1_left_platoon]
            vehicle_3_platooning_times.insert(0, vehicle_3_platooning_times[2] - vehicle_1_engagement_duration) # Now [time_veh1_engaged, time_received_initial_ack, time_received_second_ack, time_veh1_left_platoon]  

            print("Vehicle 1: " + str(vehicle_1_platooning_times))
            print("Vehicle 2: " + str(vehicle_2_platooning_times))
            print("Vehicle 3: " + str(vehicle_3_platooning_times))

            #received_tcm_analysis(vehicle_1_rosbag, time_vehicle_1_platooning_start, test_type, 1, expected_tcm_count=7)
            #received_tcm_analysis(vehicle_2_rosbag, vehicle_2_platooning_times[1], test_type, 2, expected_tcm_count=7)
            #received_tcm_analysis(vehicle_3_rosbag, vehicle_3_platooning_times[1], test_type, 3, expected_tcm_count=7)

            print("IHP2-6 N/A for SLR Test; no platoon-specific TCMs")
            print("IHP2-10 N/A for SLR Test; TCM speed limit matches original speed limit")
            print("IHP2-11 N/A for SLR Test; no headway-specific TCMs")
            print("IHP2-12 N/A for SLR Test; TCM speed limit matches original speed limit")
            print("IHP2-13 N/A for SLR Test; no headway-specific TCMs")

            #downtrack_distance_analysis(vehicle_2_rosbag, vehicle_1_rosbag, test_type, 2, 1, vehicle_2_platooning_times[0])
            #downtrack_distance_analysis(vehicle_3_rosbag, vehicle_2_rosbag, test_type, 3, 2, vehicle_3_platooning_times[0])

            generate_platooning_plot(vehicle_2_rosbag, vehicle_2_platooning_times[2], vehicle_2_platooning_times[3], vehicle_2_rosbag_name, test_type, 2)
            generate_platooning_plot(vehicle_3_rosbag, vehicle_3_platooning_times[2], vehicle_3_platooning_times[3], vehicle_3_rosbag_name, test_type, 3)

            # Get vehicle 2 time when vehicle 1 engages

            # Get vehicle 3 time when vehicle 1 engages

            generate_three_vehicle_speed_plot(vehicle_1_rosbag, vehicle_2_rosbag, vehicle_3_rosbag, time_1_start_engagement, vehicle_1_platooning_times, vehicle_2_platooning_times, vehicle_3_platooning_times, test_type, test_num=2)

    print("Analysis complete")


if __name__ == "__main__":
    main()