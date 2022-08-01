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
#   python3.7 analyze_basic_travel_rosbags.py <path to folder containing Basic Travel Use Case .bag files>

# Helper Function: Get times associated with the system entering the geofence and exiting the geofence
def get_geofence_entrance_and_exit_times(bag):
    # Initialize geofence entrance and exit times
    time_enter_active_geofence = rospy.Time() # Time that the vehicle has entered the geofence
    time_exit_active_geofence = rospy.Time() # Time that the vehicle has exited the geofence--------------------------------------------------------------------------------------------------------

    # Find geofence entrance and exit times
    is_on_active_geofence = False
    found_geofence_entrance_time = False
    found_geofence_exit_time = False
    for topic, msg, t in bag.read_messages(topics=['/environment/active_geofence']):
        # If first occurrence of being in the active geofence, set the start time
        if (msg.is_on_active_geofence and not is_on_active_geofence):
            time_enter_active_geofence = t
            #print("Entered geofence at " + str(t.to_sec()))
            found_geofence_entrance_time = True
            is_on_active_geofence = True
        
        # If final occurrence of being in the active geofence, set the end time
        if (not msg.is_on_active_geofence and is_on_active_geofence):
            time_exit_active_geofence = t
            found_geofence_exit_time = True
            break

    # Check if both geofence start and end time were found
    found_geofence_times = False
    if (found_geofence_entrance_time and found_geofence_exit_time):
        found_geofence_times = True
    
    return time_enter_active_geofence, time_exit_active_geofence, found_geofence_times

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

# Helper function: Begin with the time that the vehicle exits the active geofence according to  
#                  /guidance/active_geofence topic. This helper function adjusts the time to be 
#                  based on /guidance/route_state in order to be more accurate.
def adjust_geofence_exit_time(bag, time_exit_geofence, original_speed_limit):
    # Get the true time of the end of the geofence, based on when /guidance/route_state displays the
    #         original speed limit for the current vehicle location
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_exit_geofence):
        if msg.speed_limit == original_speed_limit:
            true_time_exit_geofence = t
            break

    return true_time_exit_geofence

# Helper Function: Get start and end times of the period of engagement that includes the in-geofence section
def get_test_case_engagement_times(bag, time_enter_active_geofence, time_exit_active_geofence):
    # Initialize system engagement start and end times
    time_start_engagement = rospy.Time()
    time_stop_engagement = rospy.Time()

    # Loop through /guidance/state messages to determine start and end times of engagement that include the in-geofence section
    is_engaged = False
    found_engagement_times = False
    has_reached_geofence_entrance = False
    has_reached_geofence_exit = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/state']):
        # If entering engagement, track this start time
        if (msg.state == 4 and not is_engaged):
            time_start_engagement = t
            is_engaged = True

        # Store the last recorded engagement timestamp in case CARMA ends engagement before a new guidance
        #       state can be published.
        if (msg.state == 4):
            time_last_engaged = t
        
        # If exiting engagement, check whether this period of engagement included the geofence entrance and exit times
        elif (msg.state != 4 and is_engaged):
            is_engaged = False
            time_stop_engagement = t

            # Check if the engagement start time was before the geofence entrance and exit times
            if (time_start_engagement <= time_enter_active_geofence and t >= time_enter_active_geofence):
                has_reached_geofence_entrance = True
            if (time_start_engagement <= time_exit_active_geofence and t >= time_exit_active_geofence):
                has_reached_geofence_exit = True

            # Set flag if this engagement period includes both the geofence entrance and exit times
            if (has_reached_geofence_entrance and has_reached_geofence_exit):
                found_test_case_engagement_times = True
                break
    
    # If CARMA ended engagement before guidance state could be updated, check if the last recorded
    #    time of engagement came after exiting the geofence
    if not found_engagement_times:
        if time_last_engaged >= time_exit_active_geofence:
            time_stop_engagement = time_last_engaged
            found_engagement_times = True
    
    return time_start_engagement, time_stop_engagement, found_engagement_times

# Helper Function: Get the lanelet IDs that are included in the geofence.
def get_geofence_lanelets(bag, time_start_engagement, advisory_speed_limit):
    # Loop through route_state topic and add all unique 
    lanelets_in_geofence = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement):
        if (msg.speed_limit - advisory_speed_limit < 0.01):
            if msg.lanelet_id not in lanelets_in_geofence:
                lanelets_in_geofence.append(msg.lanelet_id)

    print("Lanelet IDs in the geofence: " + str(lanelets_in_geofence))
    
    return lanelets_in_geofence

# Helper Function: Get the route downtrack associated with the geofence entrance
def get_geofence_entrance_downtrack(bag, time_enter_geofence):

    # Return the first 'downtrack' published to /guidance/route_state after entering the geofence
    downtrack_enter_geofence = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_geofence):
        downtrack_enter_geofence = msg.down_track
        break

    print("Downtrack of geofence entrance: " + str(downtrack_enter_geofence) + " meters")

    return downtrack_enter_geofence

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

# Helper Function: Get all important timestamps for platoon negotiation from the Rear Follower Vehicle's perspective
#        Message 1: Follower Broadcasts "JOIN_PLATOON_AT_REAR" MobilityRequest
#        Message 2: Follower Receives MobilityRequest from Leader with is_accepted==True
#        Message 3: Follower Broadcasts "PLATOON_FOLLOWER_JOIN" MobilityRequest
#        Message 4: Follower Receives MobilityRequest from Leader with is_accepted==True
def get_follower_platoon_negotiation_times(bag, time_test_start_engagement):
    # Obtain times that vehicle receives an 'is_accepted' mobility_request message
    # Note: First acceptance is in response to Follower vehicle's 'JOIN_PLATOON_AT_REAR' MobilityRequest
    #       Second acceptance is in response to Follower vehicle's 'PLATOON_FOLLOWER_JOIN' MobilityRequest (End of Platoon Negotiation)
    time_received_first_acceptance = rospy.Time()
    time_received_second_acceptance = rospy.Time()
    has_received_first_acceptance = False
    has_received_second_acceptance = False
    has_received_additional_acceptances = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_response'], start_time = time_test_start_engagement):
        if msg.is_accepted == True:
            if not has_received_first_acceptance:
                print("Received first MobilityRequest with is_accepted==True at time " + str(t))
                time_received_first_acceptance = t
                has_received_first_acceptance = True
            elif not has_received_second_acceptance:
                print("Received second MobilityRequest with is_accepted==True at time " + str(t))
                time_received_second_acceptance = t
                has_received_second_acceptance = True
            else:
                print("Error: Received additional MobilityRequest with is_accepted==True at time " + str(t))
                has_received_additional_acceptances = True

    # Obtain times that vehicle broadcasts its final 'JOIN_PLATOON_AT_REAR' and 'PLATOON_FOLLOWER_JOIN' MobilityRequest messages
    # Note: 'JOIN_PLATOON_AT_REAR' is request from the Follower vehicle to join the platoon at its rear (Beginning of Platoon Negotiation)
    #       'PLATOON_FOLLOWER_JOIN' is indication that the Follower vehicle is officially joining the platoon
    has_sent_last_join_at_rear = False
    has_sent_last_follower_join = False
    has_sent_additional_join_at_rear = False
    has_sent_additional_follower_join = False
    time_last_sent_join_at_rear = rospy.Time()
    time_last_sent_follower_join = rospy.Time()
    previous_plan_type = -1
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_request'], start_time = time_test_start_engagement):
        # Track whether the previous MobilityRequest plan_type was the last 'JOIN_PLATOON_AT_REAR' message
        if previous_plan_type == 3 and previous_plan_type != msg.plan_type.type:
            if has_sent_last_join_at_rear:
                print("Error: New 'last' sent 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_sent_join_at_rear))
                has_sent_additional_join_at_rear = True
            else:
                has_sent_last_join_at_rear = True
                print("Sent last 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_sent_join_at_rear))
        # Track whether the previous MobilityRequest was the last 'PLATOON_FOLLOWER_JOIN' message
        elif previous_plan_type == 4 and previous_plan_type != msg.plan_type.type:
            if has_sent_last_follower_join:
                print("Error: New 'last' sent 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_sent_join_at_rear))
                has_sent_additional_follower_join = True
            else:
                has_sent_last_follower_join = True
                print("Sent last 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_sent_follower_join))
        
        # Update 'previous' values based on the current timestamp and plan type
        if msg.plan_type.type == 3:
            time_last_sent_join_at_rear = t
            previous_plan_type = msg.plan_type.type
        elif msg.plan_type.type == 4:
            time_last_sent_follower_join = t
            previous_plan_type = msg.plan_type.type
    
    # Track whether the last broadcasted MobilityRequest message was a 'JOIN_PLATOON_AT_REAR' message
    if previous_plan_type == 3:
        if has_sent_last_join_at_rear:
            print("Error: New 'last' sent 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_sent_join_at_rear))
            has_sent_additional_join_at_rear = True
        else:
            print("Sent last 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_sent_join_at_rear)) 
            has_sent_last_join_at_rear = True  
    # Track whether the last broadcasted MobilityRequest message was a 'PLATOON_FOLLOWER_JOIN' message         
    elif previous_plan_type == 4:
        if has_sent_last_follower_join:
            print("Error: New 'last' sent 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_sent_join_at_rear))
            has_sent_additional_follower_join = True
        else:
            print("Sent last 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_sent_follower_join))
            has_sent_last_follower_join = True
    
    if has_received_additional_acceptances or has_sent_additional_join_at_rear or has_sent_additional_follower_join:
        print("WARNING: Platooning metrics may be incorrect; multiple occurrences of one or more negotiation messages has occurred.")
    else:
        print("Platooning negotiation was successful.")

    return time_last_sent_follower_join, time_last_sent_join_at_rear, time_received_first_acceptance, time_received_second_acceptance

# Helper Function: Get all important timestamps for platoon negotiation from the Front Vehicle's perspective
#        Message 1: Leader receives "JOIN_PLATOON_AT_REAR" MobilityRequest
#        Message 2: Leader broadcasts MobilityRequest from Leader with is_accepted==True
#        Message 3: Leader  Broadcasts "PLATOON_FOLLOWER_JOIN" MobilityRequest
#        Message 4: Follower Receives MobilityRequest from Leader with is_accepted==True
def get_leader_platoon_negotiation_times(bag, time_test_start_engagement):
    # Obtain times that vehicle broadcasts an 'is_accepted' mobility_request message
    # Note: First acceptance is in response to Follower vehicle's 'JOIN_PLATOON_AT_REAR' MobilityRequest
    #       Second acceptance is in response to Follower vehicle's 'PLATOON_FOLLOWER_JOIN' MobilityRequest (End of Platoon Negotiation)
    time_sent_first_acceptance = rospy.Time()
    time_sent_second_acceptance = rospy.Time()
    has_sent_first_acceptance = False
    has_sent_second_acceptance = False
    has_sent_additional_acceptances = False
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_response'], start_time = time_test_start_engagement):
        if msg.is_accepted == True:
            if not has_sent_first_acceptance:
                print("Sent first MobilityRequest with is_accepted==True at time " + str(t))
                time_sent_first_acceptance = t
                has_sent_first_acceptance = True
            elif not has_sent_second_acceptance:
                print("Sent second MobilityRequest with is_accepted==True at time " + str(t))
                time_sent_second_acceptance = t
                has_sent_second_acceptance = True
            else:
                print("Error: Sent additional MobilityRequest with is_accepted==True at time " + str(t))
                has_sent_additional_acceptances = True

    # Obtain times that vehicle broadcasts its final 'JOIN_PLATOON_AT_REAR' and 'PLATOON_FOLLOWER_JOIN' MobilityRequest messages
    # Note: 'JOIN_PLATOON_AT_REAR' is request from the Follower vehicle to join the platoon at its rear (Beginning of Platoon Negotiation)
    #       'PLATOON_FOLLOWER_JOIN' is indication that the Follower vehicle is officially joining the platoon
    has_received_last_join_at_rear = False
    has_received_last_follower_join = False
    has_received_additional_join_at_rear = False
    has_received_additional_follower_join = False
    time_last_received_join_at_rear = rospy.Time()
    time_last_received_follower_join = rospy.Time()
    previous_plan_type = -1
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_request'], start_time = time_test_start_engagement):
        # Track whether the previous MobilityRequest plan_type was the last 'JOIN_PLATOON_AT_REAR' message
        if previous_plan_type == 3 and previous_plan_type != msg.plan_type.type:
            if has_received_last_join_at_rear:
                print("Error: New 'last' received 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_received_join_at_rear))
                has_received_additional_join_at_rear = True
            else:
                has_received_last_join_at_rear = True
                print("Received last 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_received_join_at_rear))
        # Track whether the previous MobilityRequest was the last 'PLATOON_FOLLOWER_JOIN' message
        elif previous_plan_type == 4 and previous_plan_type != msg.plan_type.type:
            if has_received_last_follower_join:
                print("Error: New 'last' received 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_received_join_at_rear))
                has_received_additional_follower_join = True
            else:
                has_received_last_follower_join = True
                print("Received last 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_received_follower_join))
        
        # Update 'previous' values based on the current timestamp and plan type
        if msg.plan_type.type == 3:
            time_last_received_join_at_rear = t
            previous_plan_type = msg.plan_type.type
        elif msg.plan_type.type == 4:
            time_last_received_follower_join = t
            previous_plan_type = msg.plan_type.type
    
    # Track whether the last broadcasted MobilityRequest message was a 'JOIN_PLATOON_AT_REAR' message
    if previous_plan_type == 3:
        if has_received_last_join_at_rear:
            print("Error: New 'last' received 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_received_join_at_rear))
            has_received_additional_join_at_rear = True
        else:
            has_received_last_join_at_rear = True
            print("Received last 'JOIN_PLATOON_AT_REAR' at time " + str(time_last_received_join_at_rear))   
    # Track whether the last broadcasted MobilityRequest message was a 'PLATOON_FOLLOWER_JOIN' message         
    elif previous_plan_type == 4:
        if has_received_last_follower_join:
            print("Error: New 'last' received 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_received_join_at_rear))
            has_received_additional_follower_join = True
        else:
            has_received_last_follower_join = True
            print("Received last 'PLATOON_FOLLOWER_JOIN' at time " + str(time_last_received_follower_join))
    
    if has_sent_additional_acceptances or has_received_additional_join_at_rear or has_received_additional_follower_join:
        print("WARNING: Platooning metrics may be incorrect; multiple occurrences of one or more negotiation messages has occurred.")
    else:
        print("Platooning negotiation was successful.")

    return time_last_received_follower_join, time_last_received_join_at_rear, time_sent_first_acceptance, time_sent_second_acceptance

# Helper Function: Obtain the total amount of time spent conducting platoon negotiation from the Rear Vehicle's perspective
# Note: Platoon Negotiation is time between the last broadcasted "JOIN_PLATOON_AT_REAR" message and 
#       the second received MobilityRequest from Leader with is_accepted==True
def get_platoon_negotiation_duration(time_end_negotiation, time_start_negotiation, is_leader = False):
    duration_platoon_negotiation = (time_end_negotiation - time_start_negotiation).to_sec()

    if is_leader:
        print("Time required for platoon negotiation: " + str(duration_platoon_negotiation) + " sec (LEADER)")
    else:
        print("Time required for platoon negotiation: " + str(duration_platoon_negotiation) + " sec (FOLLOWER)")

    return duration_platoon_negotiation

###########################################################################################################
# Basic Travel B-19: The information communicated by the CARMA Cloud vehicle is the advisory speed limit 
#                    in the geofenced area.
###########################################################################################################
def get_basic_travel_TCM_data(bag):
    # Check that TCM Message is received for max speed (an advisory speed limit)
    has_advisory_speed = False
    time_first_msg_received = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        if msg.tcmV01.params.detail.choice == 12:
            time_first_msg_received = t
            advisory_speed = msg.tcmV01.params.detail.maxspeed
            has_advisory_speed = True

            print("TCM Messages Received: Advisory Speed: " + str(advisory_speed) + " mph")

            break
    
    is_successful = False
    if has_advisory_speed:
        print("B-19 succeeded; TCM message received with advisory speed limit " + str(advisory_speed) + " mph")
        is_successful = True
    else:
        print("B-19 failed; no TCM message with an advisory speed limit was received.")
        is_successful = True

    return advisory_speed, time_first_msg_received, is_successful

###########################################################################################################
# BASIC TRAVEL B-1: Geofenced area is a part of the initial route plan.
###########################################################################################################
def check_basic_travel_geofence_lanelets_in_initial_route(bag, lanelets_in_geofence):
    # Get each set route from the bag file (includes set routes and updated/re-rerouted routes)
    shortest_path_lanelets = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route']):
        # Print Debug Statement that includes the list of lanelet IDs in the route:
        print("Shortest Path Route Update at " + str(t.to_sec()) + ": " + str(msg.shortest_path_lanelet_ids))
        
        shortest_path_lanelets.append([])
        for lanelet_id in msg.shortest_path_lanelet_ids:
            shortest_path_lanelets[-1].append(lanelet_id)

    # Check whether the geofence lanelets are included in the initial route
    initial_route_includes_geofence_lanelets = True # Flag for BT-B-1 success
    initial_route_lanelets = shortest_path_lanelets[0]
    for geofence_lanelet_id in lanelets_in_geofence:
        if geofence_lanelet_id not in initial_route_lanelets:
            print("Geofence Lanelet ID " + str(geofence_lanelet_id) + " is not in initial route.")
            initial_route_includes_geofence_lanelets = False
            break

    # Print result statements and return success flags
    if (initial_route_includes_geofence_lanelets):
        print("B-1 succeeded; all geofence lanelets " + str(lanelets_in_geofence) + " were in the initial route")
    else:
        print("B-1 failed: not all geofence lanelets " + str(lanelets_in_geofence) + " were in the initial route.")

    return initial_route_includes_geofence_lanelets

###########################################################################################################
# BASIC TRAVEL B-2: The vehicle receives a message from CARMA Cloud informing of an advisory speed limit 
#                   ahead. The vehicle processes this information.
###########################################################################################################
def check_geofence_lanelet_speed_limits(bag, lanelets_in_geofence, advisory_speed_limit):
    epsilon = 0.01
    has_correct_geofence_lanelet_speed_limits = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state']):

        # Failure if  the current lanelet is one of the geofence lanelets and its speed limit doesn't match the advisory speed limit
        if msg.lanelet_id in lanelets_in_geofence:
            if (abs(msg.speed_limit - advisory_speed_limit) >= epsilon):
                print("Lanelet ID " + str(msg.lanelet_id) + " has speed limit of " + str(msg.speed_limit) + " m/s. " + \
                      "Does not match advisory speed limit of " + str(advisory_speed_limit) + " m/s.")
                has_correct_geofence_lanelet_speed_limits = False
                break
    
    if has_correct_geofence_lanelet_speed_limits:
        print("B-2 succeeded; all geofence lanelets " + str(lanelets_in_geofence) + " had advisory speed limit of " + \
              str(advisory_speed_limit) + " m/s")
    else:
        print("B-2 failed; not all geofence lanelets had an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
    
    return has_correct_geofence_lanelet_speed_limits

###########################################################################################################
# Basic Travel B-3: The vehicle receives the message from CARMA Cloud, which informs of the speed limit 
#                   ahead, early enough that the vehicle could decelerate at a rate less than 2 m/s^2 
#                   in order to enter the geofence at the advisory speed limit.
###########################################################################################################
def check_time_received_first_TCM_message(bag, time_received_first_msg, time_enter_geofence, advisory_speed_limit):
    # (m/s^2) Maximum deceleration limit allowed to slow down for geofence's advisory speed limit
    max_decel_limit = 2.0 
    
    # Get the vehicle speed at the time the first TCM message is received
    speed_received_first_msg = 0.0
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_received_first_msg):
        speed_received_first_msg = msg.speed * 0.277777 # Conversion from kph to m/s
        break

    # Get the route downtrack at the time the first TCM message is received
    downtrack_received_first_msg = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_received_first_msg):
        downtrack_received_first_msg = msg.down_track
        break

    # Get the route downtrack at the time the geofence is entered
    downtrack_enter_geofence = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_geofence):
        downtrack_enter_geofence = msg.down_track
        break

    # Determine if the message was received early enough that the vehicle could decelerate in preparation for the
    # geofence's advisory speed limit without surpassing the maximum deceleration limit
    distance_for_decel = downtrack_enter_geofence - downtrack_received_first_msg
    decel_required = ((speed_received_first_msg**2) - (advisory_speed_limit**2)) / (2*distance_for_decel)
    if abs(decel_required) <= max_decel_limit:
        has_decel_within_limit = True
    
    is_successful = False
    if has_decel_within_limit:
        print("B-3 succeeded; TCM message received with deceleration of " + str(decel_required) + " m/s^2 required before geofence")
        is_successful = True
    else:
        print("B-3 failed; TCM message received with deceleration of " + str(decel_required) + " m/s^2 required before geofence.")
        is_successful = False

    return is_successful

def get_route_original_speed_limit(bag, time_test_start_engagement):
    original_speed_limit = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_test_start_engagement):
        original_speed_limit = msg.speed_limit
        break

    return original_speed_limit

###########################################################################################################
# Basic Travel B-5: The rear vehicle conducts a full lane merge by travelling through the following lanelets in order: 
#         (1) 96760, (2) 32982, (3) 34915
###########################################################################################################
def check_lane_merge_before_geofence(bag):
    lanelet_id_1 = 96760
    lanelet_id_2 = 32982
    lanelet_id_3 = 34915

    travels_from_lanelet_1_to_2 = False
    travels_from_lanelet_2_to_3 = False
    previous_lanelet_id = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state']):
        current_lanelet_id = msg.lanelet_id

        if previous_lanelet_id == lanelet_id_1 and current_lanelet_id == lanelet_id_2:
            travels_from_lanelet_1_to_2 = True

        if travels_from_lanelet_1_to_2:
            if previous_lanelet_id == lanelet_id_2 and current_lanelet_id == lanelet_id_3:
                travels_from_lanelet_2_to_3 = True
                break

        previous_lanelet_id = current_lanelet_id
    
    is_successful = False
    if travels_from_lanelet_1_to_2 and travels_from_lanelet_2_to_3:
        print("B-5 Succeeded: Vehicle travelled through the following lanelets in order: " + str(lanelet_id_1) + ", " + str(lanelet_id_2) + ", " + str(lanelet_id_3))
        is_successful = True
    else:
        print("B-5 Failed: Vehicle travelled through the following lanelets in order: " + str(lanelet_id_1) + ", " + str(lanelet_id_2) + ", " + str(lanelet_id_3))
        is_successful = False
    
    return is_successful


###########################################################################################################
# BASIC TRAVEL B-6: After the rear vehicle has completed its lane merge, the front vehicle will receive 
#                   the 'Join Platoon at Rear' MobilityRequest message from the rear vehicle. The front 
#                   vehicle shall respond with an 'Acknowledgement' MobilityRequestResponse message 
#                   signaling acceptance.
###########################################################################################################
def check_join_at_rear_negotiation(bag, time_received_first_acceptance, time_last_sent_join_at_rear, time_start_engagement, start_lane_following_lanelet):
    time_finish_lane_merge = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement):
        if msg.lanelet_id == start_lane_following_lanelet:
            time_finish_lane_merge = t
            break
    
    duration_first_acceptance_after_lane_merge = (time_received_first_acceptance - time_finish_lane_merge).to_sec()
    duration_first_acceptance_after_join_at_rear = (time_received_first_acceptance - time_last_sent_join_at_rear).to_sec()
    
    is_successful = False
    if duration_first_acceptance_after_join_at_rear > 0.0:
        is_successful = True
        print("B-6 succeeded; First acceptance received " + str(duration_first_acceptance_after_join_at_rear) + " sec after 'JOIN_PLATOON_AT_REAR")
        if duration_first_acceptance_after_lane_merge > 0.0:
            print("B-6 Note: First acceptance occurred " + str(round(duration_first_acceptance_after_lane_merge,2)) + " sec after end of lane merge")
        else:
            print("B-6 Note: First acceptance occurred " + str(round(-duration_first_acceptance_after_lane_merge,2)) + " sec before end of lane merge")
    else:
        is_successful = False
        print("B-6 failed; First acceptance received " + str(duration_first_acceptance_after_join_at_rear) + " sec after 'JOIN_PLATOON_AT_REAR")

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-7: After the rear vehicle receives the 'Acknowledgement' MobilityRequestResponse message 
#                   from the front vehicle signaling its acceptance for the rear vehicle to join its 
#                   platoon, the rear vehicle shall respond with a 'Platoon Follower Join' MobilityRequest 
#                   message to the front vehicle signaling that it is joining the front vehicle's platoon.
###########################################################################################################
def check_follower_join_negotiation(time_received_first_acceptance, time_last_sent_follower_join):
    duration_follower_join_after_first_acceptance = (time_last_sent_follower_join - time_received_first_acceptance).to_sec()
    
    is_successful = False
    if duration_follower_join_after_first_acceptance > 0.0:
        is_successful = True
        print("B-7 succeeded; 'PLATOON_FOLLOWER_JOIN' broadcasted " + str(duration_follower_join_after_first_acceptance) + " sec after first acceptance.")
    else:
        is_successful = False
        print("B-7 failed; 'PLATOON_FOLLOWER_JOIN' broadcasted " + str(duration_follower_join_after_first_acceptance) + " sec after first acceptance.")

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-8: When the rear vehicle broadcasts its 'Platoon Follower Join' MobilityRequest message 
#                   (the message signaling that it is joining the platoon), the time headway between the 
#                   lead vehicle and the rear vehicle shall be +/- 2.0 seconds of the desired 2.5 second time headway.
###########################################################################################################
def check_time_headway_after_platoon_negotiation(bag, time_received_second_acceptance):
    desired_time_headway = 2.5 # (seconds) Desired time headway between rear vehicle and front vehicle during platooning
    threshold_time_headway = 2.0 # (seconds) Allowable time headway offset from the desired value at initial platoon formation
    min_time_headway = desired_time_headway - threshold_time_headway # (seconds) 
    max_time_headway = desired_time_headway + threshold_time_headway # (seconds)
    
    # Obtain the distance headway between the rear vehicle and the front vehicle when the rear vehicle first becomes a 'Follower'
    has_found_actual_time_headway_at_platoon_start = False
    actual_gap = 0.0
    duration_since_second_acceptance = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_received_second_acceptance):
        if msg.state == 5:
            if (msg.actual_gap > 0):
                actual_speed = msg.desired_gap / desired_time_headway
                actual_time_headway = msg.actual_gap / actual_speed
                has_found_actual_time_headway_at_platoon_start = True

                # Obtain the duration between this first occurrence of being a 'Follower' and when the second acceptance message was received
                duration_since_second_acceptance = (t - time_received_second_acceptance).to_sec()
                break
    
    is_successful = False
    if has_found_actual_time_headway_at_platoon_start:
        # Successful if the time headway at platooning start is within the minimum and maximum values
        if (min_time_headway <= actual_time_headway <= max_time_headway):
            print("B-8 succeeded; Time headway at start of platooning is " + str(actual_time_headway) + " sec (" + str(duration_since_second_acceptance) + " sec after 2nd acceptance)")
            is_successful = True
        else:
            print("B-8 failed; Time headway at start of platooning is " + str(actual_time_headway) + " sec (" + str(duration_since_second_acceptance) + " sec after 2nd acceptance)")
            is_successful = False
    else:
        print("B-8 failed; Rear vehicle never published PlatooningInfo msg with 'FOLLOWING' status")
        is_successful = False
    
    return is_successful

###########################################################################################################
# BASIC TRAVEL B-9: After platoon negotiation is complete, the rear vehicle changes its platooning status 
#                   from 'CandidateFollower' to 'Follower'.
###########################################################################################################
def check_follower_state_after_platoon_negotiation(bag, time_last_sent_join_at_rear):
    has_candidate_follower_state = False # Flag to track whether there has been a 'CandidateFollower' state
    has_follower_state = False # Flag to track whether there has been a 'Follower' state
    duration_since_last_sent_join_at_rear = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_last_sent_join_at_rear):
        # If a 'CandidateFollower' state occurs for the first time, set the appropriate flag to True
        if not has_candidate_follower_state:
            if msg.state == 3:
                has_candidate_follower_state = True
        
        # If a 'Follower' state occurs for the first time, set the appropriate flag to True
        elif not has_follower_state:
            if msg.state == 5:
                # Obtain the time duration between the start of the 'Follower' state and time_last_sent_join_at_rear
                duration_since_last_sent_join_at_rear = (t - time_last_sent_join_at_rear).to_sec()
                has_follower_state = True
                break
    
    is_successful = False
    if has_candidate_follower_state and has_follower_state:
        print("B-9 successful; Rear vehicle transitioned from CANDIDATEFOLLOWER to FOLLOWER (" + str(duration_since_last_sent_join_at_rear) + " sec after 'JOIN_PLATOON_AT_REAR' msg)")
        is_successful = True
    elif has_candidate_follower_state:
        print("B-9 failed; Rear vehicle never transitioned to FOLLOWER")
        is_successful = False
    else:
        print("B-9 failed; Rear vehicle never transitioned to CANDIDATEFOLLOWER")
        is_successful = False
    
    return is_successful

###########################################################################################################
# BASIC TRAVEL B-10: After platoon negotiation is complete, the lead vehicle changes its platooning status 
#                    from 'LeaderWaiting' to 'Leader'.
###########################################################################################################
def check_leader_state_after_platoon_negotiation(bag, time_last_received_join_at_rear):
    has_leader_waiting_state = False # Flag to track whether there has been a 'LeaderWaiting' state
    has_leader_state = False # Flag to track whether there has been a 'Leader' state
    duration_since_last_received_join_at_rear = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_last_received_join_at_rear):
        # If this is the first occurrence of a 'LeaderWaiting' state, set the appropriate flag
        if not has_leader_waiting_state:
            if msg.state == 2:
                has_leader_waiting_state = True
        # If this is the first occurrence of a 'Leader' state, set the appropriate flag
        elif not has_leader_state:
            if msg.state == 4:
                duration_since_last_received_join_at_rear = (t - time_last_received_join_at_rear).to_sec()
                has_leader_state = True
                break
    
    # Successful if both flags are true
    is_successful = False
    if has_leader_waiting_state and has_leader_state:
        print("B-10 successful; Front vehicle transitioned from LEADERWAITING to LEADER (" + str(duration_since_last_received_join_at_rear) + " sec after 'JOIN_PLATOON_AT_REAR' msg)")
        is_successful = True
    elif has_leader_waiting_state:
        print("B-10 failed; Front vehicle never transitioned to LEADER")
        is_successful = False
    else:
        print("B-10 failed; Front vehicle never transitioned to LEADERWAITING")
        is_successful = False
    
    return is_successful

###########################################################################################################
# BASIC TRAVEL B-11: During platooning operations, the time gap between the front vehicle and the rear 
#                    vehicle shall always be +/- 0.5 seconds of the desired 2.5 second time gap.
###########################################################################################################
def check_distance_gap_during_platooning(bag, time_received_second_acceptance, time_end_engagement):
    # Parameters used for this metric
    threshold_time_gap = 0.5 # (seconds) Threshold offset from desired time gap; if the actual time gap is within this threshold, it is considered successful
    desired_time_gap = 2.5 # (seconds) The absolute desired time gap between the Follower vehicle and the Leader vehicle during platooning operations
    min_time_gap = desired_time_gap - threshold_time_gap # The minimum allowable time gap between the Follower vehicle and the Leader vehicle during platooning operations 
    max_time_gap = desired_time_gap + threshold_time_gap # The maximum allowable time gap between the Follower vehicle and the Leader vehicle during platooning operations

    # Variables that are tracked for this metric
    count_follower_msgs = 0
    count_negative_gap_msgs = 0
    lowest_time_gap = 100.0
    largest_time_gap = -100.0
    count_below_min_time_gap_msgs = 0
    count_above_max_time_gap_msgs = 0
    count_successful_time_gap_msgs = 0
    prev_negative_t = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_received_second_acceptance, end_time = time_end_engagement):
        # Only consider messages when the vehicle's platooning state is 'FOLLOWER'
        if msg.state == 5:
            if msg.actual_gap < 0:
                count_negative_gap_msgs += 1
                duration_since_prev_neg = (t - prev_negative_t).to_sec()
                if duration_since_prev_neg >= 0.5:
                    print("Time since previous negative actual gap: " + str(duration_since_prev_neg) + " seconds")
                prev_negative_t = t
                print("Received negative actual gap at time " + str(t.to_sec()))
                if count_negative_gap_msgs == 20:
                    break
            else:
                actual_speed = msg.desired_gap / desired_time_gap
                actual_time_gap = msg.actual_gap / actual_speed
                if (actual_time_gap < min_time_gap):
                    #print("Found time headway below 2.0")
                    count_below_min_time_gap_msgs += 1
                    if actual_time_gap < lowest_time_gap:
                        lowest_time_gap = actual_time_gap
                elif (actual_time_gap > max_time_gap):
                    #print("Found time headway above 3.0 " + str((t - time_received_second_acceptance).to_sec()) + " sec after negotiation")
                    count_above_max_time_gap_msgs += 1
                    if actual_time_gap > largest_time_gap:
                        largest_time_gap = actual_time_gap
                else:
                    count_successful_time_gap_msgs += 1
            
            count_follower_msgs += 1

    pct_negative_gap_msgs = (float(count_negative_gap_msgs) / float(count_follower_msgs)) * 100.0
    pct_below_min_time_gap_msgs = (float(count_below_min_time_gap_msgs) / float(count_follower_msgs)) * 100.0
    pct_above_max_time_gap_msgs = (float(count_above_max_time_gap_msgs) / float(count_follower_msgs)) * 100.0
    pct_successful_time_gap_msgs = (float(count_successful_time_gap_msgs) / float(count_follower_msgs)) * 100.0

    is_successful = False
    if pct_successful_time_gap_msgs == 1:
        print("B-11 succeeded (100% Successful): Statistics below:")
        is_successful = True
    else:
        print("B-11 failed (Not 100% Successful); Statistics below:")
        is_successful = False

    print(str(count_follower_msgs) + " Follower Messages; " + str(count_negative_gap_msgs) + " Negative (" + str(round(pct_negative_gap_msgs,2)) + "%); " \
          + str(count_below_min_time_gap_msgs) + " Below (" + str(round(pct_below_min_time_gap_msgs,2)) + "%); " + str(count_above_max_time_gap_msgs) + " Above (" \
          + str(round(pct_above_max_time_gap_msgs,2)) + "%); " + str(count_successful_time_gap_msgs) + " Successful (" + str(round(pct_successful_time_gap_msgs,2)) + "%)")
    if count_above_max_time_gap_msgs > 0:
        print("B-11 Stat: Largest time gap: " + str(largest_time_gap))
    if count_below_min_time_gap_msgs > 0:
        print("B-11 Stat: Smallest time gap: " + str(lowest_time_gap))

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-12: (FOLLOWER VEHICLE) During platooning operations, the front 'Leader' vehicle shall
#                    continuously broadcast 'STATUS' platooning MobilityOperation messages to the rear 'Follower'
#                    vehicle with a time gap between sequentially broadcasted messages below 0.2 seconds
#                    at least 90% of the time.
###########################################################################################################
def check_percentage_successful_status_msg_follower(bag, time_received_second_acceptance):
    # Parameters used for the computation of this metric
    min_percent_time_successful = 90.0 # (90%); Percent of active platooning time that the minimum 'STATUS' frequency must be achieved
    max_duration_between_status_msgs = 0.20 # (Seconds); Minimum duration between sequentially broadcasted 'STATUS' messages


    # Obtain the timestamp of the last received MobilityOperation message from the leader. This is considered the end of
    #        platooning since the leader has disengaged.
    time_last_received_mob_op = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_operation'], start_time = time_received_second_acceptance):
        time_last_received_mob_op = t

    # Obtain the quantity of 'STATUS' MobilityOperation messages received during platooning operations
    is_first_status_msg = True
    time_sent_prev_status_msg = rospy.Time()
    duration_since_prev_status_msg = 0.0
    total_time_between_unsuccessful_status_msgs = 0.0
    total_time_between_successful_status_msgs = 0.0
    total_time_spent_platooning = 0.0
    largest_duration_between_status_msgs = 0.0
    count_status_msgs = 0
    count_status_msgs_above_max_duration = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_received_second_acceptance, end_time = time_last_received_mob_op):
        if "STATUS" in msg.strategy_params:
            if is_first_status_msg:
                time_sent_prev_status_msg = t
                is_first_status_msg = False
                continue
            else:
                duration_since_prev_status_msg = (t - time_sent_prev_status_msg).to_sec()

                # Track the number of messages that are broadcasted after the (1/min_frequency) duration just for debugging purposes
                if (duration_since_prev_status_msg >= max_duration_between_status_msgs):
                    #print("Duration of " + str(duration_since_prev_status_msg) + " at " + str(t.to_sec()) + " sec")
                    total_time_between_unsuccessful_status_msgs += duration_since_prev_status_msg
                    count_status_msgs_above_max_duration += 1
                    if duration_since_prev_status_msg > largest_duration_between_status_msgs:
                        largest_duration_between_status_msgs = duration_since_prev_status_msg
                else:
                    total_time_between_successful_status_msgs += duration_since_prev_status_msg
                total_time_spent_platooning += duration_since_prev_status_msg
                time_sent_prev_status_msg = t
                count_status_msgs += 1
    
    percent_time_successful = (total_time_between_successful_status_msgs / total_time_spent_platooning) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_successful >= min_percent_time_successful:
        print("B-12 Succeeded; Time between messages was below " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")
        is_successful = True
    else:
        print("B-12 Failed; Time between messages was below " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_status_msgs_above_max_duration) / float(count_status_msgs)) * 100.0
    print(str(count_status_msgs) + " Total STATUS Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_status_msgs_above_max_duration) + " after " + str(round(max_duration_between_status_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between STATUS messages was " + str(largest_duration_between_status_msgs) + " sec")

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-12: (LEADER VEHICLE) During platooning operations, the front 'Leader' vehicle shall
#                    continuously broadcast 'STATUS' platooning MobilityOperation messages to the rear 'Follower'
#                    vehicle with a time gap between sequentially broadcasted messages below 0.2 seconds
#                    at least 90% of the time.
###########################################################################################################
def check_percentage_successful_status_msg_leader(bag, time_sent_second_acceptance):
    # Parameters used for the computation of this metric
    min_percent_time_successful = 90.0 # (90%); Percent of active platooning time that the minimum 'STATUS' frequency must be achieved
    max_duration_between_status_msgs = 0.20 # (Seconds); Minimum duration between sequentially broadcasted 'STATUS' messages

    # Obtain the timestamp of the last broadcasted MobilityOperation message from the leader. This is considered the end of
    #        platooning since the leader has disengaged.
    time_last_sent_mob_op = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance):
        time_last_sent_mob_op = t
    print("Leader sent last MobilityOperation at " + str(time_last_sent_mob_op.to_sec()))

    # Obtain the quantity of 'STATUS' MobilityOperation messages broadcasted during platooning operations
    is_first_status_msg = True
    time_sent_prev_status_msg = rospy.Time()
    duration_since_prev_status_msg = 0.0
    total_time_between_unsuccessful_status_msgs = 0.0
    total_time_between_successful_status_msgs = 0.0
    total_time_spent_platooning = 0.0
    largest_duration_between_status_msgs = 0.0
    count_status_msgs = 0
    count_status_msgs_above_max_duration = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance, end_time = time_last_sent_mob_op):
        if "STATUS" in msg.strategy_params:
            if is_first_status_msg:
                time_sent_prev_status_msg = t
                is_first_status_msg = False
                continue
            else:
                duration_since_prev_status_msg = (t - time_sent_prev_status_msg).to_sec()

                # Track the number of messages that are broadcasted after the (1/min_frequency) duration just for debugging purposes
                if (duration_since_prev_status_msg >= max_duration_between_status_msgs):
                    #print("Duration of " + str(duration_since_prev_status_msg) + " at " + str(t.to_sec()) + " sec")
                    total_time_between_unsuccessful_status_msgs += duration_since_prev_status_msg
                    count_status_msgs_above_max_duration += 1
                    if duration_since_prev_status_msg > largest_duration_between_status_msgs:
                        largest_duration_between_status_msgs = duration_since_prev_status_msg
                else:
                    total_time_between_successful_status_msgs += duration_since_prev_status_msg
                total_time_spent_platooning += duration_since_prev_status_msg
                time_sent_prev_status_msg = t
                count_status_msgs += 1
    
    percent_time_successful = (total_time_between_successful_status_msgs / total_time_spent_platooning) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_successful >= min_percent_time_successful:
        print("B-12 Succeeded; Time between messages was below " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")
        is_successful = True
    else:
        print("B-12 Failed; Time between messages was below " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_status_msgs_above_max_duration) / float(count_status_msgs)) * 100.0
    print(str(count_status_msgs) + " Total STATUS Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_status_msgs_above_max_duration) + " after " + str(round(max_duration_between_status_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between STATUS messages was " + str(largest_duration_between_status_msgs) + " sec")

    return is_successful

###########################################################################################################
# Basic Travel B-13: The executed trajectory to prepare for the geofence will include a deceleration section and 
#           the average deceleration amount shall be no less than 1 m/s^2.
###########################################################################################################
def check_deceleration_before_geofence(bag, time_enter_geofence, original_speed_limit, advisory_speed_limit):  
    # Parameters used for metric evaluation
    min_average_deceleration = 1.0 # m/s^2  
    start_decel_percent_of_original_speed_limit = 0.90 # Percentage (0.90 is 90%) of original speed limit for current speed to be considered the start of deceleration
    end_decel_percent_of_advisory_speed_limit = 1.10 # Percentage (1.10 is 110%) of advisory speed limit for current speed to be considered the end of deceleration

    # Variables to track during metric evaluation
    speed_start_decel = 0.0
    speed_end_decel = 0.0
    time_start_decel = rospy.Time()
    time_end_decel = rospy.Time()
    has_found_start_of_decel = False
    has_found_end_of_decel = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_enter_geofence):
        if has_found_start_of_decel:
            decel = (msg.twist.linear.x - prev_speed) / (t-prev_time).to_sec()
            #print("Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x) + "; " + str(decel) + " m/s^2")
            prev_speed = msg.twist.linear.x
            prev_time = t  

        # Get start time of deceleration
        if not has_found_start_of_decel and (msg.twist.linear.x < (start_decel_percent_of_original_speed_limit * original_speed_limit)):
            print("Start Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x))
            time_start_decel = t
            speed_start_decel = msg.twist.linear.x
            has_found_start_of_decel = True
            prev_speed = msg.twist.linear.x
            prev_time = t

        # Get end time of deceleration 
        if has_found_start_of_decel and (msg.twist.linear.x < (end_decel_percent_of_advisory_speed_limit * advisory_speed_limit)):
            print("End Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x))
            time_end_decel = t
            speed_end_decel = msg.twist.linear.x
            has_found_end_of_decel = True
            break
    
    deceleration_completed_before_geofence = False
    if (time_enter_geofence-time_end_decel).to_sec() > 0:
        deceleration_completed_before_geofence = True

    is_successful = False
    if has_found_start_of_decel and has_found_end_of_decel:

        # Calculate the average deceleration rate during the deceleration phase
        average_deceleration = (speed_start_decel - speed_end_decel) / (time_end_decel - time_start_decel).to_sec()

        if average_deceleration >= min_average_deceleration and deceleration_completed_before_geofence:
            print("B-13 succeeded; average deceleration above 1.0 m/s^2 occurred before geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = True
        elif average_deceleration >= min_average_deceleration and not deceleration_completed_before_geofence:
            print("B-13 failed; average deceleration above 1.0 m/s^2 occurred after geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
        elif average_deceleration <= min_average_deceleration and deceleration_completed_before_geofence:
            print("B-13 failed; a deceleration below 1.0 m/s^2 occurred before the geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
        elif average_deceleration <= min_average_deceleration and not deceleration_completed_before_geofence:
            print("B-13 failed; a deceleration below 1.0 m/s^2 occurred after the geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
    elif has_found_start_of_decel:
        print("B-13 failed; did not find end of deceleration phase")
        is_successful = False
    else:
        print("B-13 failed; did not find start of deceleration phase")
        is_successful = False

    return is_successful

###########################################################################################################
# Basic Travel B-14: The executed trajectory will start calling for acceleration back to the original speed limit 
#           no more than 30 feet away from the end of the geo-fenced area.
###########################################################################################################
def check_acceleration_distance_after_geofence(bag, time_exit_geofence):
    max_distance_from_geofence_end = 30.0 # (feet) Max distance from end of geofence for first lane change point
    min_distance_from_geofence_end = 0.0 # (feet) Minimum distance from end of geofence for first lane change point
    required_sequential_speed_increases = 5 # The number of sequential speed increases required to be considered the start of acceleration
    conversion_meters_to_feet = 3.28084 # 1 meter is 3.28084 feet

    # Get the downtrack at the end of the geofence
    downtrack_exit_geofence = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_exit_geofence):
        downtrack_exit_geofence = msg.down_track
        break

    # Variables to track during metric evaluation
    has_found_start_of_accel = False
    prev_speed = 0.0
    prev_t = rospy.Time()
    has_found_start_of_accel = False
    time_start_of_accel = rospy.Time()
    is_first_speed = True
    count_speed_increases = 0

    # Get the timestamp associated with the start of vehicle acceleration after exiting the geofence
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_exit_geofence):
        if is_first_speed:
            prev_speed = msg.twist.linear.x
            prev_t = t
            is_first_speed = False
            continue

        # Get start time of acceleration
        current_speed = msg.twist.linear.x
        duration_since_previous_speed = (t - prev_t).to_sec()
        current_accel = (current_speed - prev_speed) / duration_since_previous_speed
        #print("Current acceleration: " + str(current_accel) + " m/s^2; speed is " + str(current_speed) + " m/s")

        # Check if this is the start of acceleration
        if count_speed_increases == 0:
            time_start_of_accel = t # Always track the start time of consecutive speed increases

        if current_speed > prev_speed:
            count_speed_increases += 1
        else:
            count_speed_increases = 0

        # End loop if reached the required number of consecutive speed increases
        if count_speed_increases == required_sequential_speed_increases:
            has_found_start_of_accel = True
            break

        # Update variables
        prev_t = t
        prev_speed = current_speed

    # Get the distance after the geofence that the vehicle begins accelerating if any acceleration was found
    if has_found_start_of_accel:
        # Get the downtrack at the start of acceleration back to the original speed limit
        downtrack_start_of_accel = 0.0
        for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_of_accel):
            downtrack_start_of_accel = msg.down_track
            break

        # Obtain the distance (feet) after the geofence that the vehicle begins accelerating
        distance_start_accel_after_geofence = (downtrack_start_of_accel - downtrack_exit_geofence) * conversion_meters_to_feet
    
    # Set is_successful flag and print debug statements
    is_successful = False
    if has_found_start_of_accel:
        if distance_start_accel_after_geofence >= min_distance_from_geofence_end and distance_start_accel_after_geofence <= max_distance_from_geofence_end:
            print("B-14 succeeded; vehicle acceleration began " + str(distance_start_accel_after_geofence) + " feet after exiting the geofence.")
            is_successful = True
        else:
            print("B-14 failed; vehicle acceleration began " + str(distance_start_accel_after_geofence) + " feet after exiting the geofence.")
            is_successful = False
    else:
        print("B-14 failed; no vehicle acceleration occurred after exiting the geofence.")
        is_successful = False

    return is_successful

###########################################################################################################
# Basic Travel B-15: The executed trajectory back to normal operations will include an acceleration portion and 
#           the average acceleration over the entire acceleration time shall be no less than 1 m/s^2.
###########################################################################################################
def check_acceleration_rate_after_geofence(bag, time_exit_geofence, original_speed_limit, advisory_speed_limit):
    # Parameters used for metric evaluation
    min_average_acceleration = 1.0 # m/s^2  
    end_accel_percent_of_original_speed_limit = 0.90
    start_accel_percent_of_advisory_speed_limit = 1.10

    # Variables to track during metric evaluation
    speed_start_accel = 0.0
    speed_end_accel = 0.0
    time_start_accel = rospy.Time()
    time_end_accel = rospy.Time()
    has_found_start_of_accel = False
    has_found_end_of_accel = False

    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_exit_geofence):
        if has_found_start_of_accel:
            accel = (msg.twist.linear.x - prev_speed) / (t-prev_time).to_sec()
            #print("Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x) + "; " + str(accel) + " m/s^2")
            prev_speed = msg.twist.linear.x
            prev_time = t
        
        # Get start time of deceleration
        if not has_found_start_of_accel and (msg.twist.linear.x > (start_accel_percent_of_advisory_speed_limit * advisory_speed_limit)):
            print("Accel Start Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x))
            time_start_accel = t
            speed_start_accel = msg.twist.linear.x
            has_found_start_of_accel = True
            prev_speed = msg.twist.linear.x
            prev_time = t

        # Get end time of deceleration 
        if has_found_start_of_accel and (msg.twist.linear.x > (end_accel_percent_of_original_speed_limit * original_speed_limit)):
            print("Accel End Time: " + str(t.to_sec()) + "; " + str(msg.twist.linear.x))
            time_end_accel = t
            speed_end_accel = msg.twist.linear.x
            has_found_end_of_accel = True
            break

    # If the full acceleration phase has been found, determine the average acceleration for this phase
    if has_found_start_of_accel and has_found_end_of_accel:
        average_acceleration = (speed_end_accel - speed_start_accel) / (time_end_accel - time_start_accel).to_sec()
        print("Avg acceleration: " + str(average_acceleration))

    # Print success/failure statement and return success flag
    is_successful = False
    if (has_found_start_of_accel and has_found_end_of_accel):
        if average_acceleration >= min_average_acceleration:
            print("B-15 succeeded; average acceleration after geofence was above 1.0 m/s^2: " + str(average_acceleration) + " m/s^2")
            is_successful = True
        else:
            print("B-15 failed; average acceleration after geofence was below 1.0 m/s^2: " + str(average_acceleration) + " m/s^2")
            is_successful = False
    elif has_found_start_of_accel:
        print("B-15 failed; no end of acceleration after exiting the geofence was found.")
        is_successful = True
    else:
        print("B-15 failed; no acceleration after exiting the geofence was found.")
        is_successful = False

    return is_successful

###########################################################################################################
# Basic Travel B-16: The planned route must end with the CP vehicle having been at steady state, after all other 
#           maneuvers, for at least 10 seconds.
###########################################################################################################
def check_steady_state_after_geofence(bag, time_exit_geofence, time_end_engagement, original_speed_limit):
    # Parameters used for metric evaluation
    # (m/s) Threshold offset from speed limit; vehicle considered at steady state when its speed is within this offset of the speed limit
    threshold_speed_limit_offset = 0.894 # 0.894 m/s is 2 mph
    # (m/s) Minimum speed to be considered at steady state
    min_steady_state_speed = original_speed_limit - threshold_speed_limit_offset
    # (m/s) Maximum speed to be considered at steady state
    max_steady_state_speed = original_speed_limit + threshold_speed_limit_offset
    # (seconds) Minimum required threshold at steady state after completing all geofence-triggered maneuvers
    min_steady_state_time = 10.0

    # Conduct steady state evaluation 
    # Get the start time of the vehicle reaching steady state (if one exists)
    has_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_exit_geofence, end_time = time_end_engagement):
        # Vehicle has reached steady state when its speed within threshold range of steady state speed
        if (max_steady_state_speed >= msg.twist.linear.x >= min_steady_state_speed):
            time_start_steady_state = t
            has_steady_state = True
            break
    
    # If the start time of steady state was found, find the longest duration of steady state time
    has_passed_steady_state_time_threshold = False
    max_steady_state_duration = 0.0
    if (has_steady_state):
        has_passed_steady_state_time_threshold = False
        is_at_steady_state = True
        steady_state_duration = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_steady_state, end_time = time_end_engagement):
            current_speed = msg.twist.linear.x

            # If system is entering steady state, reset the steady state start time:
            if ((max_steady_state_speed >= current_speed >= min_steady_state_speed) and not is_at_steady_state):
                is_at_steady_state = True
                time_start_steady_state = t

            # If system is maintaining steady state, check if it has passed the threshold time of continuous steady state:
            elif ((max_steady_state_speed >= current_speed >= min_steady_state_speed) and is_at_steady_state):
                steady_state_duration = (t - time_start_steady_state).to_sec()

                if steady_state_duration > max_steady_state_duration:
                    max_steady_state_duration = steady_state_duration

                if (steady_state_duration >= min_steady_state_time):
                    has_passed_steady_state_time_threshold = True
            
            # If system has exited steady state, reset the steady state flag
            elif  ((max_steady_state_speed <= current_speed or current_speed <= min_steady_state_speed) and is_at_steady_state):
                is_at_steady_state = False

    if (has_passed_steady_state_time_threshold):
        print("B-16 succeeded; system reached continuous steady state for " + str(max_steady_state_duration) + " seconds after geofence-triggered maneuvers.")
        is_successful = True
    else:
        if has_steady_state:
            print("B-16 failed; system reached continuous steady state for " + str(steady_state_duration) + " seconds after geofence-triggered maneuvers. " \
                + " At least " + str(min_steady_state_time) + " seconds required.")
            is_successful = False
        if not has_steady_state:
            print("B-16 failed; system did not reach steady state after geofence-triggered maneuvers.")
            is_successful = False

    return is_successful

###########################################################################################################
# Basic Travel B-17: The entire scenario will satisfy all previous criteria using any of the speeds given here for 
#           the "regular speed limit" (i.e. the speed limit when not in the geo-fence).
###########################################################################################################
def check_speed_limit_when_not_in_geofence(bag, time_start_engagement, time_enter_geofence, time_exit_geofence, time_end_engagement, original_speed_limit):
    # (m/s) Threshold offset from speed limit to account for floating point precision
    threshold_speed_limit_offset = 0.01
    max_speed_limit = original_speed_limit + threshold_speed_limit_offset # (m/s)
    min_speed_limit = original_speed_limit - threshold_speed_limit_offset # (m/s)

    # Tolerance for the time between the geofence start and end and finding a speed limit that belongs in the geofence:
    # Note: This tolerance is due to timing not being fully-synchronized between /environment/active_geofence and all other topics
    time_tolerance_geofence = 0.80 # seconds

    # Check speed limit before entering geofence
    is_correct_speed_limit_before_geofence = False
    time_incorrect_speed_limit_before_geofence = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_enter_geofence):
        # The first correct speed limit has been found
        if ((max_speed_limit >= msg.speed_limit >= min_speed_limit) and not is_correct_speed_limit_before_geofence):
            is_correct_speed_limit_before_geofence = True
        # An incorrect speed limit has been found; will trigger a failure if not within tolerance of the geofence entrance
        elif((msg.speed_limit >= max_speed_limit or msg.speed_limit <= min_speed_limit) and is_correct_speed_limit_before_geofence):
            time_incorrect_speed_limit_before_geofence = abs(time_enter_geofence - t).to_sec()
            
            # Not a failure if within tolerance time of the geofence entrance
            if (time_incorrect_speed_limit_before_geofence <= time_tolerance_geofence):
                is_correct_speed_limit_before_geofence = True
                continue

            # Failure if not within tolerance time of the geofence entrance
            print("Speed limit " + str(msg.speed_limit) + " m/s found at " + str(t.to_sec()) + \
                ", which is " + str(time_incorrect_speed_limit_before_geofence) + " seconds before entering the geofence.")
            is_correct_speed_limit_before_geofence = False
            break

    # Check speed limit after exiting geofence
    is_correct_speed_limit_after_geofence = False
    time_incorrect_speed_limit_after_geofence = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_exit_geofence, end_time = time_end_engagement):
        # Speed limit is correct for the first time after exiting the geofence
        if ((max_speed_limit >= msg.speed_limit >= min_speed_limit) and not is_correct_speed_limit_after_geofence):
            time_incorrect_speed_limit_after_geofence = abs(t - time_exit_geofence).to_sec()
            if (time_incorrect_speed_limit_after_geofence <= time_tolerance_geofence):
                is_correct_speed_limit_after_geofence = True
                continue
        
        # Failure; speed limit is incorrect and not within tolerance time of exiting geofence:
        elif((msg.speed_limit >= max_speed_limit or msg.speed_limit <= min_speed_limit) and not is_correct_speed_limit_after_geofence):
            time_incorrect_speed_limit_after_geofence = abs(t - time_exit_geofence).to_sec()
            print("Speed limit " + str(msg.speed_limit) + " m/s found at " + str(t.to_sec()) + \
                    ", which is " + str(time_incorrect_speed_limit_after_geofence) + " seconds after exiting the geofence.")
            is_correct_speed_limit_after_geofence = False
            break

        # Failure; speed limit is incorrect after having been correct before
        elif((msg.speed_limit >= max_speed_limit or msg.speed_limit <= min_speed_limit) and is_correct_speed_limit_after_geofence):
            is_correct_speed_limit_after_geofence = False
            break

    # Print success/failure statement and return success flag
    if (not is_correct_speed_limit_before_geofence and not is_correct_speed_limit_after_geofence):
        print("B-17 failed; speed limit was not always " + str(original_speed_limit) + " m/s before OR after the geofence.")
        is_successful = False
    elif(not is_correct_speed_limit_before_geofence):
        print("B-17 failed; speed limit was not always " + str(original_speed_limit) + " m/s before the geofence.")
        is_successful = False
    elif(not is_correct_speed_limit_after_geofence):
        print("B-17 failed; speed limit was not always " + str(original_speed_limit) + " m/s after the geofence.")
        is_successful = False
    else:
        print("B-17 succeeded; speed limit was always " + str(original_speed_limit) + " m/s before AND after the geofence.")
        is_successful = True

    return is_successful

###########################################################################################################
# Basic Travel B-18: The advisory speed limit communicated by CARMA Cloud shall be 10 mph less than the 
#                    regular speed limit.
###########################################################################################################
def check_advisory_speed_limit(bag, advisory_speed_limit, original_speed_limit):
    # (m/s) Required offset from normal speed limit required for the advisory speed limit
    speed_limit_offset = 4.4704 # 4.4704 m/s is 10 mph
    # (m/s) Threshold offset from expected advisory speed limit to account for floating point precision
    threshold_speed_limit_offset = 0.01
    max_advisory_speed_limit = (original_speed_limit - speed_limit_offset) + threshold_speed_limit_offset
    min_advisory_speed_limit = (original_speed_limit - speed_limit_offset) - threshold_speed_limit_offset

    # Evaluate speed limits, print success/failure statement, and return success flag
    advisory_speed_limit_mph = int(advisory_speed_limit * 2.23694) # Conversion of m/s to mph
    original_speed_limit_mph = int(original_speed_limit * 2.23694) # Conversion of m/s to mph
    speed_limit_offset_mph = int(speed_limit_offset * 2.23694) # Conversion of m/s to mph
    if (max_advisory_speed_limit >= advisory_speed_limit >= min_advisory_speed_limit):
        print("B-18 succeeded; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")
        is_successful = True
    else:
        print("B-18 failed; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is not " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")       
        is_successful = False
    
    return is_successful

###########################################################################################################
# Basic Travel B-20 (Front Vehicle) and B-21 (Rear Vehicle): The vehicle is fully contained within its lane 
#                    (it does not breach either of the lane lines) for at least 95% of the time spent with
#                     the CARMA system engaged.
###########################################################################################################
def check_vehicle_inside_lane(bag, time_start_engagement, time_end_engagement, is_front_vehicle):
    # Parameters used for this metric evaulation
    vehicle_half_width = 1.01 # (meters) for Chrysler Pacifica Minivan; 0.955 Ford Fusion; 0.947 Lexus
    percent_required_inside_lane = 95.0 # (percentage; 0.0 to 100.0)
    lane_width = 3.7 # (meters)
    max_allowed_cross_track = (lane_width / 2.0) - vehicle_half_width
    time_begin_evaluation_after_start_engagement = 5.0 # (seconds) Ensures we don't capture crosstrack error from driver manually positioning vehicle in poor position at start of test
    time_start = rospy.Time(time_start_engagement.to_sec() + time_begin_evaluation_after_start_engagement)

    # Variables to track during metric evaluation
    max_found_cross_track = 0.0
    time_outside_lane = 0.0
    time_inside_lane = 0.0
    prev_t = rospy.Time()
    first = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start, end_time = time_end_engagement):
        # For first message, just save its timestamp within the 'prev_t' object
        if first:
            prev_t = t
            first = False
            continue
        
        # If cross track indicates vehicle is outside its lane, update total amount of time spent outside of lane
        if msg.cross_track > max_allowed_cross_track:
            time_outside_lane += (t - prev_t).to_sec()

            # Debug Statement
            #print("Time " + str(t.to_sec()) + ": Outside Lane CTE " + str(msg.cross_track) + " meters")
        # If cross track indicates vehicle is inside its lane, update total amount of time spent inside of lane
        else:
            time_inside_lane += (t - prev_t).to_sec()

            # Debug Statement
            #print("Time " + str(t.to_sec()) + ": Inside Lane CTE " + str(msg.cross_track) + " meters")

        if msg.cross_track > max_found_cross_track:
            max_found_cross_track = msg.cross_track
        
        prev_t = t
    
    percent_inside_lane = (time_inside_lane / (time_inside_lane + time_outside_lane)) * 100.0
    max_dist_outside_lane = (max_found_cross_track + vehicle_half_width) - (lane_width / 2.0) 

    is_successful = False
    if percent_inside_lane >= percent_required_inside_lane:
        if is_front_vehicle:
            print("B-20 succeeded; vehicle was fully inside its lane for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (>= 95%).")
        else:
            print("B-21 succeeded; vehicle was fully inside its lane for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (>= 95%).")
        print(str(round(time_inside_lane,4)) + " sec inside lane; " + str(round(time_outside_lane,4)) + " sec outside lane; max crosstrack: " + str(round(max_found_cross_track,4)) + "; max dist outside lane: " + str(round(max_dist_outside_lane,4)) + " m")
        is_successful = True
    else:
        if is_front_vehicle:
            print("B-20 failed; vehicle was fully inside its lane for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (< 95%).")
        else:
            print("B-21 failed; vehicle was fully inside its lane for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (< 95%).")
        print(str(round(time_inside_lane,4)) + " sec inside lane; " + str(round(time_outside_lane,4)) + " sec outside lane; max crosstrack: " + str(round(max_found_cross_track,4)) + "; max dist outside lane: " + str(round(max_dist_outside_lane,4)) + " m")
        is_successful = True
    return is_successful

###########################################################################################################
# Basic Travel B-22 (Front Vehicle) and B-23 (Rear Vehicle): The vehicle never breaches the left or right 
#                   lane line associated with its current lane by more than 0.1 meters while the CARMA 
#                   system is engaged.
###########################################################################################################
def check_vehicle_always_within_maximum_crosstrack(bag, time_start_engagement, time_end_engagement, is_front_vehicle):
    # Parameters used for this metric evaulation
    vehicle_half_width = 1.01 # (meters) for Chrysler Pacifica Minivan; 0.955 Ford Fusion; 0.947 Lexus
    lane_width = 3.7 # (meters)
    max_distance_over_lane_line = 0.1 # (meters)
    max_allowed_cross_track = (lane_width / 2.0) - vehicle_half_width + max_distance_over_lane_line
    time_begin_evaluation_after_start_engagement = 5.0 # (seconds) Ensures we don't capture crosstrack error from driver manually positioning vehicle in poor position at start of test
    time_start = rospy.Time(time_start_engagement.to_sec() + time_begin_evaluation_after_start_engagement)

    # Variables to track during metric evaluation
    breached_max_cross_track = False
    max_found_cross_track = 0.0
    time_outside_allowable_cte = 0.0
    time_inside_allowable_cte = 0.0
    prev_t = rospy.Time()
    first = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start, end_time = time_end_engagement):        
        # For first message, save its timestamp within the 'prev_t' object
        if first:
            prev_t = t
            first = False
            continue
        
        # If cross track indicates vehicle is outside its lane by more than allowable distance, update total amount of time spent outside of lane
        if msg.cross_track > max_allowed_cross_track:
            time_outside_allowable_cte += (t - prev_t).to_sec()
            breached_max_cross_track = True

            # Debug Statement
            #print("Time " + str(t.to_sec()) + ": Outside Lane CTE " + str(msg.cross_track) + " meters")
        # If cross track indicates vehicle is note outside its lane by more than allowable distance, update total amount of time spent inside of lane
        else:
            time_inside_allowable_cte += (t - prev_t).to_sec()

            # Debug Statement
            #print("Time " + str(t.to_sec()) + ": Inside Lane CTE " + str(msg.cross_track) + " meters")

        if msg.cross_track > max_found_cross_track:
            max_found_cross_track = msg.cross_track
        
        prev_t = t
    
    percent_inside_lane = (time_inside_allowable_cte / (time_inside_allowable_cte + time_outside_allowable_cte)) * 100.0
    max_dist_outside_lane = (max_found_cross_track + vehicle_half_width) - (lane_width / 2.0) 

    is_successful = False
    if not breached_max_cross_track:
        if is_front_vehicle:
            print("B-22 succeeded; vehicle never breached lane lines by more than 0.1 meters.")
        else:
            print("B-23 succeeded; vehicle never breached lane lines by more than 0.1 meters.")
        print(str(round(time_inside_allowable_cte,4)) + " sec not over line by 0.1 m; " + str(round(time_outside_allowable_cte,4)) + " sec over line by 0.1 m; max crosstrack: " + str(round(max_found_cross_track,4)) + "; max dist outside lane: " + str(round(max_dist_outside_lane,4)) + " m")
        is_successful = True
    else:
        if is_front_vehicle:
            print("B-22 failed; vehicle breached lane lines by more than 0.1 meters for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (< 95%).")
        else:
            print("B-23 failed; vehicle breached lane lines by more than 0.1 meters for " + str(round(percent_inside_lane,4)) + "% of time spent engaged (< 95%).")
        print(str(round(time_inside_allowable_cte,4)) + " sec not over line by 0.1 m; " + str(round(time_outside_allowable_cte,4)) + " sec ove rline by 0.1 m; max crosstrack: " + str(round(max_found_cross_track,4)) + "; max dist outside lane: " + str(round(max_dist_outside_lane,4)) + " m")
        is_successful = True
    return is_successful

###########################################################################################################
# BASIC TRAVEL B-24: (Leader VEHICLE) During platooning operations, the front 'Leader' vehicle shall 
#                    continuously broadcast 'INFO' platooning MobilityOperation messages with a time gap 
#                    between sequentially broadcasted messages below 0.5 seconds at least 90% of the time.
###########################################################################################################
def check_percentage_successful_info_msg(bag, time_sent_second_acceptance):
    # Parameters used for the computation of this metric
    min_percent_time_successful = 90.0 # (90%); Percent of active platooning time that the minimum 'INFO' frequency must be achieved
    max_duration_between_info_msgs = 0.50 # (Seconds); Maximum duration between sequentially broadcasted 'INFO' messages

    # Obtain the timestamp of the last broadcasted MobilityOperation message from the leader. This is considered the end of
    #        platooning since the leader has disengaged.
    time_last_sent_mob_op = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance):
        time_last_sent_mob_op = t
    print("Leader sent last MobilityOperation at " + str(time_last_sent_mob_op.to_sec()))

    # Obtain the quantity of 'INFO' MobilityOperation messages broadcasted during platooning operations
    is_first_info_msg = True
    time_sent_prev_info_msg = rospy.Time()
    duration_since_prev_info_msg = 0.0
    total_time_between_unsuccessful_info_msgs = 0.0
    total_time_between_successful_info_msgs = 0.0
    total_time_spent_platooning = 0.0
    largest_duration_between_info_msgs = 0.0
    count_info_msgs = 0
    count_info_msgs_above_max_duration = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance, end_time = time_last_sent_mob_op):
        if "INFO" in msg.strategy_params:
            if is_first_info_msg:
                time_sent_prev_info_msg = t
                is_first_info_msg = False
                continue
            else:
                duration_since_prev_info_msg = (t - time_sent_prev_info_msg).to_sec()

                # Track the number of messages that are broadcasted after the (1/min_frequency) duration just for debugging purposes
                if (duration_since_prev_info_msg >= max_duration_between_info_msgs):
                    #print("Duration of " + str(duration_since_prev_info_msg) + " at " + str(t.to_sec()) + " sec")
                    total_time_between_unsuccessful_info_msgs += duration_since_prev_info_msg
                    count_info_msgs_above_max_duration += 1
                    if duration_since_prev_info_msg > largest_duration_between_info_msgs:
                        largest_duration_between_info_msgs = duration_since_prev_info_msg
                else:
                    total_time_between_successful_info_msgs += duration_since_prev_info_msg
                total_time_spent_platooning += duration_since_prev_info_msg
                time_sent_prev_info_msg = t
                count_info_msgs += 1
    
    percent_time_successful = (total_time_between_successful_info_msgs / total_time_spent_platooning) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_successful >= min_percent_time_successful:
        print("B-24 Succeeded; Time between messages was below " + str(round(max_duration_between_info_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")
        is_successful = True
    else:
        print("B-24 Failed; Time between messages was below " + str(round(max_duration_between_info_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_info_msgs_above_max_duration) / float(count_info_msgs)) * 100.0
    print(str(count_info_msgs) + " Total INFO Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_info_msgs_above_max_duration) + " after " + str(round(max_duration_between_info_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between INFO messages was " + str(largest_duration_between_info_msgs) + " sec")

###########################################################################################################
# BASIC TRAVEL B-25: After the rear vehicle responds to the front vehicle with its 'Platoon Follower Join' 
#                    MobilityRequest message (the message signaling that it is joining the platoon), it shall 
#                    take no more than 10 seconds for the rear vehicle to achieve a time headway with the front 
#                    vehicle that is within 0.5 seconds of the desired 2.5 second time headway.
###########################################################################################################
def check_duration_before_successful_time_headway(bag, time_received_second_acceptance, time_enter_geofence):
    threshold_time_gap = 0.5 # (seconds) Threshold offset from desired time gap; if the actual time gap is within this threshold, it is considered successful
    desired_time_gap = 2.5 # (seconds) The absolute desired time gap between the Follower vehicle and the Leader vehicle during platooning operations
    min_time_gap = desired_time_gap - threshold_time_gap # The minimum allowable time gap between the Follower vehicle and the Leader vehicle during platooning operations 
    max_time_gap = desired_time_gap + threshold_time_gap # The maximum allowable time gap between the Follower vehicle and the Leader vehicle during platooning operations
    max_duration_before_successful_time_gap = 10.0 # (seconds)

    has_unsuccessful_time_gap = True
    time_first_steady_state_successful_time_gap = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/platooning_info'], start_time = time_received_second_acceptance, end_time = time_enter_geofence):
        # Only consider messages when the vehicle's platooning state is 'FOLLOWER'
        if msg.state == 5:
            # Obtain the current time headway
            actual_speed = msg.desired_gap / desired_time_gap
            actual_time_gap = msg.actual_gap / actual_speed

            # Set flag if the current time headway is outside the desired range
            if (actual_time_gap < min_time_gap or actual_time_gap > max_time_gap):
                has_unsuccessful_time_gap = True

            # Otherwise, get the time of the first time headway that is not outside the desired range
            # Note: This must be after the last time headway that is outside the desired range
            else:
                if has_unsuccessful_time_gap:
                    time_first_steady_state_successful_time_gap = t
                    has_unsuccessful_time_gap = False

    duration_before_successful_time_headway = (time_first_steady_state_successful_time_gap - time_received_second_acceptance).to_sec()

    is_successful = False
    if duration_before_successful_time_headway <= max_duration_before_successful_time_gap:
        print("B-25 Succeeded; achieved steady state time headway within desired range " + str(duration_before_successful_time_headway) + " sec after platoon negotiation (<= 10 sec).")
        is_successful = True
    else:
        print("B-25 Failed; achieved steady state time headway within desired range " + str(duration_before_successful_time_headway) + " sec after platoon negotiation. (> 10 sec)")
        is_successful = False

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-26: During platooning operations, the time gap between sequentially broadcasted 'STATUS' platooning
#                    MobilityOperation messages from the front 'Leader' vehicle to the rear 'Follower' vehicle
#                    shall never exceed 1 second.
###########################################################################################################
def check_max_time_between_status_msg(bag, time_sent_second_acceptance):
    # Parameters used for the computation of this metric
    max_percent_time_unsuccessful = 0.0 # (Percent)
    max_duration_between_status_msgs = 1.0 # (Seconds); Minimum duration between sequentially broadcasted 'STATUS' messages

    # Obtain the timestamp of the last broadcasted MobilityOperation message from the leader. This is considered the end of
    #        platooning since the leader has disengaged.
    time_last_sent_mob_op = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance):
        time_last_sent_mob_op = t
    print("Leader sent last MobilityOperation at " + str(time_last_sent_mob_op.to_sec()))

    # Obtain the quantity of 'STATUS' MobilityOperation messages broadcasted during platooning operations
    is_first_status_msg = True
    time_sent_prev_status_msg = rospy.Time()
    duration_since_prev_status_msg = 0.0
    total_time_between_unsuccessful_status_msgs = 0.0
    total_time_between_successful_status_msgs = 0.0
    total_time_spent_platooning = 0.0
    largest_duration_between_status_msgs = 0.0
    count_status_msgs = 0
    count_status_msgs_above_max_duration = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance, end_time = time_last_sent_mob_op):
        if "STATUS" in msg.strategy_params:
            if is_first_status_msg:
                time_sent_prev_status_msg = t
                is_first_status_msg = False
                continue
            else:
                duration_since_prev_status_msg = (t - time_sent_prev_status_msg).to_sec()

                # Track the number of messages that are broadcasted after the (1/min_frequency) duration just for debugging purposes
                if (duration_since_prev_status_msg >= max_duration_between_status_msgs):
                    #print("Duration of " + str(duration_since_prev_status_msg) + " at " + str(t.to_sec()) + " sec")
                    total_time_between_unsuccessful_status_msgs += duration_since_prev_status_msg
                    count_status_msgs_above_max_duration += 1
                    if duration_since_prev_status_msg > largest_duration_between_status_msgs:
                        largest_duration_between_status_msgs = duration_since_prev_status_msg
                else:
                    total_time_between_successful_status_msgs += duration_since_prev_status_msg
                total_time_spent_platooning += duration_since_prev_status_msg
                time_sent_prev_status_msg = t
                count_status_msgs += 1
    
    percent_time_unsuccessful = (total_time_between_unsuccessful_status_msgs / total_time_spent_platooning) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_unsuccessful <= max_percent_time_unsuccessful:
        print("B-26 Succeeded; Time between STATUS messages was above " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_unsuccessful,3)) + \
              "% of the time. (Must be lower than " + str(round(max_percent_time_unsuccessful,3)) + "%)")
        is_successful = True
    else:
        print("B-26 Failed; Time between STATUS messages was above " + str(round(max_duration_between_status_msgs,3)) + " seconds " + str(round(percent_time_unsuccessful,3)) + \
              "% of the time. (Must be lower than " + str(round(max_percent_time_unsuccessful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_status_msgs_above_max_duration) / float(count_status_msgs)) * 100.0
    print(str(count_status_msgs) + " Total STATUS Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_status_msgs_above_max_duration) + " after " + str(round(max_duration_between_status_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between STATUS messages was " + str(largest_duration_between_status_msgs) + " sec")

    return is_successful

###########################################################################################################
# BASIC TRAVEL B-27: During platooning operations, the time gap between sequentially broadcasted 'Follower' platooning
#                    MobilityOperation messages from the front 'Leader' vehicle to the rear 'Follower' vehicle
#                    shall never exceed 1 second.
###########################################################################################################
def check_max_time_between_info_msg(bag, time_sent_second_acceptance):
    # Parameters used for the computation of this metric
    max_percent_time_unsuccessful = 0.0 # (Percent)
    max_duration_between_info_msgs = 1.0 # (Seconds); Minimum duration between sequentially broadcasted 'INFO' messages

    # Obtain the timestamp of the last broadcasted MobilityOperation message from the leader. This is considered the end of
    #        platooning since the leader has disengaged.
    time_last_sent_mob_op = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance):
        time_last_sent_mob_op = t
    print("Leader sent last MobilityOperation at " + str(time_last_sent_mob_op.to_sec()))

    # Obtain the quantity of 'INFO' MobilityOperation messages broadcasted during platooning operations
    is_first_info_msg = True
    time_sent_prev_info_msg = rospy.Time()
    duration_since_prev_info_msg = 0.0
    total_time_between_unsuccessful_info_msgs = 0.0
    total_time_between_successful_info_msgs = 0.0
    total_time_spent_platooning = 0.0
    largest_duration_between_info_msgs = 0.0
    count_info_msgs = 0
    count_info_msgs_above_max_duration = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation'], start_time = time_sent_second_acceptance, end_time = time_last_sent_mob_op):
        if "INFO" in msg.strategy_params:
            if is_first_info_msg:
                time_sent_prev_info_msg = t
                is_first_info_msg = False
                continue
            else:
                duration_since_prev_info_msg = (t - time_sent_prev_info_msg).to_sec()

                # Track the number of messages that are broadcasted after the (1/min_frequency) duration just for debugging purposes
                if (duration_since_prev_info_msg >= max_duration_between_info_msgs):
                    #print("Duration of " + str(duration_since_prev_info_msg) + " at " + str(t.to_sec()) + " sec")
                    total_time_between_unsuccessful_info_msgs += duration_since_prev_info_msg
                    count_info_msgs_above_max_duration += 1
                    if duration_since_prev_info_msg > largest_duration_between_info_msgs:
                        largest_duration_between_info_msgs = duration_since_prev_info_msg
                else:
                    total_time_between_successful_info_msgs += duration_since_prev_info_msg
                total_time_spent_platooning += duration_since_prev_info_msg
                time_sent_prev_info_msg = t
                count_info_msgs += 1
    
    percent_time_unsuccessful = (total_time_between_unsuccessful_info_msgs / total_time_spent_platooning) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_unsuccessful <= max_percent_time_unsuccessful:
        print("B-27 Succeeded; Time between messages was above " + str(round(max_duration_between_info_msgs,3)) + " seconds " + str(round(percent_time_unsuccessful,3)) + \
              "% of the time. (Must be lower than " + str(round(max_percent_time_unsuccessful,3)) + "%)")
        is_successful = True
    else:
        print("B-27 Failed; Time between messages was above " + str(round(max_duration_between_info_msgs,3)) + " seconds " + str(round(percent_time_unsuccessful,3)) + \
              "% of the time. (Must be lower than " + str(round(max_percent_time_unsuccessful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_info_msgs_above_max_duration) / float(count_info_msgs)) * 100.0
    print(str(count_info_msgs) + " Total INFO Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_info_msgs_above_max_duration) + " after " + str(round(max_duration_between_info_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between INFO messages was " + str(largest_duration_between_info_msgs) + " sec")

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
    csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Vehicle Role",
                                 "BT-B-1 Result", "BT-B-2 Result", "BT-B-3 Result", "BT-B-4 Result", "BT-B-5 Result", "BT-B-6 Result", 
                                 "BT-B-7 Result", "BT-B-8 Result", "BT-B-9 Result", "BT-B-10 Result","BT-B-11 Result", "BT-B-12 Result", 
                                 "BT-B-13 Result", "BT-B-14 Result", "BT-B-15 Result", "BT-B-16 Result", "BT-B-17 Result", "BT-B-18 Result", 
                                 "BT-19 Result", "BT-20 Result", "BT-21 Result", "BT-22 Result", "BT-23 Result", "BT-24 Result", "BT-25 Result",
                                 "BT-26 Result", "BT-27 Result"])
    
    # Create list of Basic Travel White Pacifica (Leader) bag files to be processed
    BT_leader_white_pacifica_bag_files = ["_2021-07-21-21-14-34_down-selected.bag",
                                               "_2021-07-21-21-29-32_down-selected.bag",
                                               "_2021-07-21-21-38-21_down-selected.bag",
                                               "_2021-07-21-21-48-21_down-selected.bag",
                                               "_2021-07-22-13-22-00_down-selected.bag",
                                               "_2021-07-22-13-30-47_down-selected.bag",
                                               "_2021-07-22-13-41-35_down-selected.bag",
                                               "_2021-07-22-13-51-01_down-selected.bag",
                                               "_2021-07-22-14-01-37_down-selected.bag",
                                               "_2021-07-22-14-10-12_down-selected.bag",
                                               "_2021-07-22-14-21-18_down-selected.bag",
                                               "_2021-07-22-15-10-36_down-selected.bag",
                                               "_2021-07-22-15-20-10_down-selected.bag",
                                               "_2021-07-22-15-30-47_down-selected.bag",
                                               "_2021-07-22-15-41-31_down-selected.bag"]

    # Create list of Basic Travel Black Pacifica (Follower) bag files to be processed
    BT_follower_black_pacifica_bag_files = ["_2021-07-21-21-13-55_down-selected.bag",
                                                 "_2021-07-21-21-30-03_down-selected.bag",
                                                 "_2021-07-21-21-37-01_down-selected.bag",
                                                 "_2021-07-21-21-49-15_down-selected.bag",
                                                 "_2021-07-22-13-09-04_down-selected.bag",
                                                 "_2021-07-22-13-31-33_down-selected.bag",
                                                 "_2021-07-22-13-41-48_down-selected.bag",
                                                 "_2021-07-22-13-52-00_down-selected.bag",
                                                 "_2021-07-22-14-03-01_down-selected.bag",
                                                 "_2021-07-22-14-10-54_down-selected.bag",
                                                 "_2021-07-22-14-22-57_down-selected.bag",
                                                 "_2021-07-22-15-08-37_down-selected.bag",
                                                 "_2021-07-22-15-20-19_down-selected.bag",
                                                 "_2021-07-22-15-31-41_down-selected.bag",
                                                 "_2021-07-22-15-42-47_down-selected.bag"]

    # Concatenate all Basic Travel bag files into one list
    BT_bag_files = BT_leader_white_pacifica_bag_files + BT_follower_black_pacifica_bag_files

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in BT_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in BT_leader_white_pacifica_bag_files:
            print("Basic Travel Test Case - Leader")
        elif bag_file in BT_follower_black_pacifica_bag_files:
            print("Basic Travel Test Case - Follower")
        
        # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
        sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(BT_bag_files.index(bag_file) + 1) + " of " + str(len(BT_bag_files)) + ")")
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

        # Get the rosbag times associated with entering and exiting the active geofence
        print("Getting geofence times at " + str(datetime.datetime.now()))
        time_enter_geofence, time_exit_geofence, found_geofence_times = get_geofence_entrance_and_exit_times(bag)
        print("Got geofence times at " + str(datetime.datetime.now()))
        if (not found_geofence_times):
            print("Could not find geofence entrance and exit times in bag file.")
            continue

        # Get the rosbag times associated with the starting engagement and ending engagement for the Basic Travel use case test
        print("Getting engagement times at " + str(datetime.datetime.now()))
        time_test_start_engagement, time_test_end_engagement, found_test_times = get_test_case_engagement_times(bag, time_enter_geofence, time_exit_geofence)
        print("Got engagement times at " + str(datetime.datetime.now()))
        if (not found_test_times):
            print("Could not find test case engagement start and end times in bag file.")
            continue
        
        # Debug Statements
        print("Engagement starts at " + str(time_test_start_engagement.to_sec()))
        print("Entered Geofence at " + str(time_enter_geofence.to_sec()))
        print("Exited Geofence at " + str(time_exit_geofence.to_sec()))
        print("Engagement ends at " + str(time_test_end_engagement.to_sec()))
        print("Time spent in geofence: " + str((time_exit_geofence - time_enter_geofence).to_sec()) + " seconds")
        print("Time spent engaged: " + str((time_test_end_engagement - time_test_start_engagement).to_sec()) + " seconds")

        original_speed_limit = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit) + " m/s")

        # Update the exit geofence time to be based off of /guidance/route_state for improved accuracy
        time_exit_geofence = adjust_geofence_exit_time(bag, time_exit_geofence, original_speed_limit)
        print("Adjusted geofence exit time (based on /guidance/route_state): " + str(time_exit_geofence))

        print_lanelet_entrance_times(bag, time_test_start_engagement)

        downtrack_enter_geofence = get_geofence_entrance_downtrack(bag, time_enter_geofence)

        # Initialize results 
        b_1_result = None
        b_2_result = None
        b_3_result = None
        b_4_result = None
        b_5_result = None
        b_6_result = None
        b_7_result = None
        b_8_result = None
        b_9_result = None
        b_10_result = None
        b_11_result = None
        b_12_result = None
        b_13_result = None
        b_14_result = None
        b_15_result = None
        b_16_result = None
        b_17_result = None
        b_18_result = None
        b_19_result = None
        b_20_result = None
        b_21_result = None
        b_22_result = None
        b_23_result = None
        b_24_result = None
        b_25_result = None
        b_26_result = None
        b_27_result = None

        # Metric BT-B-19
        advisory_speed_limit, time_received_first_msg, b_19_result = get_basic_travel_TCM_data(bag)
        
        # Convert advisory speed limit from BT-B-19 to m/s for future metric evaluations
        advisory_speed_limit = advisory_speed_limit * 0.44704 # Conversion from mph to m/s

        lanelets_in_geofence = get_geofence_lanelets(bag, time_enter_geofence, advisory_speed_limit)
        
        # Metric BT-B-1
        b_1_result = check_basic_travel_geofence_lanelets_in_initial_route(bag, lanelets_in_geofence)

        # Metric BT-B-2
        b_2_result = check_geofence_lanelet_speed_limits(bag, lanelets_in_geofence, advisory_speed_limit)

        # Metric BT-B-3
        b_3_result = check_time_received_first_TCM_message(bag, time_received_first_msg, time_enter_geofence, advisory_speed_limit)

        # Metric BT-B-4: This metric is manually-evaluated using the vehicle speed plot

        # Metric BT-B-5 
        b_5_result = None
        if bag_file in BT_follower_black_pacifica_bag_files:
            check_lane_merge_before_geofence(bag)
        else:
            print("B-5: N/A (Leader Vehicle)")

        if bag_file in BT_follower_black_pacifica_bag_files:
            time_last_sent_follower_join, time_last_sent_join_at_rear, time_received_first_acceptance, time_received_second_acceptance = get_follower_platoon_negotiation_times(bag, time_test_start_engagement)
            duration_platoon_negotiation = get_platoon_negotiation_duration(time_received_second_acceptance, time_last_sent_join_at_rear, is_leader=False)
            start_lane_following_lanelet = 34915
            b_6_result = check_join_at_rear_negotiation(bag, time_received_first_acceptance, time_last_sent_join_at_rear, time_test_start_engagement, start_lane_following_lanelet)
            b_7_result = check_follower_join_negotiation(time_received_first_acceptance, time_last_sent_follower_join)
            b_8_result = check_time_headway_after_platoon_negotiation(bag, time_received_second_acceptance)
            b_9_result = check_follower_state_after_platoon_negotiation(bag, time_last_sent_join_at_rear)
            print("B-10: N/A (Follower Vehicle)")
            b_11_result = check_distance_gap_during_platooning(bag, time_received_second_acceptance, time_test_end_engagement)
            b_12_result = check_percentage_successful_status_msg_follower(bag, time_received_second_acceptance)

        else:
            print("B-6: N/A (Leader Vehicle)")
            print("B-7: N/A (Leader Vehicle)")
            print("B-8: N/A (Leader Vehicle)")
            print("B-9: N/A (Leader Vehicle)")
            time_last_received_follower_join, time_last_received_join_at_rear, time_sent_first_acceptance, time_sent_second_acceptance = get_leader_platoon_negotiation_times(bag, time_test_start_engagement)
            duration_platoon_negotiation = get_platoon_negotiation_duration(time_sent_second_acceptance, time_last_received_join_at_rear, is_leader=True)
            b_10_result = check_leader_state_after_platoon_negotiation(bag, time_last_received_join_at_rear)
            print("B-11: N/A (Leader Vehicle)")
            b_12_result = check_percentage_successful_status_msg_leader(bag, time_sent_second_acceptance)
            b_13_result = check_deceleration_before_geofence(bag, time_enter_geofence, original_speed_limit, advisory_speed_limit)
            b_14_result = check_acceleration_distance_after_geofence(bag, time_exit_geofence)
            b_15_result = check_acceleration_rate_after_geofence(bag, time_exit_geofence, original_speed_limit, advisory_speed_limit)

        b_16_result = check_steady_state_after_geofence(bag, time_exit_geofence, time_test_end_engagement, original_speed_limit)

        b_17_result = check_speed_limit_when_not_in_geofence(bag, time_test_start_engagement, time_enter_geofence, time_exit_geofence, time_test_end_engagement, original_speed_limit)

        b_18_result = check_advisory_speed_limit(bag, advisory_speed_limit, original_speed_limit)

        if bag_file in BT_follower_black_pacifica_bag_files:
            print("B-20: N/A (Follower Vehicle)")
            b_21_result = check_vehicle_inside_lane(bag, time_test_start_engagement, time_test_end_engagement, is_front_vehicle = False)
            b_22_result = check_vehicle_always_within_maximum_crosstrack(bag, time_test_start_engagement, time_test_end_engagement, is_front_vehicle = False)
            print("B-23: N/A (Follower Vehicle)")
            print("B-24: N/A (Follower Vehicle)")
            b_25_result = check_duration_before_successful_time_headway(bag, time_received_second_acceptance, time_enter_geofence)
            print("B-26: N/A (Follower Vehicle)")
            print("B-26: N/A (Follower Vehicle)")
        else:
            b_20_result = check_vehicle_inside_lane(bag, time_test_start_engagement, time_test_end_engagement, is_front_vehicle = True)
            print("B-21: N/A (Leader Vehicle)")
            print("B-22: N/A (Leader Vehicle)")
            b_23_result = check_vehicle_always_within_maximum_crosstrack(bag, time_test_start_engagement, time_test_end_engagement, is_front_vehicle = True)
            b_24_result = check_percentage_successful_info_msg(bag, time_sent_second_acceptance)
            print("B-25: N/A (Leader Vehicle)")
            b_26_result = check_max_time_between_status_msg(bag, time_sent_second_acceptance)
            b_27_result = check_max_time_between_info_msg(bag, time_sent_second_acceptance)

        # Get vehicle type that this bag file is from
        vehicle_name = "Unknown"
        if bag_file in BT_leader_white_pacifica_bag_files:
            vehicle_name = "White Pacifica"
        elif bag_file in BT_follower_black_pacifica_bag_files:
            vehicle_name = "Black Pacifica"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        vehicle_role = "Unknown"
        if bag_file in BT_leader_white_pacifica_bag_files:
            vehicle_role = "Leader"
        elif bag_file in BT_follower_black_pacifica_bag_files:
            vehicle_role = "Follower"

        # Write simple pass/fail results to .csv file for appropriate row:
        csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
                                     b_1_result, b_2_result, b_3_result, b_4_result, b_5_result, b_6_result, b_7_result,
                                     b_8_result, b_9_result, b_10_result, b_11_result, b_12_result, b_13_result, b_14_result, 
                                     b_15_result, b_16_result, b_17_result, b_18_result, b_19_result, b_20_result,
                                     b_21_result, b_22_result, b_23_result, b_24_result, b_25_result, b_26_result, b_27_result])
        
    sys.stdout = orig_stdout
    text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()