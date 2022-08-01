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
#   python3.7 analyze_wz_rosbags.py <path to folder containing Workzone Use Case .bag files>

def generate_speed_plot(bag):
    # Get the vehicle speed and plot it
    speed_received_first_msg = 0.0
    first = True
    times = []
    speeds = []
    crosstracks = []
    downtracks = []
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status']):
    #for topic, msg, t in bag.read_messages(topics=['/guidance/route_state']):
        if first:
            time_start = t
            first = False
            continue
        
        times.append((t-time_start).to_sec())
        speeds.append(msg.speed * 0.621371) # Conversion from kph to mph
        #crosstracks.append(msg.cross_track)
        #downtracks.append(msg.down_track)
    
    plt.plot(times,speeds)
    #plt.plot(times,crosstracks)
    #plt.plot(times,downtracks)
    plt.show()

    return


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
            #print(msg)
            found_geofence_entrance_time = True
            is_on_active_geofence = True
        
        # If final occurrence of being in the active geofence, set the end time
        if (not msg.is_on_active_geofence and is_on_active_geofence):
            time_exit_active_geofence = t
            found_geofence_exit_time = True
            
            time_in_geofence = (time_exit_active_geofence - time_enter_active_geofence).to_sec()
            print("Spent " + str(time_in_geofence) + " sec in geofence. Started at " + str(time_enter_active_geofence.to_sec()))
            is_on_active_geofence = False
            #break

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

###########################################################################################################
# Workzone WZ-1: The geofenced area is a part of the initial route plan.
#
# Workzone WZ-5: The vehicle receives a message from CC that includes the closed lane ahead. The vehicle 
#                processes this closed lane information.
###########################################################################################################
def check_geofence_in_initial_route(bag, closed_lanelets):
    # Get each set route from the bag file (includes set routes and updated/re-rerouted routes)
    shortest_path_lanelets = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route']):
        # Print as Debug Statement
        print("Shortest Path Route Update at " + str(t.to_sec()) + ": " + str(msg.shortest_path_lanelet_ids))
        
        shortest_path_lanelets.append([])
        for lanelet_id in msg.shortest_path_lanelet_ids:
            shortest_path_lanelets[-1].append(lanelet_id)

    # If there are two route paths, check that the first (original) route contains the closed lanelet(s) and the second route doesn't
    # Note: Assumes there should be only two routes: (1) the initial route and (2) the re-routed route
    initial_route_includes_closed_lane = False # Flag for B-1 success
    map_is_updated_for_closed_lane = False # Flag for B-11 success
    if (len(shortest_path_lanelets) == 2):
        original_shortest_path = shortest_path_lanelets[0]
        rerouted_shortest_path = shortest_path_lanelets[1]

        for lanelet_id in closed_lanelets:
            if lanelet_id in original_shortest_path:
                initial_route_includes_closed_lane = True
            else:
                initial_route_includes_closed_lane = False
                break

        for lanelet_id in closed_lanelets:
            if lanelet_id not in rerouted_shortest_path:
                map_is_updated_for_closed_lane = True
            else:
                map_is_updated_for_closed_lane = False
                break
    else:
        print("Invalid quantity of route updates found in bag file (" + str(len(shortest_path_lanelets)) + " found, 2 expected)")

    # Print result statements and return success flags
    if (initial_route_includes_closed_lane):
        print("WZ-1 succeeded; all closed lanelets " + str(closed_lanelets) + " were in the initial route")
    else:
        print("WZ-1 failed: not all closed lanelets " + str(closed_lanelets) + " were in the initial route.")

    if (map_is_updated_for_closed_lane):
        print("WZ-5 succeeded: no closed lanelets " + str(closed_lanelets) + " were in the re-routed route.")
    else:
        print("WZ-5 failed: at least 1 closed lanelet " + str(closed_lanelets) + " was in the re-routed route.")

    return initial_route_includes_closed_lane, map_is_updated_for_closed_lane

###########################################################################################################
# Workzone WZ-2: Amount of time that the vehicle is going at steady state (e.g. same lane, constant speed) 
#                before it receives the first CC message. (> 3 seconds)
###########################################################################################################
def check_steady_state_before_first_received_message(bag, time_start_engagement, time_received_first_message, original_speed_limit):
    # (m/s) Threshold offset of vehicle speed to speed limit to be considered at steady state
    threshold_speed_limit_offset = 0.89408 # 0.89408 m/s is 1 mph
    min_steady_state_speed = original_speed_limit - threshold_speed_limit_offset
    max_steady_state_speed = original_speed_limit + threshold_speed_limit_offset

    # (seconds) Minimum time between vehicle reaching steady state and first TIM MobilityOperation message being received
    min_time_between_steady_state_and_msg = 10.0

    # Get the time that the vehicle reaches within the set offset of the speed limit (while system is engaged)
    time_start_steady_state = 0.0
    has_reached_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement):
        current_speed = msg.twist.linear.x # Current vehicle speed in m/s
        if (max_steady_state_speed >= current_speed >= min_steady_state_speed):
            has_reached_steady_state = True
            time_start_steady_state = t
            break
    
    # Check if the time the vehicle reaches steady state is more than 'min_time_between_steady_state_and_msg' seconds before the first received TIM message
    if (has_reached_steady_state):
        time_between_steady_state_and_msg = (time_received_first_message - time_start_steady_state).to_sec()
        if (time_between_steady_state_and_msg >= min_time_between_steady_state_and_msg):
            is_successful = True
            print("WZ-2 succeeded; reached steady state " + str(time_between_steady_state_and_msg) + " seconds before receiving first TIM or TCM message.")
        else:
            is_successful = False
            if (time_between_steady_state_and_msg > 0):
                print("WZ-2 failed; reached steady state " + str(time_between_steady_state_and_msg) + " seconds before receiving first TIM or TCM message.")
            else:
                print("WZ-2 failed; reached steady state " + str(-time_between_steady_state_and_msg) + " seconds after receiving first TIM or TCM message.")
    else:
        print("WZ-2 failed; vehicle never reached steady state during rosbag recording.")
        is_successful = False

    return is_successful

###########################################################################################################
# Workzone WZ-4: The vehicle receives a message from CC that includes the new speed limit ahead. The vehicle 
#                processes this new speed limit.
#
# Workzone WZ-5: The vehicle receives a message from CC that includes the closed lane ahead. The vehicle 
#                processes this closed lane information.
#
# Workzone WZ-6: The vehicle receives a message from CC that includes the "taper right" closed lane ahead. 
#                The vehicle processes this closed lane information.
#
# Workzone WZ-7: The vehicle receives a message from CC that includes the "open right" closed lane ahead. 
#                The vehicle processes this closed lane information.
#
# Workzone WZ-8: The vehicle receives a message from CC that includes the "open right" closed lane ahead. 
#                The vehicle processes this closed lane information.
###########################################################################################################
def get_workzone_TCM_data(bag):
    # Check that TCM Messages are received for closed lane, open right, taper right, direction reversal, and advisory speed
    has_closed_lane = False
    has_advisory_speed = False
    has_direction_reversal = False
    has_taper_right = False
    has_open_right = False
    advisory_speed = 0.0
    time_first_msg_received = rospy.Time()
    first = True
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        if first:
            time_first_msg_received = t
            first = False
        
        if msg.tcmV01.params.detail.choice == 5 and msg.tcmV01.params.detail.closed == 1:
            #print(msg)
            has_closed_lane = True
        elif msg.tcmV01.params.detail.choice == 5 and msg.tcmV01.params.detail.closed == 3:
            latitude = msg.tcmV01.geometry.reflat
            longitude = msg.tcmV01.geometry.reflon
            print("Taper right has stop bar location at lat=" + str(latitude) + ", longitude=" + str(longitude))
            has_taper_right = True
        elif msg.tcmV01.params.detail.choice == 5 and msg.tcmV01.params.detail.closed == 5:
            #print(msg)
            has_open_right = True
        elif msg.tcmV01.params.detail.choice == 7 and msg.tcmV01.params.detail.direction == 1:
            #print(msg)
            has_direction_reversal = True
        elif msg.tcmV01.params.detail.choice == 12:
            #print(msg)
            has_advisory_speed = True
            advisory_speed = msg.tcmV01.params.detail.maxspeed

        if has_closed_lane and has_advisory_speed and has_direction_reversal and has_taper_right and has_open_right:
            print("TCM Messages Received: Closed Lane; Direction Reversal; Taper Right; Open Right; Advisory Speed: " + str(advisory_speed) +  \
                " mph)")
            break
    
    wz_4_successful = False
    if has_advisory_speed:
        print("WZ-4 succeeded; TCM message received with advisory speed limit " + str(advisory_speed) + " mph")
        wz_4_successful = True
    else:
        print("WZ-4 failed; no TCM message with an advisory speed limit was received.")

    wz_5_successful = False
    if has_closed_lane:
        print("WZ-5 succeeded; TCM message received with closed lane")
        wz_5_successful = True
    else:
        print("WZ-5 failed; no TCM message with closed lane.")

    wz_6_successful = False
    if has_taper_right:
        print("WZ-6 succeeded; TCM message received with a taper right closed lane")
        wz_6_successful = True
    else:
        print("WZ-6 failed; no TCM message with a taper right closed lane was received.")
    
    wz_7_successful = False
    if has_open_right:
        print("WZ-7 succeeded; TCM message received with an open right closed lane")
        wz_7_successful = True
    else:
        print("WZ-7 failed; no TCM message with an open right closed lane was received.")

    wz_8_successful = False
    if has_direction_reversal:
        print("WZ-8 succeeded; TCM message received with a direction reversal")
        wz_8_successful = True
    else:
        print("WZ-8 failed; no TCM message with a direction reversal was received.")

    return advisory_speed, time_first_msg_received, wz_4_successful, wz_5_successful, wz_6_successful, wz_7_successful, wz_8_successful

###########################################################################################################
# Workzone WZ-9: The vehicle shall continuously receive SPAT messages with a time gap between sequentially 
#                received SPAT messages below 0.5 seconds at least 90% of the time while engaged.
###########################################################################################################
def check_percentage_successful_spat_msg(bag, time_start_engagement, time_end_engagement):
    # Parameters used for the computation of this metric
    min_percent_time_successful = 90.0 # (90%); Percent of active platooning time that the maximum or better 'SPAT' frequency must be achieved
    max_duration_between_msgs = 0.50 # (Seconds); Maximum duration between sequentially broadcasted 'SPAT' messages

    # Obtain the quantity of SPAT messages received during engagement
    duration_since_prev_msg = 0.0
    total_time_between_unsuccessful_msgs = 0.0
    total_time_between_successful_msgs = 0.0
    largest_duration_between_msgs = 0.0
    total_time = 0.0
    total_time_between_unsuccessful_msgs = 0.0
    count_msgs = 0
    count_msgs_above_max_duration = 0
    time_prev_msg = rospy.Time()
    first = True
    current_signal_state = 0
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_spat'], start_time = time_start_engagement, end_time = time_end_engagement):
        if first:
            time_prev_msg = t
            first = False
            continue
        
        for movement in msg.intersection_state_list:
            signal_state = movement.movement_list[1].movement_event_list[0].event_state.movement_phase_state
            if (signal_state != current_signal_state):
                print("Transitioned from signal " + str(current_signal_state) + " to " + str(signal_state) + " at " + str(t.to_sec()))
                current_signal_state = signal_state
            #print("***************************")
            #print(len(movement.movement_list))
            #print(str(count_msgs) + " msg has phase:")
            #print(str(movement.movement_list[0].movement_event_list[0].event_state.movement_phase_state) + " for SG_ID " + str(movement.movement_list[0].signal_group))
            #print("SG_ID " + str(movement.movement_list[1].signal_group) + " has signal " + str(movement.movement_list[1].movement_event_list[0].event_state.movement_phase_state))

            #for move in movement.movement_list:
            #    for event in move.movement_event_list:
            #        print(event.event_state.movement_phase_state)
            #        msg.intersection_state_list.movement_list.event_state.movement_phase_state
        

        
        duration_since_prev_msg = (t - time_prev_msg).to_sec()
        time_prev_msg = t
        #print("Duration between SPAT messages: " + str(duration_since_prev_msg) + " sec")

        # Track the number of messages received after the (1/min_frequency) duration just for debugging purposes
        if (duration_since_prev_msg >= max_duration_between_msgs):
            total_time_between_unsuccessful_msgs += duration_since_prev_msg
            count_msgs_above_max_duration += 1
        else:
            total_time_between_successful_msgs += duration_since_prev_msg

        if duration_since_prev_msg > largest_duration_between_msgs:
            largest_duration_between_msgs = duration_since_prev_msg
        
        total_time += duration_since_prev_msg
        count_msgs += 1
    
    percent_time_successful = (total_time_between_successful_msgs / total_time) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if percent_time_successful >= min_percent_time_successful:
        print("WZ-9 Succeeded; Time between SPAT messages was below " + str(round(max_duration_between_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")
        is_successful = True
    else:
        print("WZ-9 Failed; Time between SPAT messages was below " + str(round(max_duration_between_msgs,3)) + " seconds " + str(round(percent_time_successful,3)) + \
              "% of the time. (Must be greater than " + str(round(min_percent_time_successful,3)) + "%)")        
        is_successful = False
    
    pct_above_max_duration = (float(count_msgs_above_max_duration) / float(count_msgs)) * 100.0
    print(str(count_msgs) + " Total SPAT Messages sent (" + str(round(100-pct_above_max_duration,3)) + "% Successful); " + str(count_msgs_above_max_duration) + " after " + str(round(max_duration_between_msgs,3)) + " sec (" \
        + str(round(pct_above_max_duration,3)) + "%);; Longest duration " \
          + " between SPAT messages was " + str(largest_duration_between_msgs) + " sec")

    return is_successful

###########################################################################################################
# Workzone WZ-10: The vehicle shall never receive sequential SPAT messages with a time gap between messages 
#                 that exceeds 1 second.
###########################################################################################################
def check_duration_between_spat_msg_below_max_duration(bag, time_start_engagement, time_end_engagement):
    # Parameters used for the computation of this metric
    max_duration_between_msgs = 1.00 # (Seconds); Max duration between sequentially broadcasted 'SPAT' messages

    # Obtain the quantity of SPAT messages received during engagement
    duration_since_prev_msg = 0.0
    total_time_between_unsuccessful_msgs = 0.0
    total_time_between_successful_msgs = 0.0
    largest_duration_between_msgs = 0.0
    total_time = 0.0
    total_time_between_unsuccessful_msgs = 0.0
    count_msgs = 0
    count_msgs_above_max_duration = 0
    time_prev_msg = rospy.Time()
    first = True
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_spat'], start_time = time_start_engagement, end_time = time_end_engagement):
        if first:
            time_prev_msg = t
            first = False
            continue
        
        m = msg
        duration_since_prev_msg = (t - time_prev_msg).to_sec()
        time_prev_msg = t
        #print("Duration between SPAT messages: " + str(duration_since_prev_msg) + " sec")

        # Track the number of messages received after the (1/min_frequency) duration just for debugging purposes
        if (duration_since_prev_msg >= max_duration_between_msgs):
            total_time_between_unsuccessful_msgs += duration_since_prev_msg
            count_msgs_above_max_duration += 1
            print(str(duration_since_prev_msg) + " sec between SPAT messages")
        else:
            total_time_between_successful_msgs += duration_since_prev_msg

        if duration_since_prev_msg > largest_duration_between_msgs:
            largest_duration_between_msgs = duration_since_prev_msg
        
        total_time += duration_since_prev_msg
        count_msgs += 1
    #print(msg)
    percent_time_successful = (total_time_between_successful_msgs / total_time) * 100.0

    # Update is_successful flag and print debug statements
    is_successful = False
    if count_msgs_above_max_duration == 0:
        print("WZ-10 Succeeded; " + str(count_msgs_above_max_duration) + " SPAT messages more than 1.0 sec apart")
        is_successful = True
    else:
        print("WZ-10 Succeeded; " + str(count_msgs_above_max_duration) + " SPAT messages more than 1.0 sec apart")    
        is_successful = False

    return is_successful

###########################################################################################################
# Workzone WZ-11: After the vehicle processes a received SPAT message and determines that it will arrive at
#                 a red or yellow signal indication, the vehicle's actual trajectory in preparation for a 
#                 stop will include a deceleration section. The average deceleration over the entire section 
#                 shall be no less than 1 m/s^2, and the average deceleration over any 1-second portion of 
#                 the section shall be no greater than 2.0 m/s^2.
###########################################################################################################
def check_deceleration_for_red_light(bag, time_start_engagement):
    # Obtain the timestamp at which the vehicle comes to a complete stop at the red light
    time_stopped = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement):
        if (t-time_start_engagement).to_sec() < 5.0:
            continue
        
        if msg.speed < 0.10:
            time_stopped = t
            print("Vehicle came to complete stop " + str((time_stopped - time_start_engagement).to_sec()) + " seconds after engagement")
            break

    # Verify that light indication is RED (signal phase 3) at time_stopped
    one_second_duration = rospy.Duration(1.0)
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_spat'], start_time = time_stopped - one_second_duration):
        signal_at_stop = msg.intersection_state_list[0].movement_list[1].movement_event_list[0].event_state.movement_phase_state

        if (signal_at_stop == 3):
            print("Vehicle came to a stop at a Red Signal Indication: " + str(signal_at_stop))
        elif (signal_at_stop == 6):
            print("Vehicle came to a stop at a Green Signal Indication: " + str(signal_at_stop))
        elif (signal_at_stop == 8):
            print("Vehicle came to a stop at a Yellow Signal Indication: " + str(signal_at_stop))
        
        break

    # Obtain time_last_accel, which is the last acceleration before decelerating to a complete stop
    prev_speed = 0.0
    first = True
    time_last_accel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement, end_time = time_stopped-one_second_duration):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        if delta_speed >= 0:
            time_last_accel = t
            speed_after_final_increase = msg.speed * 0.277777 # Conversion from kph to m/s
        
        prev_speed = msg.speed
    
    print("Final speed increase before stop occurred at " + str(time_last_accel.to_sec()) + ", " + str((time_last_accel-time_start_engagement).to_sec()) + " seconds after engagement")
    print("Speed after final acceleration before decelerating to a stop was " + str(speed_after_final_increase) + " m/s ")

    # Obtain average decelation over all 1-second windows in deceleration section
    all_one_second_windows_successful = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_last_accel, end_time = (time_stopped-one_second_duration)):
        speed_initial = msg.speed * 0.277777 # Conversion from kph to m/s
        time_initial = t

        speed_final = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = (time_initial+one_second_duration)):
            speed_final = msg.speed * 0.277777 # Conversion from kph to m/s
            t_final = t
            break

        one_second_decel = (speed_initial - speed_final) / (t_final-time_initial).to_sec()

        if one_second_decel > 2.0:
            print("Failure: 1-second window had decel rate was " + str(one_second_decel) + " m/s^2; end time was " + str((time_stopped - t_final).to_sec()) + " seconds before stopping")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has decel rate at " + str(one_second_decel) + " m/s^2")

    # Obtain the average deceleration over the entire deceleration section
    average_total_decel = speed_after_final_increase / (time_stopped-time_last_accel).to_sec()

    total_deceleration_rate_successful = False
    if average_total_decel >= 1.0:
        print("WZ-11 (total deceleration before red light stop) succeeded; total average deceleration was " + str(average_total_decel) + " m/s^2")
        total_deceleration_rate_successful = True
    else:
        print("WZ-11 (total deceleration before red light stop) failed; total average deceleration was " + str(average_total_decel) + " m/s^2")
    
    if all_one_second_windows_successful:
        print("WZ-11 (1-second window deceleration before red light stop) succeeded; no occurrences of 1-second average deceleration above 2.0 m/s^2")
    else:
        print("WZ-11 (1-second window deceleration before red light stop) failed; at least one occurrence of 1-second average deceleration above 2.0 m/s^2")

    is_successful = False
    if total_deceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True

    return is_successful, time_last_accel, time_stopped

def euclidean_distance(p1, p2):
    distance = math.sqrt((p1[0]-p2[0])**2 + (p1[1] - p2[1])**2)
    return distance

###########################################################################################################
# Workzone WZ-12: After the vehicle processes a received SPAT message and determines that it will arrive at
#                 a red or yellow signal indication, the vehicle will come to a complete stop 0-15 feet before 
#                 the corresponding stop bar for the specified traffic signal.
###########################################################################################################
def check_stop_location_for_red_light(bag, stop_bar_location, dist_rear_axle_to_front_bumper, time_start_engagement, time_start_decel, time_stopped):
    # Obtain true stopping location in map frame
    system_stop_location = [0,0]
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_stopped):
        system_stop_location[0] = msg.pose.position.x
        system_stop_location[1] = msg.pose.position.y
        print("Red Light: System stopped at map location x=" + str(system_stop_location[0]) + ", y=" + str(system_stop_location[1]))
        break

    # Obtain the location of the system at the start of engagement
    system_start_location = [0,0]
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_start_engagement):
        system_start_location[0] = msg.pose.position.x
        system_start_location[1] = msg.pose.position.y
        break
    
    dist_start_to_vehicle_stop = euclidean_distance(system_start_location, system_stop_location)
    dist_start_to_stop_bar = euclidean_distance(system_start_location, stop_bar_location)
    dist_vehicle_to_stop_bar = euclidean_distance(stop_bar_location, system_stop_location)
    print("Distance start to stop: " + str(dist_start_to_vehicle_stop) + " meters")
    print("Distance start to stop bar: " + str(dist_start_to_stop_bar) + " meters")
    print("Distance stop bar to stop: " + str(dist_vehicle_to_stop_bar) + " meters")

    is_successful = False
    if dist_start_to_vehicle_stop < dist_start_to_stop_bar:
        dist_vehicle_to_stop_bar = dist_vehicle_to_stop_bar - dist_rear_axle_to_front_bumper
        if dist_vehicle_to_stop_bar < 0:
            print("WZ-12 Failed; Vehicle stopped with front bumper " + str(abs(dist_vehicle_to_stop_bar)) + " meters after stop bar")
        elif dist_vehicle_to_stop_bar < 15.0:
            print("WZ-12 Succeeded; Vehicle stopped with front bumper " + str(dist_vehicle_to_stop_bar) + " meters in front of stop bar")
            is_successful = True
        else:
            print("WZ-12 Failed; Vehicle stopped with front bumper " + str(dist_vehicle_to_stop_bar) + " meters before stop bar")
    else:
        dist_vehicle_to_stop_bar = dist_vehicle_to_stop_bar + dist_rear_axle_to_front_bumper
        print("WZ-12 Failed; Vehicle stopped with front bumper " + str(dist_vehicle_to_stop_bar) + " meters after stop bar")

    return is_successful

###########################################################################################################
# Workzone WZ-13: If the vehicle is stopped for a red or yellow signal indication and receives a new SPAT 
#                 message for a green signal indication, the vehicle will begin accelerating within 0-4 seconds.
###########################################################################################################
def check_acceleration_time_after_green_light(bag, time_stopped):
    max_time_to_start_accel = 4.0 # (Seconds)

    # Get occurrence of time_start_green_light after coming to a stop
    time_start_green_light = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_spat'], start_time = time_stopped):
        signal_at_stop = msg.intersection_state_list[0].movement_list[1].movement_event_list[0].event_state.movement_phase_state

        if signal_at_stop == 6:
            time_start_green_light = t
            break

    # Get timestamp of first acceleration after time_start_green_lightm
    time_start_accel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_green_light):
        if msg.speed > 0.1:
            time_start_accel = t
            break

    time_to_start_accel = (time_start_accel-time_start_green_light).to_sec()

    is_successful = False
    if time_to_start_accel <= max_time_to_start_accel:
        print("WZ-13 Succeeded; after green light, system took " + str(time_to_start_accel) + " seconds to begin accelerating (less than 4)")
        is_successful = True
    else:
        print("WZ-13 Succeeded; after green light, system took " + str(time_to_start_accel) + " seconds to begin accelerating (more than 4)")

    return is_successful

###########################################################################################################
# Workzone WZ-14: If the vehicle is stopped for a red or yellow signal indication and receives a new SPAT
#                 message for a green signal indication, the actual trajectory back to normal operations 
#                 will include an acceleration section. The average acceleration over the entire section 
#                 shall be no less than 1 m/s^2, and the average acceleration over any 1-second portion of 
#                 the section shall be no greater than 2.0 m/s^2.
###########################################################################################################
def check_acceleration_after_stop(bag, time_stopped):
    # Obtain time_start_accel
    time_start_accel = rospy.Time()
    speed_start_accel = 0.0
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_stopped):
        if msg.speed >= 0.1:
            speed_start_accel = msg.speed * 0.277778 # Conversion kph to m/s
            time_start_accel = t
            break

    # Obtain time_end_accel (first deceleration after starting accel; also print the speed at end of accel)
    prev_speed = 0.0
    first = True
    time_end_accel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_accel):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        if delta_speed <= 0:
            time_end_accel = t
            speed_end_accel = msg.speed * 0.277778 # Conversion kph to m/s
            speed_after_final_increase_mph = msg.speed * 0.621371 # Conversion from kph to mph
            print("Speed at end of accel after light turns green: " + str(speed_after_final_increase_mph) + " mph")
            break

        prev_speed = msg.speed

    # Check the total average
    total_average_decel = (speed_end_accel - speed_start_accel) / (time_end_accel-time_start_accel).to_sec()
    print("Total average decel: " + str(total_average_decel) + " m/s^2")

    # Check the 1-second window averages
    # Obtain average decelation over all 1-second windows in acceleration section
    one_second_duration = rospy.Duration(1.0)
    all_one_second_windows_successful = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_accel, end_time = (time_end_accel-one_second_duration)):
        speed_initial = msg.speed * 0.277777 # Conversion from kph to m/s
        time_initial = t

        speed_final = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = (time_initial+one_second_duration)):
            speed_final = msg.speed * 0.277777 # Conversion from kph to m/s
            t_final = t
            break
        
        one_second_decel = (speed_final - speed_initial) / (t_final-time_initial).to_sec()

        if one_second_decel > 2.0:
            print("Failure: 1-second window had accel rate was " + str(one_second_decel) + " m/s^2; end time was " + str((time_stopped - t_final).to_sec()) + " seconds before stopping")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has accel rate at " + str(one_second_decel) + " m/s^2")

    # Print success/failure statements
    total_deceleration_rate_successful = False
    if total_average_decel >= 1.0:
        print("WZ-14 (total accel after complete stop) succeeded; total average acceleration was " + str(total_average_decel) + " m/s^2")
        total_deceleration_rate_successful = True
    else:
        print("WZ-14 (total accel after complete stop) failed; total average acceleration was " + str(total_average_decel) + " m/s^2")
    
    if all_one_second_windows_successful:
        print("WZ-14 (1-second accel after complete stop) succeeded; no occurrences of 1-second average acceleration above 2.0 m/s^2")
    else:
        print("WZ-14 (1-second accel after complete stop) failed; at least one occurrence of 1-second average acceleration above 2.0 m/s^2")

    is_successful = False
    if total_deceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True

    return is_successful


###########################################################################################################
# Workzone WZ-15: If the vehicle processes a received SPAT message and determines that it will arrive at a 
#                 green traffic signal indication, the vehicle shall travel past the traffic signal at a speed 
#                 of +/- 2 mph of the speed limit while the signal indication is green.
###########################################################################################################
def check_vehicle_speed_at_green_light(bag, advisory_speed_limit, stop_bar_location, time_start_engagement):
    # Get time_vehicle_at_stop_bar (which is the time it passes the stop bar line)
    min_dist_to_stop_bar = 1.0 # meteres
    vehicle_location = [0,0]
    time_vehicle_at_stop_bar = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_start_engagement):
        vehicle_location[0] = msg.pose.position.x
        vehicle_location[1] = msg.pose.position.y
        if (euclidean_distance(vehicle_location, stop_bar_location) <= min_dist_to_stop_bar):
            time_vehicle_at_stop_bar = t
            break

    # Get traffic signal value at time_pass_green_light
    has_passed_at_green_light = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_spat'], start_time = time_vehicle_at_stop_bar):
        signal_at_stop = msg.intersection_state_list[0].movement_list[1].movement_event_list[0].event_state.movement_phase_state

        if (signal_at_stop == 3):
            print("WZ-15 Failed; Vehicle passed light during a Red Signal Indication: ")
        elif (signal_at_stop == 6):
            print("WZ-15 succeeded; Vehicle passed light during a Green Signal Indication: ")
            has_passed_at_green_light = True
        elif (signal_at_stop == 8):
            print("WZ-15 Failed; Vehicle passed light during a Yellow Signal Indication: ")
        
        break

    # Get vehicle speed at time_pass_green_light

    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_vehicle_at_stop_bar):
        speed_at_light = msg.speed * 0.621371 # Conversion from kph to mph
        break

    has_passed_green_light_at_correct_speed = False
    original_speed_limit = advisory_speed_limit * 2.23694 # m/s to mph
    min_speed = original_speed_limit - 2.0
    max_speed = original_speed_limit + 2.0
    if min_speed <= speed_at_light <= max_speed:
        has_passed_at_green_light = True
        print("WZ-15 succeeded; vehicle passed light at speed " + str(speed_at_light) + " mph")
    else:
        print("WZ-15 failed; vehicle passed light at speed " + str(speed_at_light) + " mph")
        

    # Print success/failure statements
    is_successful = False
    if has_passed_at_green_light and has_passed_green_light_at_correct_speed:
        is_successful = True

    return is_successful, time_vehicle_at_stop_bar

###########################################################################################################
# Workzone WZ-16: The actual trajectory in preparation for the geofenced area with an advisory speed limit 
#                 will include a deceleration section. The average deceleration over the entire section shall 
#                 be no less than 1 m/s^2, and the average deceleration over any 1-second portion of the section 
#                 shall be no greater than 2.0 m/s^2.
###########################################################################################################
def check_deceleration_after_green_light(bag, time_vehicle_at_stop_bar, time_start_engagement):
    # Obtain the last speed decrease time before reaching stop bar
    prev_speed = 0.0
    first = True
    time_last_decel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement, end_time = time_vehicle_at_stop_bar):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        if delta_speed <= 0:
            time_last_decel = t
            speed_after_final_decel = msg.speed * 0.277777 # Conversion from kph to m/s
            #print("Time " + str(t.to_sec()) + " has speed " + str(speed_after_final_decel))
        
        prev_speed = msg.speed

    prev_speed = 0.0
    first = True
    time_last_accel = rospy.Time()
    one_second_duration = rospy.Duration(1.0)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement, end_time = time_last_decel - one_second_duration):
        if first:
            prev_speed = msg.speed
            first = False
            continue

        delta_speed = msg.speed - prev_speed
        if delta_speed >= 0:
            time_last_accel = t
            speed_after_final_accel = msg.speed * 0.277777 # Conversion from kph to m/s
        
        prev_speed = msg.speed
    
    speed_after_final_accel_mph = speed_after_final_accel * 2.23694
    speed_after_final_decel_mph = speed_after_final_decel * 2.23694
    print("(WZ-16): Speed start of decel: " + str(speed_after_final_accel) + " m/s; " + str(speed_after_final_accel_mph) + " mph")
    print("(WZ-16): Speed end of decel " + str(speed_after_final_decel) + " m/s; " + str(speed_after_final_decel_mph) + " mph")
    speed_start_decel = speed_after_final_accel
    speed_end_decel = speed_after_final_decel

    total_average_decel = (speed_start_decel - speed_end_decel) / (time_last_decel - time_last_accel).to_sec()

    
    # Obtain average decelation over all 1-second windows in deceleration section
    one_second_duration = rospy.Duration(1.0)
    all_one_second_windows_successful = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_last_accel, end_time = (time_last_decel-one_second_duration)):
        speed_initial = msg.speed * 0.277777 # Conversion from kph to m/s
        time_initial = t

        speed_final = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = (time_initial+one_second_duration)):
            speed_final = msg.speed * 0.277777 # Conversion from kph to m/s
            t_final = t
            break

        one_second_decel = (speed_initial - speed_final) / (t_final-time_initial).to_sec()

        if one_second_decel > 2.0:
            print("Failure: 1-second window had decel rate was " + str(one_second_decel) + " m/s^2; end time was " + str((time_last_accel - t_final).to_sec()) + " seconds before end of deceleration")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has decel rate at " + str(one_second_decel) + " m/s^2")
    
    # Print success/failure statements
    total_deceleration_rate_successful = False
    if total_average_decel >= 1.0:
        print("WZ-16 (total decel for geofence) succeeded; total average deceleration was " + str(total_average_decel) + " m/s^2")
        total_deceleration_rate_successful = True
    else:
        print("WZ-16 (total decel for geofence) failed; total average deceleration was " + str(total_average_decel) + " m/s^2")
    
    if all_one_second_windows_successful:
        print("WZ-16 (1-second decel for geofence) succeeded; no occurrences of 1-second average deceleration above 2.0 m/s^2")
    else:
        print("WZ-16 (1-second decel for geofence) failed; at least one occurrence of 1-second average deceleration above 2.0 m/s^2")


    is_successful = False
    if total_deceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True
    return is_successful

###########################################################################################################
# Workzone WZ-17: When navigating the "taper right" closed lane section of the route, the vehicle will travel 
#                 sequentially through lanelets 1302928, 1303118, and 1303305.
###########################################################################################################

###########################################################################################################
# Workzone WZ-18: When navigating the "open right" closed lane section of the route, the vehicle will travel
#                 sequentially through lanelets 1303305, 1303304, and 1302937.
###########################################################################################################

###########################################################################################################
# Workzone WZ-19: After exiting the geofenced area with an advisory speed limit, the vehicle will begin 
#                 accelerating back to the original speed limit within 0-30 feet.
###########################################################################################################

###########################################################################################################
# Workzone WZ-20: After exiting the geofenced area with an advisory speed limit, the actual trajectory back 
#                 to normal operations will include an acceleration section. The average acceleration over 
#                 the entire section shall be no less than 1 m/s^2, and the average acceleration over any 
#                 1-second portion of the section shall be no greater than 2.0 m/s^2.
###########################################################################################################

###########################################################################################################
# Workzone WZ-21: When not changing lanes, the vehicle is fully contained within its lane (it does not cross
#                 either of the lane lines) for at least 95% of the time spent with the CARMA system engaged.
###########################################################################################################

###########################################################################################################
# Workzone WZ-22: When not changing lanes, the vehicle never crosses the left or right lane line associated 
#                 with its current lane by more than 0.1 meters while the CARMA system is engaged.
###########################################################################################################

###########################################################################################################
# Workzone WZ-23: After exiting the geofenced area, system is at steady state for at least 3 seconds before 
#                 the end of the use case.
###########################################################################################################
def check_steady_state_after_geofence(bag, time_end_engagement, original_speed_limit):
    ten_second_duration = rospy.Duration(10.0)


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
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_end_engagement - ten_second_duration, end_time = time_end_engagement):
        # Vehicle has reached steady state when its speed within threshold range of steady state speed
        if (max_steady_state_speed >= msg.twist.linear.x >= min_steady_state_speed):
            time_start_steady_state = t
            has_steady_state = True
            break

    time_steady_state = (time_end_engagement - time_start_steady_state).to_sec()

    if (time_steady_state >= 3.0):
        print("WZ-23 succeeded; system ended use case at steady state for " + str(time_steady_state) + " seconds")
        is_successful = True
    else:
        if has_steady_state:
            print("WZ-23 failed; system ended use case at steady state for " + str(time_steady_state) + " seconds")
            is_successful = False
        if not has_steady_state:
            print("WZ-23 failed; system did not reach steady state at end of use case.")
            is_successful = False

    return is_successful

###########################################################################################################
# Basic Travel WZ-24: The entire scenario will satisfy all previous criteria using any of the speeds given here for 
#           the "regular speed limit" (i.e. the speed limit when not in the geo-fence).
###########################################################################################################
def check_speed_limit_when_not_in_geofence(bag, original_speed_limit):
    expected_speed_limit = 8.9408 # m/s (20 mph)

    # (m/s) Threshold offset from speed limit to account for floating point precision
    threshold_speed_limit_offset = 0.01
    max_speed_limit = expected_speed_limit + threshold_speed_limit_offset # (m/s)
    min_speed_limit = expected_speed_limit - threshold_speed_limit_offset # (m/s)

    is_successful = False
    if min_speed_limit <= original_speed_limit <= max_speed_limit:
        print("WZ-24 Succeeded; speed limit outside of geofence is " + str(original_speed_limit) + " m/s (20 mph)")
        is_successful = True
    else:
        print("WZ-24 Failed; speed limit outside of geofence is " + str(original_speed_limit) + " m/s")

    return is_successful

###########################################################################################################
# Workzone WZ-25: The advisory speed limit communicated by CARMA Cloud shall be 5 mph less than the 
#                    regular speed limit.
###########################################################################################################
def check_advisory_speed_limit(bag, advisory_speed_limit, original_speed_limit):
    # (m/s) Required offset from normal speed limit required for the advisory speed limit
    speed_limit_offset = 2.2352 # 4.4704 m/s is 10 mph
    # (m/s) Threshold offset from expected advisory speed limit to account for floating point precision
    threshold_speed_limit_offset = 0.01
    max_advisory_speed_limit = (original_speed_limit - speed_limit_offset) + threshold_speed_limit_offset
    min_advisory_speed_limit = (original_speed_limit - speed_limit_offset) - threshold_speed_limit_offset

    # Evaluate speed limits, print success/failure statement, and return success flag
    advisory_speed_limit_mph = int(advisory_speed_limit * 2.23694) # Conversion of m/s to mph
    original_speed_limit_mph = int(original_speed_limit * 2.23694) # Conversion of m/s to mph
    speed_limit_offset_mph = int(speed_limit_offset * 2.23694) # Conversion of m/s to mph
    if (max_advisory_speed_limit >= advisory_speed_limit >= min_advisory_speed_limit):
        print("WZ-25 succeeded; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")
        is_successful = True
    else:
        print("WZ-25 failed; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is not " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")       
        is_successful = False
    
    return is_successful

# Main Function; run all tests from here
def main():  
    if len(sys.argv) < 2:
        print("Need 1 arguments: process_bag.py <path to source folder with .bag files> ")
        exit()
    
    #source_folder = sys.argv[1]

    # Re-direct the output of print() to a specified .txt file:
    #orig_stdout = sys.stdout
    current_time = datetime.datetime.now()
    #text_log_filename = "Results_" + str(current_time) + ".txt"
    #text_log_file_writer = open(text_log_filename, 'w')
    #sys.stdout = text_log_file_writer

    # Create .csv file to make it easier to view overview of results (the .txt log file is still used for more in-depth information):
    #csv_results_filename = "Results_" + str(current_time) + ".csv"
    #csv_results_writer = csv.writer(open(csv_results_filename, 'w'))
    #csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Test Type",
    #                             "WZ-1 Result", "WZ-2 Result", "WZ-3 Result", "WZ-4 Result", "WZ-5 Result", "WZ-6 Result", 
    #                             "WZ-7 Result", "WZ-8 Result", "WZ-9 Result", "WZ-10 Result","WZ-11 Result", "WZ-12 Result", 
    #                             "WZ-13 Result", "WZ-14 Result", "WZ-15 Result", "WZ-16 Result", "WZ-17 Result", "WZ-18 Result", 
    #                             "WZ-19 Result", "WZ-20 Result", "WZ-21 Result", "WZ-22 Result", "WZ-23 Result", "WZ-24 Result", "WZ-25 Result"])
    
    # Create list of Red Light Workzone Black Pacifica bag files to be processed
    black_pacifica_red_bag_files = [] 

    # Create list of Red Light Workzone Ford Fusion bag files to be processed
    ford_fusion_red_bag_files = ["_2021-09-22-16-00-28-red.bag",
                                 "_2021-09-22-19-03-26-red.bag"]

    # Create list of Red Light Workzone Blue Lexus bag files to be processed
    blue_lexus_red_bag_files = []

    # Create list of Green Light Workzone Black Pacifica bag files to be processed
    black_pacifica_green_bag_files = []

    # Create list of Green Light Workzone Ford Fusion bag files to be processed
    ford_fusion_green_bag_files = []

    # Create list of Green Light Workzone Blue Lexus bag files to be processed
    blue_lexus_green_bag_files = []

    # Concatenate all Basic Travel bag files into one list
    red_light_bag_files = black_pacifica_red_bag_files + ford_fusion_red_bag_files + blue_lexus_red_bag_files
    green_light_bag_files = black_pacifica_green_bag_files + ford_fusion_green_bag_files + blue_lexus_green_bag_files
    WZ_bag_files = red_light_bag_files + green_light_bag_files

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in WZ_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in black_pacifica_red_bag_files:
            print("Black Pacifica Red Light Workzone Test Case")
        elif bag_file in ford_fusion_red_bag_files:
            print("Ford Fusion Red Light Workzone Test Case")
        elif bag_file in blue_lexus_red_bag_files:
            print("Blue Lexus Red Light Workzone Test Case")
        if bag_file in black_pacifica_green_bag_files:
            print("Black Pacifica Green Light Workzone Test Case")
        elif bag_file in ford_fusion_green_bag_files:
            print("Ford Fusion Green Light Workzone Test Case")
        elif bag_file in blue_lexus_green_bag_files:
            print("Blue Lexus Green Light Workzone Test Case")
            
        # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
        #sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(WZ_bag_files.index(bag_file) + 1) + " of " + str(len(WZ_bag_files)) + ")")
        #sys.stdout = text_log_file_writer

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
        wz_1_result = None
        wz_2_result = None
        wz_3_result = None
        wz_4_result = None
        wz_5_result = None
        wz_6_result = None
        wz_7_result = None
        wz_8_result = None
        wz_9_result = None
        wz_10_result = None
        wz_11_result = None
        wz_12_result = None
        wz_13_result = None
        wz_14_result = None
        wz_15_result = None
        wz_16_result = None
        wz_17_result = None
        wz_18_result = None
        wz_19_result = None
        wz_20_result = None
        wz_21_result = None
        wz_22_result = None
        wz_23_result = None
        wz_24_result = None
        wz_25_result = None
        wz_26_result = None
        wz_27_result = None

        # Metrics WZ-4, WZ-5, WZ-6, WZ-7, and WZ-8
        advisory_speed_limit, time_first_msg_received, wz_4_result, wz_5_result, wz_6_result, wz_7_result, wz_8_result = get_workzone_TCM_data(bag)
        
        # Convert advisory speed limit from WZ-19 to m/s for future metric evaluations
        advisory_speed_limit = advisory_speed_limit * 0.44704 # Conversion from mph to m/s

        lanelets_in_geofence = get_geofence_lanelets(bag, time_enter_geofence, advisory_speed_limit)
        
        # Metric WZ-1 and WZ-5
        closed_lanelets = [12459, 1245999, 1245998]
        wz_1_result, wz_5_result = check_geofence_in_initial_route(bag, closed_lanelets)

        # Metric WZ-2
        wz_2_result = check_steady_state_before_first_received_message(bag, time_test_start_engagement, time_first_msg_received, original_speed_limit)

        # Metric WZ-9
        wz_9_result = check_percentage_successful_spat_msg(bag, time_test_start_engagement, time_test_end_engagement)

        # Metric WZ-10
        wz_10_result = check_duration_between_spat_msg_below_max_duration(bag, time_test_start_engagement, time_test_end_engagement)

        stop_bar_location = [-36.8063, 323.251] # Map [x,y] coordinate of stop bar. Hardcoded; same value for every test
        #end_geofence_location = [0,0] # Map [x,y] coordinate of the end of the geofence. Hardcoded; same value for every test

        if bag_file in red_light_bag_files:
            wz_11_result, time_last_accel, time_stopped = check_deceleration_for_red_light(bag, time_test_start_engagement)

            dist_rear_axle_to_front_bumper = 4.0 # TODO: Obtain improved measurement for this distance for each vehicle
            wz_12_result = check_stop_location_for_red_light(bag, stop_bar_location, dist_rear_axle_to_front_bumper, time_test_start_engagement, time_last_accel, time_stopped)
            #generate_speed_plot(bag)

            wz_13_result = check_acceleration_time_after_green_light(bag, time_stopped)

            wz_14_result = check_acceleration_after_stop(bag, time_stopped)

            print("WZ-15 N/A (Red Light Bag)")
            print("WZ-16 N/A (Red Light Bag)")

        else:
            print("WZ-11 N/A (Green Light Bag)")
            print("WZ-12 N/A (Green Light Bag)")
            print("WZ-13 N/A (Green Light Bag)")
            print("WZ-14 N/A (Green Light Bag)")


            wz_15_result, time_vehicle_at_stop_bar = check_vehicle_speed_at_green_light(bag, advisory_speed_limit, stop_bar_location, time_test_start_engagement)

            wz_16_result = check_deceleration_after_green_light(bag, time_vehicle_at_stop_bar, time_test_start_engagement)

        wz_23_result = check_steady_state_after_geofence(bag, time_test_end_engagement, original_speed_limit)

        wz_24_result = check_speed_limit_when_not_in_geofence(bag, original_speed_limit)

        wz_25_result = check_advisory_speed_limit(bag, advisory_speed_limit, original_speed_limit)

        # Get vehicle type that this bag file is from
        vehicle_name = "Unknown"
        if bag_file in black_pacifica_green_bag_files or bag_file in black_pacifica_red_bag_files:
            vehicle_name = "Black Pacifica"
        elif bag_file in ford_fusion_green_bag_files or bag_file in ford_fusion_red_bag_files:
            vehicle_name = "Ford Fusion"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        vehicle_role = "Unknown"
        if bag_file in red_light_bag_files:
            vehicle_role = "Red Light"
        elif bag_file in green_light_bag_files:
            vehicle_role = "Green Light"

        # Write simple pass/fail results to .csv file for appropriate row:
        #csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
        #                             wz_1_result, wz_2_result, wz_3_result, wz_4_result, wz_5_result, wz_6_result, wz_7_result,
        #                             wz_8_result, wz_9_result, wz_10_result, wz_11_result, wz_12_result, wz_13_result, wz_14_result, 
        #                             wz_15_result, wz_16_result, wz_17_result, wz_18_result, wz_19_result, wz_20_result,
        #                             wz_21_result, wz_22_result, wz_23_result, wz_24_result, wz_25_result])
        
    #sys.stdout = orig_stdout
    #text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()