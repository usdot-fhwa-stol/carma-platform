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

# Usage:
# python3.7 analyze_freight_workzone_rosbags.py <path to folder containing Work Zone Use Case .bag files>

def generate_speed_plot(bag, time_start_engagement, time_end_engagement):
    # Speed command: /hardware_interface/arbitrated_speed_commands: msg.speed (m/s)
    # True Speed:    /hardware_interface/vehicle/twist: msg.twist.linear.x (m/s)

    # Get the true vehicle speed (m/s) and the associated time with each data point
    first = True
    true_vehicle_speed_times = []
    true_vehicle_speeds = []
    # Note: This topic name assumes a pacmod controller is being used (freightliners or lexus)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue

        true_vehicle_speed_times.append((t-time_start).to_sec())
        true_vehicle_speeds.append(msg.twist.linear.x) # Current speed in m/s

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


    # Get the time of the first TCM received event
    received_tcm_times = []
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        if msg.tcm_v01.params.detail.choice == 5:
            received_tcm_times.append((t-time_start).to_sec())
            break

    # Get the time of all TCRs broadcasted
    broadcasted_tcr_times = []
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_geofence_request'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        broadcasted_tcr_times.append((t-time_start).to_sec())

    # Get the time of each re-route
    route_generation_times = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route'], start_time = time_start_engagement, end_time = time_end_engagement): # time_start_engagement+time_duration):
        route_generation_times.append((t-time_start).to_sec())

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot commanded vehicle speed (m/s) vs. time
    ax.plot(cmd_vehicle_speed_times, cmd_vehicle_speeds, 'g:', label='Commanded Speed (m/s)')

    # Plot true vehicle speed (m/s) vs. time
    ax.plot(true_vehicle_speed_times, true_vehicle_speeds, 'b--', label='Actual Speed (m/s)')

    # Optional: Plot a vertical bar at the time of the first received TCM
    #ax.axvline(x = received_tcm_times[0], color = 'r', label = 'First TCM Received')

    # Optional: Plot a vertical bar at the time of the first completed re-route
    #ax.axvline(x = route_generation_times[1], color = 'orange', label = "Reroute Completed")

    # Optional: Plot a vertical bar for each broadcasted TCR
    #for i in range(0, len(broadcasted_tcr_times)):
    #    if i == 0:
    #        ax.axvline(x = broadcasted_tcr_times[i], color = 'g', label = 'All TCRs Broadcasted')
    #    else:
    #        ax.axvline(x = broadcasted_tcr_times[i], color = 'g')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title("Speed (Commanded and Actual)") # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title
    plt.show() # Display the plot

    return

def generate_crosstrack_plot(bag, time_start_engagement, time_end_engagement):
    # Crosstrack Error: /guidance/route_state msg.cross_track (meters)
    total_duration = 40
    time_duration = rospy.Duration(total_duration)

    # Get the cross track error and the time associated with each data point
    first = True
    crosstrack_errors = []
    crosstrack_error_times = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement + rospy.Duration(45.0), end_time = time_end_engagement - rospy.Duration(55.0)): # time_start_engagement+time_duration):
        if first:
            time_start = t
            first = False
            continue

        crosstrack_error_times.append((t-time_start).to_sec())
        crosstrack_errors.append(msg.cross_track) # Crosstrack Error (meters)

    # Create the initial plot with the defined figure size
    fig, ax = plt.subplots(figsize=(9,5.5))

    # Plot true cross-track error (meters) vs. time
    ax.plot(crosstrack_error_times, crosstrack_errors, 'g', label='Cross-track Error (Meters)')

    # Optional: Plot a horizontal bar at a positive value for reference
    ax.axhline(y = 0.8, color = 'r', label = '+/- 0.8 Meters (For Reference)')

    # Optional: Plot a horizontal bar at a negative value for reference
    ax.axhline(y = -0.8, color = 'r')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title("Cross-track Error") # Figure Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Figure X Label
    ax.set_ylabel("Cross-track Error (meters)") # Figure Y Label
    plt.show() # Display the plot

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


###########################################################################################################
# Workzone FWZ-1: The geofenced area is a part of the initial route plan.
#
# Workzone FWZ-8: The vehicle receives a message from CC that includes the closed lane ahead. The vehicle 
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
    if (len(shortest_path_lanelets) > 1):
        original_shortest_path = shortest_path_lanelets[0]
        rerouted_shortest_path = shortest_path_lanelets[-1]

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
        print("Invalid quantity of route updates found in bag file (" + str(len(shortest_path_lanelets)) + " found, more than 1 expected)")

    # Print result statements and return success flags
    if (initial_route_includes_closed_lane):
        print("FWZ-1 succeeded; all closed lanelets " + str(closed_lanelets) + " were in the initial route")
    else:
        print("FWZ-1 failed: not all closed lanelets " + str(closed_lanelets) + " were in the initial route.")

    if (map_is_updated_for_closed_lane):
        print("FWZ-8 succeeded: no closed lanelets " + str(closed_lanelets) + " were in the re-routed route.")
    else:
        print("FWZ-8 failed: at least 1 closed lanelet " + str(closed_lanelets) + " was in the re-routed route.")

    return initial_route_includes_closed_lane, map_is_updated_for_closed_lane


###########################################################################################################
# FWZ-2: The vehicle will travel at steady-state (e.g. same lane, constant speed) for at least 5 seconds
#        before it enters the geofenced area defined in the CC message.
###########################################################################################################
def check_start_steady_state_before_geofence(bag, time_start_engagement, time_enter_geofence, original_speed_limit):
    # (m/s) Threshold offset of vehicle speed to speed limit to be considered at steady state
    threshold_speed_limit_offset = 0.89408 # 0.89408 m/s is 2 mph
    min_steady_state_speed = original_speed_limit - threshold_speed_limit_offset
    max_steady_state_speed = original_speed_limit + threshold_speed_limit_offset

    # (seconds) Minimum time between vehicle reaching steady state and and vehicle entering geofence
    min_time_between_steady_state_and_geofence = 5.0

    # Get the time that the vehicle reaches within the set offset of the speed limit (while system is engaged)
    time_start_steady_state = 0.0
    has_reached_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement):
        current_speed = msg.twist.linear.x # Current vehicle speed in m/s
        if (max_steady_state_speed >= current_speed >= min_steady_state_speed):
            has_reached_steady_state = True
            time_start_steady_state = t
            break
    
    # Check if the time the vehicle reaches steady state is more than 'min_time_between_steady_state_and_geofence' seconds before entering the geofence
    if (has_reached_steady_state):
        time_between_steady_state_and_geofence = (time_enter_geofence - time_start_steady_state).to_sec()
        if (time_between_steady_state_and_geofence >= min_time_between_steady_state_and_geofence):
            is_successful = True
            print("FWZ-2 succeeded; reached steady state " + str(time_between_steady_state_and_geofence) + " seconds before entering geofence.")
        else:
            is_successful = False
            if (time_between_steady_state_and_geofence > 0):
                print("FWZ-2 failed; reached steady state " + str(time_between_steady_state_and_geofence) + " seconds before entering geofence.")
            else:
                print("FWZ-2 failed; reached steady state " + str(-time_between_steady_state_and_geofence) + " seconds after entering geofence.")
    else:
        print("FWZ-2 failed; vehicle never reached steady state during rosbag recording.")
        is_successful = False

    return is_successful, time_start_steady_state

###########################################################################################################
# FWZ-7: The vehicle receives a message from CS that includes the new speed limit for the geofence area. 
#        The vehicle successfully processes this new speed limit.
###########################################################################################################
def check_in_geofence_speed_limits(bag, time_enter_geofence, time_exit_geofence, advisory_speed_limit):
    ms_threshold = 0.03 # Threshold that a received/processed speed limit can differ from the expected advisory speed limit

    # Check that a TrafficControlMessage was published using the correct advisory speed limit
    has_communicated_advisory_speed_limit = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        if (msg.tcm_v01.params.detail.choice == 12 and advisory_speed_limit - ms_threshold <= msg.tcm_v01.params.detail.maxspeed <= advisory_speed_limit + ms_threshold):
            has_communicated_advisory_speed_limit = True
            break
        else:
            has_communicated_advisory_speed_limit = False
            break

    # Check that lanelets travelled through within the geofence have the expected advisory speed limit applied
    time_buffer = rospy.Duration(2.0) # (Seconds) Buffer after entering geofence and before exiting geofence for which advisory speed limit is observed
    has_correct_geofence_lanelet_speed_limits = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = (time_enter_geofence+time_buffer), end_time = (time_exit_geofence-time_buffer)):
        # Failure if the current lanelet is one of the geofence lanelets and its speed limit doesn't match the advisory speed limit
        if (abs(msg.speed_limit - advisory_speed_limit) >= ms_threshold):
            print("Lanelet ID " + str(msg.lanelet_id) + " has speed limit of " + str(msg.speed_limit) + " m/s. " + \
                  "Does not match advisory speed limit of " + str(advisory_speed_limit) + " m/s.")
            has_correct_geofence_lanelet_speed_limits = False
            break
    
    if (has_communicated_advisory_speed_limit and has_correct_geofence_lanelet_speed_limits):
        print("FWZ-7 succeeded; system received and processed an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = True
    else:
        print("FWZ-7 failed; system did not receive and process an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = False

    return is_successful

###########################################################################################################
# FWZ-9: The vehicle receives a message from CS that includes info about the restricted lane ahead. 
#        The vehicle successfully processes this restricted lane information.
###########################################################################################################
def check_restricted_lane_tcm_received(bag, time_start_engagement, time_end_engagement, restricted_lanelets):
    # Check that a TrafficControlMessage was published for a closed lane that is not closed to passenger vehicles
    has_communicated_restricted_lane = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        # Evaluate a received TCM for a closed lane
        if (msg.tcm_v01.params.detail.choice == 5):

            # Determine whether the closed lane is closed to passenger vehicles
            # Note: Lane is considered restricted if it is not closed to passenger vehicles
            is_restricted_lane = True
            for value in msg.tcm_v01.params.vclasses:
                # vehicle_class 5 is passenger vehicles
                if value.vehicle_class == 5:
                    is_restricted_lane = False
            
            # Set boolean flags for this metric
            if is_restricted_lane:
                has_communicated_restricted_lane = True
                break

    # Get each set route from the bag file (includes set routes and updated/re-rerouted routes)
    shortest_path_lanelets = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route']):        
        shortest_path_lanelets.append([])
        for lanelet_id in msg.shortest_path_lanelet_ids:
            shortest_path_lanelets[-1].append(lanelet_id)

    # Verify that the final route update does not include the restricted lanelets
    map_is_updated_for_restricted_lane = False # Flag for B-11 success
    if (len(shortest_path_lanelets) > 1):
        rerouted_shortest_path = shortest_path_lanelets[-1]


        for lanelet_id in restricted_lanelets:
            if lanelet_id not in rerouted_shortest_path:
                map_is_updated_for_restricted_lane = True
            else:
                map_is_updated_for_restricted_lane = False
                break
    else:
        print("Invalid quantity of route updates found in bag file (" + str(len(shortest_path_lanelets)) + " found, more than 1 expected)")

    # Print result statements and return success flags
    is_successful = False
    if (has_communicated_restricted_lane and map_is_updated_for_restricted_lane):
        print("FWZ-9 succeeded: received restricted lane TCM and the restricted lanelets " + str(restricted_lanelets) + " were not in the re-routed route.")
        is_successful = True
    elif (has_communicated_advisory_speed_limit and not map_is_updated_for_restricted_lane):
        print("FWZ-9 failed: received restricted lane TCM but the restricted lanelets " + str(restricted_lanelets) + " were in the re-routed route.")
    else:
        print("FWZ-9 failed: CMV did not received restricted lane TCM.")

    return is_successful

###########################################################################################################
# FWZ-10: The vehicle returns an Acknowledgement message within 1 second after successfully processing 
#         a TCM from CS.
###########################################################################################################
def check_tcm_acknowledgements(bag, time_start_engagement, time_end_engagement, num_tcms):
    max_duration_before_ack = 1.0 # (seconds) Maximum duration between receiving a TCM and acknowledging it

    # Initialize array of times that each TCM is received 
    # Note: Index 0 is TCM msgnum 1, Index 1 is TCM msgnum 2, etc.
    tcm_receive_times = []
    for i in range(0, num_tcms):
        tcm_receive_times.append(None)

    # Initialize array of durations between receiving a TCM and broadcasting an acknowledgement
    # Note: Index 0 is TCM msgnum 1, Index 1 is TCM msgnum 2, etc.
    tcm_ack_durations = []
    for i in range(0, num_tcms):
        tcm_ack_durations.append(None)

    # Get the time that each TCM was received
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        msg_num = msg.tcm_v01.msgnum
        if tcm_receive_times[msg_num - 1] == None:
            tcm_receive_times[msg_num - 1] = t
        if None not in tcm_receive_times:
            print("FWZ-10 (DEBUG): Received all TCMs")
            break
    
    # Check the first positive acknowledgement time associated with each received TCM
    num = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_mobility_operation']):
        if msg.strategy == "carma3/Geofence_Acknowledgement":
            if "acknowledgement:1" in msg.strategy_params:
                msg_split = msg.strategy_params.split(':') # Split string by ':' character
                msg_num = int(msg_split[2].split(',')[0]) # Split the second elemnt of the above split by the ',' character and obtain the first element
                if tcm_ack_durations[msg_num - 1] == None:
                    duration = (t - tcm_receive_times[msg_num - 1]).to_sec()
                    tcm_ack_durations[msg_num - 1] = duration
                if None not in tcm_ack_durations:
                    print("FWZ-10 (DEBUG): Acknowledged all TCMs")
                    break

    is_successful = True
    for i in range(0, num_tcms):
        if tcm_ack_durations[i] <= max_duration_before_ack:
            print("FWZ-10 (msgnum " + str(i+1) + ") successful; TCM Ack broadcasted in " + str(tcm_ack_durations[i]) + " seconds")
        else:
            is_successful = False
            print("FWZ-10 (msgnum " + str(i+1) + ") failed; TCM Ack broadcasted in " + str(tcm_ack_durations[i]) + " seconds")
    
    return is_successful

###########################################################################################################
# FWZ-12: After the vehicle has received a message from CS with the Work Zone information, the vehicle 
#         shall successfully update its active route in less than 3 seconds to avoid the Work Zone.
###########################################################################################################
def check_reroute_duration(bag, time_start_engagement, time_end_engagement):
    # Note: Implementation assumes 1 closed lane TCM and 1 restricted lane TCM are received

    max_duration_before_reroute = 3.0 # (seconds) Maximum duration between receiving a closed/restricted lane TCM and rerouting

    # Obtain timestamps of each closed/restricted lane TCM
    closed_lane_tcm_receive_time = None
    restricted_lane_tcm_receive_time = None
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        # Evaluate a received TCM for a closed lane
        if (msg.tcm_v01.params.detail.choice == 5):

            # Determine whether the closed lane is closed to passenger vehicles
            # Note: Lane is considered restricted if it is not closed to passenger vehicles
            is_restricted_lane = True
            for value in msg.tcm_v01.params.vclasses:
                # vehicle_class 5 is passenger vehicles
                if value.vehicle_class == 5:
                    is_restricted_lane = False
            
            # Set boolean flags for this metric
            if is_restricted_lane:
                if restricted_lane_tcm_receive_time is None:
                    restricted_lane_tcm_receive_time = t
                    print("FWZ-12 (DEBUG): Received restricted lane TCM at " + str(t))
            else:
                if closed_lane_tcm_receive_time is None:
                    closed_lane_tcm_receive_time = t
                    print("FWZ-12 (DEBUG): Received closed lane TCM at " + str(t))
    
    if closed_lane_tcm_receive_time <= restricted_lane_tcm_receive_time:
        closed_lane_tcm_received_first = True

    # Get the time of each re-route
    route_generation_times = []
    first = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route']): 
        print("FWZ-12 (DEBUG): Generated route at " + str(t))
        route_generation_times.append(t)

    if closed_lane_tcm_received_first:
        duration_reroute_after_closed_lane_tcm_received = (route_generation_times[1] - closed_lane_tcm_receive_time).to_sec()
        duration_reroute_after_restricted_lane_tcm_received = (route_generation_times[2] - restricted_lane_tcm_receive_time).to_sec()
    else:
        duration_reroute_after_closed_lane_tcm_received = (route_generation_times[2] - closed_lane_tcm_receive_time).to_sec()
        duration_reroute_after_restricted_lane_tcm_received = (route_generation_times[1] - restricted_lane_tcm_receive_time).to_sec()

    is_successful = True
    if duration_reroute_after_closed_lane_tcm_received <= max_duration_before_reroute:
        print("FWZ-12 succeeded; rerouted " + str(duration_reroute_after_closed_lane_tcm_received) + " sec after receiving closed lane TCM")
    else:
        print("FWZ-12 failed; rerouted " + str(duration_reroute_after_closed_lane_tcm_received) + " sec after receiving closed lane TCM")
        is_successful = False

    if duration_reroute_after_restricted_lane_tcm_received <= max_duration_before_reroute:
        print("FWZ-12 succeeded; rerouted " + str(duration_reroute_after_restricted_lane_tcm_received) + " sec after receiving restricted lane TCM")
    else:
        print("FWZ-12 failed; rerouted " + str(duration_reroute_after_restricted_lane_tcm_received) + " sec after receiving restricted lane TCM")
        is_successful = False

    return is_successful

###########################################################################################################
# FWZ-14: The vehicle lateral velocity during a lane change remains between 0.5 m/s and 1.25 m/s.
###########################################################################################################
def check_lanechange_lateral_acceleration(bag, time_start_engagement, time_end_engagement, num_expected_lanechanges):
    return

###########################################################################################################
# FWZ-15: After changing lanes, the vehicle will achieve steady-state (i.e. truck is driving within the lane) 
#         within 5 seconds. 
###########################################################################################################
def check_lanechange_duration(bag, time_start_engagement, time_end_engagement, num_expected_lanechanges):
    max_lanechange_duration = 5.0
    crosstrack_end_lanechange = 3.2 # (meters) Distance from original lane center for lane change to be considered complete

    # Get each timestamp of the first trajectory plan point being planned by CLC
    first = True
    is_in_lanechange = False
    num = 0
    times_start_lanechange = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_start_engagement + rospy.Duration(45.0)):
        if msg.trajectory_points[0].planner_plugin_name == "StopAndWaitPlugin":
            break

        if not is_in_lanechange:
            if msg.trajectory_points[0].planner_plugin_name != "InLaneCruisingPlugin":
                is_in_lanechange = True
                times_start_lanechange.append(t)
        
        if is_in_lanechange:
            if msg.trajectory_points[0].planner_plugin_name == "InLaneCruisingPlugin":
                is_in_lanechange = False
    
    print("FWZ-15 (DEBUG): Found starting time of " + str(len(times_start_lanechange)) + " lane changes")
    lanechange_durations = []
    for time in times_start_lanechange:
        for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time):
            if abs(msg.cross_track) >= crosstrack_end_lanechange:
                lanechange_durations.append((t-time).to_sec())
                break
    
    is_successful = True
    for i in range(0, len(lanechange_durations)):
        if lanechange_durations[i] <= max_lanechange_duration:
            print("FWZ-15 (LC " + str(i + 1) + ") succeeded; lane change completed in " + str(lanechange_durations[i]) + " seconds")
        else:
            print("FWZ-15 (LC " + str(i + 1) + ") failed; lane change completed in " + str(lanechange_durations[i]) + " seconds")
            is_successful = False

    return is_successful

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
    #text_log_filename = "Results_" + str(current_time) + ".txt"
    #text_log_file_writer = open(text_log_filename, 'w')
    #sys.stdout = text_log_file_writer

    # Create .csv file to make it easier to view overview of results (the .txt log file is still used for more in-depth information):
    #csv_results_filename = "Results_" + str(current_time) + ".csv"
    #csv_results_writer = csv.writer(open(csv_results_filename, 'w'))
    #csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Test Type",
    #                                 "FWZ-1", "FWZ-2", "FWZ-3", "FWZ-4", "FWZ-5",
    #                                 "FWZ-6", "FWZ-7", "FWZ-8", "FWZ-9", "FWZ-10",
    #                                 "FWZ-11", "FWZ-12", "FWZ-13", "FWZ-14", "FWZ-15",
    #                                 "FWZ-16", "FWZ-17", "FWZ-18", "FWZ-19", "FWZ-20",
    #                                 "FWZ-21", "FWZ-22", "FWZ-23", "FWZ-24", "FWZ-25",
    #                                 "FWZ-26", "FWZ-27", "FWZ-28", "FWZ-29", "FWZ-30",
    #                                 "FWZ-31", "FWZ-32"])

    white_truck_bag_files = ["3.2A_R1_WT_04212022.bag"]

    # Concatenate all Port Drayage bag file lists into one list
    PD_bag_files = white_truck_bag_files 

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in PD_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in white_truck_bag_files:
            print("White Truck Freight Work Zone Test Case")
        else:
            print("Unknown bag file being processed.")
            
        # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
        #sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(PD_bag_files.index(bag_file) + 1) + " of " + str(len(PD_bag_files)) + ")")
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
        
        print("Spent " + str((time_test_end_engagement - time_test_start_engagement).to_sec()) + " seconds engaged.")

        original_speed_limit = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit) + " m/s")

        # Optional: Generate vehicle speed plot for the rosbag
        #generate_speed_plot(bag, time_test_start_engagement, time_test_end_engagement)

        # Optional: Generate cross-track error plot for the rosbag
        #generate_crosstrack_plot(bag, time_test_start_engagement, time_test_end_engagement)

        # Metric FWZ-1 (geofenced area is a part of the initial route)
        # Metric FWZ-8 (final route does not include the closed lanelets))
        closed_lanelets = [10801, 10802]
        fwz_1_result, fwz_8_result = check_geofence_in_initial_route(bag, closed_lanelets)

        fwz_2_result, time_start_steady_state_before_geofence = check_start_steady_state_before_geofence(bag, time_test_start_engagement, time_enter_geofence, original_speed_limit)

        advisory_speed_limit_ms = 11.176 # 11.176 m/s is 25 mph
        fwz_7_result = check_in_geofence_speed_limits(bag, time_enter_geofence, time_exit_geofence, advisory_speed_limit_ms)

        restricted_lanelets = [20801, 20802]
        fwz_9_result = check_restricted_lane_tcm_received(bag, time_test_start_engagement, time_test_end_engagement, restricted_lanelets)
        
        expected_number_tcms = 9
        fwz_10_result = check_tcm_acknowledgements(bag, time_test_start_engagement, time_test_end_engagement, expected_number_tcms)

        fwz_12_result = check_reroute_duration(bag, time_test_start_engagement, time_test_end_engagement)

        num_expected_lanechanges = 4
        fwz_15_result = check_lanechange_duration(bag, time_test_start_engagement, time_test_end_engagement, num_expected_lanechanges)
        return

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
        if bag_file in white_truck_bag_files:
            vehicle_name = "Silver Truck"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        vehicle_role = "Freight Work Zone"

        # Write simple pass/fail results to .csv file for appropriate row:
        #csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
        #                             pd_1_result_0,
        #                             pd_4_result_0, pd_4_result_3, pd_4_result_9,
        #                             ...])
        #
        
    #sys.stdout = orig_stdout
    #text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()