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
#   python3.7 analyze_freight_workzone_rosbags.py <path to folder containing Work Zone Use Case .bag files>


# Helper function: Creates a plot with 'actual speed vs. time' and 'commanded speed vs. time' 
#                  and saves it as a .png image within the local directory
def generate_speed_plot(bag, time_start_engagement, time_end_engagement, bag_file_name):
    # Speed command: /hardware_interface/arbitrated_speed_commands: msg.speed (m/s)
    # True Speed:    /hardware_interface/vehicle/twist: msg.twist.linear.x (m/s)

    # Get the true vehicle speed (m/s) and the associated time with each data point
    first = True
    true_vehicle_speed_times = []
    true_vehicle_speeds = []
    # Note: This topic name assumes a pacmod controller is being used (freightliners or lexus)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start_engagement + rospy.Duration(30), end_time = time_start_engagement + rospy.Duration(60)): # time_start_engagement+time_duration):
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
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/arbitrated_speed_commands'], start_time = time_start_engagement + rospy.Duration(30), end_time = time_start_engagement + rospy.Duration(60)): # time_start_engagement+time_duration):
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
    ax.set_title(str(bag_file_name) + " Speed (Commanded and Actual)") # Plot Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Plot X Title
    ax.set_ylabel("Vehicle Speed (m/s)") # Plot Y Title

    # Save the plot
    filename = "Speed_" + bag_file_name + ".png"
    #plt.savefig(filename, bbox_inches='tight')
    
    plt.show() # Display the plot

    return

# Helper function: Creates a plot with 'crosstrack error vs. time' and saves it as a .png image within the local directory
def generate_crosstrack_plot(bag, time_start_engagement, time_end_engagement, bag_file_name):
    # Crosstrack Error: /guidance/route_state msg.cross_track (meters)
    total_duration = 40
    time_duration = rospy.Duration(total_duration)

    # Get the cross track error and the time associated with each data point
    first = True
    crosstrack_errors = []
    crosstrack_error_times = []
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement + rospy.Duration(0.0), end_time = time_end_engagement - rospy.Duration(0.0)): # time_start_engagement+time_duration):
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
    ax.axhline(y = 0.65, color = 'r', label = '+/- 0.65 Meters (For Reference)')

    # Optional: Plot a horizontal bar at a negative value for reference
    ax.axhline(y = -0.65, color = 'r')

    plt.rc('axes', labelsize=12)  # fontsize of the axes labels
    plt.rc('legend', fontsize=10)  # fontsize of the legend text
    ax.legend(loc = 'lower right') # Location of the legend
    ax.set_title(str(bag_file_name) + " Cross-track Error") # Figure Title
    ax.set_xlabel("Time (seconds) Since Start of Engagement") # Figure X Label
    ax.set_ylabel("Cross-track Error (meters)") # Figure Y Label
    
    # Save the plot
    filename = "CrossTrack_" + bag_file_name + ".png"
    plt.savefig(filename, bbox_inches='tight')

    #plt.show() # Display the plot

    return

# Helper function: Creates a plot with 'actual steering angle vs. time' and 'commanded steering angle vs. time' 
#                  and saves it as a .png image within the local directory
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
#         a TCM from CS.F
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
    first = True
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        if first:
            t_first_tcm = t
            first = False

        msg_num = msg.tcm_v01.msgnum
        #print("Received msg_num: " + str(msg_num))
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
                    print(msg.strategy_params)
                    print("FWZ-10 (DEBUG): Acknowledging TCM msgnum " + str(msg_num) + " " + str((t - t_first_tcm).to_sec()) + " sec after receiving first TCM")
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
    else:
        closed_lane_tcm_received_first = False

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
    lanechange_lateral_movement = []
    for time in times_start_lanechange:
        first = True
        for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time):
            if first:
                lanechange_lateral_movement.append(crosstrack_end_lanechange - abs(msg.cross_track))
                first = False
            if abs(msg.cross_track) >= crosstrack_end_lanechange:
                lanechange_durations.append((t-time).to_sec())
                break
    print("FWZ-15 (DEBUG): Starting cross tracks: " + str(lanechange_lateral_movement))
    is_successful = True
    for i in range(0, len(lanechange_durations)):
        if lanechange_durations[i] <= max_lanechange_duration:
            print("FWZ-15 (LC " + str(i + 1) + ") succeeded; lane change completed in " + str(lanechange_durations[i]) + " seconds")
        else:
            print("FWZ-15 (LC " + str(i + 1) + ") failed; lane change completed in " + str(lanechange_durations[i]) + " seconds")
            is_successful = False

    return is_successful

###########################################################################################################
# FWZ-23: Upon entering the geofenced area with an advisory speed limit, the vehicle will initiate the 
#         deceleration command to the advisory speed limit within less than 1.3 seconds.
###########################################################################################################
def check_time_to_begin_deceleration(bag, time_start_engagement, time_end_engagement, advisory_speed_limit_ms):
    max_duration_before_decel = 1.3 # seconds

    speed_limit_threshold_ms = 0.05
    min_speed_limit_ms = advisory_speed_limit_ms - speed_limit_threshold_ms
    max_speed_limit_ms = advisory_speed_limit_ms + speed_limit_threshold_ms
    time_enter_geofence = None
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_end_engagement):
        # Obtain the time that the vehicle enters a lanelet with the advisory speed limit
        if (min_speed_limit_ms <= msg.speed_limit <= max_speed_limit_ms):
            time_enter_geofence = t
            break

    num_consecutive_decel_required = 10 # Arbitrarily select that topic must show vehicle slowing down for this many consecutive messages to be considered the start of deceleration
    found_start_of_decel = False
    duration_before_decel = rospy.Duration(0.0)
    time_begin_deceleration_in_geofence = rospy.Time(0.0)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_enter_geofence, end_time = time_end_engagement): # time_start_engagement+time_duration):
        time_start = t

        num_consecutive_decel = 0
        first = True
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start, end_time = time_end_engagement): # time_start_engagement+time_duration):
            if first:
                prev_speed = msg.twist.linear.x
                first = False
                continue

            speed_start_decel_ms = msg.twist.linear.x
            if msg.twist.linear.x - prev_speed < 0:
                num_consecutive_decel += 1
                if num_consecutive_decel == num_consecutive_decel_required:
                    duration_before_decel = (time_start - time_enter_geofence).to_sec()
                    time_begin_deceleration_in_geofence = t
                    found_start_of_decel = True
            else:
                break
            
            prev_speed = msg.twist.linear.x

        if found_start_of_decel:
            break
    
    is_successful = False
    if duration_before_decel < max_duration_before_decel:
        print("FWZ-23 succeeded; vehicle began decelerating " + str(duration_before_decel) + " seconds after entering geofence")
        is_successful = True
    else:
        print("FWZ-23 failed; vehicle began decelerating " + str(duration_before_decel) + " seconds after entering geofence")


    return is_successful, time_begin_deceleration_in_geofence

###########################################################################################################
# FWZ-24: The vehicle will decelerate and achieve the advisory speed limit prior to reaching the work zone.
###########################################################################################################
def check_speed_before_workzone(bag, time_start_engagement, time_end_engagement, workzone_lanelet_id, advisory_speed_limit_ms):
    speed_limit_threshold_ms = 0.89408 # 0.89408 m/s is 2 mph
    min_speed_limit_ms = advisory_speed_limit_ms - speed_limit_threshold_ms
    max_speed_limit_ms = advisory_speed_limit_ms + speed_limit_threshold_ms

    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_start_engagement, end_time = time_end_engagement):
        if msg.lanelet_id == workzone_lanelet_id:
            time_enter_workzone = t 

    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_enter_workzone, end_time = time_end_engagement): # time_start_engagement+time_duration):
        vehicle_speed_workzone_entrance_ms = msg.twist.linear.x 
        break
    
    is_successful = False
    if (min_speed_limit_ms <= vehicle_speed_workzone_entrance_ms <= max_speed_limit_ms):
        print("FWZ-24 succeeded: Vehicle travelling at " + str(vehicle_speed_workzone_entrance_ms) + " m/s when entering the workzone")
        is_successful = True
    else:
        print("FWZ-24 failed: Vehicle travelling at " + str(vehicle_speed_workzone_entrance_ms) + " m/s when entering the workzone")

    return is_successful

###########################################################################################################
# FWZ-25: Upon entering the geofenced area with an advisory speed limit, the actual trajectory to the reduced 
#         speed operations will include an acceleration section. The average deceleration over the entire 
#         section shall be no greater than 2 m/s^2.
###########################################################################################################
def check_deceleration_for_geofence(bag, time_start_engagement, time_end_engagement, time_begin_deceleration_in_geofence, advisory_speed_limit_ms):
    max_avg_deceleration = 2.0 # m/s^2
    end_decel_speed = advisory_speed_limit_ms + 1.1176 # Add 1.1176 m/s (2.5 mph) buffer to the expected end-of-deceleration speed

    # Obtain timestamp associated with the end of the deceleration section 
    # Note: This is the moment when the vehicle's speed reaches the advisory speed limit
    first = True
    time_end_decel = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_begin_deceleration_in_geofence):
        # Obtain the speed at the start of the deceleration section
        if first:
            speed_start_decel_ms = msg.twist.linear.x
            first = False

            # Print Debug Line
            speed_start_decel_mph = msg.twist.linear.x * 2.2639 # 2.2369 mph is 1 m/s
            print("FWZ-25 (DEBUG): Speed at start of deceleration section: " + str(speed_start_decel_mph) + " mph (" + str(speed_start_decel_ms) + " m/s)")
            continue

        current_speed_ms = msg.twist.linear.x
        if (current_speed_ms <= end_decel_speed):
            time_end_decel = t
            speed_end_decel_ms = current_speed_ms

            # Print Debug Line
            speed_end_decel_mph = speed_end_decel_ms * 2.2369 # 2.2369 mph is 1 m/s
            print("FWZ-25 (DEBUG): Speed at end of deceleration section: " + str(speed_end_decel_mph) + " mph (" + str(speed_end_decel_ms) + " m/s)")
            break


    # Calculate the average deceleration across the full deceleration section
    print("FWZ-25 (DEBUG): Duration between start and end of deceleration: " + str((time_end_decel-time_begin_deceleration_in_geofence).to_sec()))
    total_average_decel = (speed_start_decel_ms - speed_end_decel_ms) / (time_end_decel - time_begin_deceleration_in_geofence).to_sec()
    
    is_successful = False
    if total_average_decel <= max_avg_deceleration:
        print("FWZ-25 succeeded: average deceleration after entering geofence was " + str(total_average_decel) + " m/s^2")
        is_successful = True
    else:
        print("FWZ-25 succeeded: average deceleration after entering geofence was " + str(total_average_decel) + " m/s^2")

    return is_successful


###########################################################################################################
# FWZ-26: After exiting the geofenced area with an advisory speed limit, the vehicle will begin accelerating
#         back to the original speed limit within less than 1.3 seconds.
###########################################################################################################
def check_time_to_begin_acceleration(bag, time_enter_geofence, time_end_engagement, original_speed_limit_ms):
    max_duration_before_accel = 10.0 # seconds

    # Obtain the time that the CMV enters a lanelet with the original speed limit (this indicates it has exited the geofenced area)
    speed_limit_threshold_ms = 0.05
    min_speed_limit_ms = original_speed_limit_ms - speed_limit_threshold_ms
    max_speed_limit_ms = original_speed_limit_ms + speed_limit_threshold_ms
    time_exit_geofence = None
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_geofence, end_time = time_end_engagement):
        # Obtain the time that the vehicle enters a lanelet with the advisory speed limit
        if (min_speed_limit_ms <= msg.speed_limit <= max_speed_limit_ms):
            time_exit_geofence = t
            break

    num_consecutive_accel_required = 20 # Arbitrarily select that topic must show vehicle speeding up for this many consecutive messages to be considered the start of acceleration
    found_start_of_accel = False
    duration_before_accel = rospy.Duration(0.0)
    time_begin_acceleration_after_geofence = rospy.Time(0.0)
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_exit_geofence, end_time = time_end_engagement): # time_start_engagement+time_duration):
        time_start = t

        num_consecutive_accel = 0
        first = True
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_start, end_time = time_end_engagement): # time_start_engagement+time_duration):
            if first:
                prev_speed = msg.twist.linear.x
                first = False
                continue

            if msg.twist.linear.x - prev_speed > 0:
                num_consecutive_accel +=1
                if num_consecutive_accel == num_consecutive_accel_required:
                    duration_before_accel = (time_start - time_exit_geofence).to_sec()
                    time_begin_acceleration_after_geofence = time_start
                    found_start_of_accel = True
            else:
                break
            
            prev_speed = msg.twist.linear.x

        if found_start_of_accel:
            break

    
    is_successful = False
    if duration_before_accel <= max_duration_before_accel:
        print("FWZ-26 succeeded; vehicle began accelerating " + str(duration_before_accel) + " seconds after exiting geofence")
        is_successful = True
    else:
        print("FWZ-26 failed; vehicle began accelerating " + str(duration_before_accel) + " seconds after exiting geofence")

    return is_successful, time_begin_acceleration_after_geofence

###########################################################################################################
# FWZ-27: After exiting the geofenced area with an advisory speed limit, the actual trajectory back to 
#         normal operations will include an acceleration section. The average acceleration over the entire 
#         section shall be no less than 1 m/s^2, and the average acceleration over any 1-second portion 
#         of the section shall be no greater than 2.0 m/s^2.
###########################################################################################################
def check_acceleration_after_geofence(bag, time_begin_acceleration_after_geofence, time_end_engagement, target_speed_limit_ms):
    min_average_accel_rate = 0.0 # m/s^2
    max_one_second_accel_rate = 1.5 # m/s^2

    # Set target speed (an offset below the target speed limit, since acceleration rate will decrease after this point)
    speed_limit_offset_ms = 0.89 # 0.89 m/s is 2 mph
    target_speed_ms = target_speed_limit_ms - speed_limit_offset_ms

    # Obtain timestamp associated with the end of the acceleration section 
    # Note: This is either the first deceleration after the start of the accel section or the moment 
    #       when the vehicle's speed reaches some offset of the speed limit.
    prev_speed = 0.0
    first = True
    time_end_accel = rospy.Time()
    speed_start_accel_ms = 0.0
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_begin_acceleration_after_geofence):
        if first:
            prev_speed = msg.twist.linear.x
            speed_start_accel_ms = msg.twist.linear.x
            first = False
            continue

        delta_speed = msg.twist.linear.x - prev_speed
        current_speed_ms = msg.twist.linear.x
        if (current_speed_ms >= target_speed_ms):
            time_end_accel = t
            speed_end_accel_ms = current_speed_ms

            # Debug Line
            speed_end_accel_mph = speed_end_accel_ms * 2.2369
            print("FWZ-27 (DEBUG): Speed at end of acceleration section: " + str(speed_end_accel_mph) + " mph")
            break

        prev_speed = msg.twist.linear.x

    # Check the total average acceleration rate
    total_avg_accel = (speed_end_accel_ms - speed_start_accel_ms) / (time_end_accel-time_begin_acceleration_after_geofence).to_sec()
    #print("Total average accel: " + str(total_avg_accel) + " m/s^2")

    # Check the 1-second window averages
    # Obtain average decelation over all 1-second windows in acceleration section
    one_second_duration = rospy.Duration(1.0)
    all_one_second_windows_successful = True
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_begin_acceleration_after_geofence, end_time = (time_end_accel-one_second_duration)):
        speed_initial_ms = msg.twist.linear.x
        time_initial = t

        speed_final_ms = 0.0
        for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = (time_initial+one_second_duration)):
            speed_final_ms = msg.twist.linear.x
            t_final = t
            break
        
        one_second_accel = (speed_final_ms - speed_initial_ms) / (t_final - time_initial).to_sec()

        if one_second_accel > max_one_second_accel_rate:
            print("Failure: 1-second window accel rate was " + str(one_second_accel) + " m/s^2; end time was " + str((t_final - t_final).to_sec()) + " seconds after start of engagement.")
            all_one_second_windows_successful = False
        #else:
        #    print("Success: 1-second window has accel rate at " + str(one_second_decel) + " m/s^2")

    # Print success/failure statements
    total_acceleration_rate_successful = False
    if total_avg_accel >= min_average_accel_rate:
        print("FWZ-27 (avg accel after geofence) Succeeded; total average acceleration was " + str(total_avg_accel) + " m/s^2")
        total_acceleration_rate_successful = True
    else:
        print("FWZ-27 (avg accel at start) Failed; total average acceleration was " + str(total_avg_accel) + " m/s^2")
    
    if all_one_second_windows_successful:
        print("FWZ-27 (1-second accel windows) Succeeded; no occurrences of 1-second average acceleration above 1.5 m/s^2")
    else:
        print("FWZ-27 (1-second accel windows) Failed; at least one occurrence of 1-second average acceleration above 1.5 m/s^2")

    is_successful = False
    if total_acceleration_rate_successful and all_one_second_windows_successful:
        is_successful = True

    return is_successful

###########################################################################################################
# FWZ-29: After exiting the geofenced area, the planned route must end with the vehicle having been at 
#         steady state for at least 5 seconds. 
###########################################################################################################
def check_steady_state_after_geofence(bag, time_begin_acceleration_after_geofence, time_end_engagement, original_speed_limit_ms):
    min_time_at_steady_state = 5.0 # Seconds

    # (m/s) Threshold offset of vehicle speed to speed limit to be considered at steady state
    threshold_speed_limit_offset = 0.89408 # 0.89408 m/s is 2 mph
    min_steady_state_speed = original_speed_limit_ms - threshold_speed_limit_offset
    max_steady_state_speed = original_speed_limit_ms + threshold_speed_limit_offset

    has_reached_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle/twist'], start_time = time_begin_acceleration_after_geofence, end_time = time_end_engagement):
        if (min_steady_state_speed <= msg.twist.linear.x <= max_steady_state_speed) and not has_reached_steady_state:
            time_start_steady_state = t
            has_reached_steady_state = True
        
        if not (min_steady_state_speed <= msg.twist.linear.x <= max_steady_state_speed) and has_reached_steady_state:
            time_end_steady_state = t
            break
        elif has_reached_steady_state:
            time_end_steady_state = t
    
    if has_reached_steady_state:
        time_at_steady_state = (time_end_steady_state - time_start_steady_state).to_sec()
    else:
        time_at_steady_state = 0.0

    is_successful = False
    if time_at_steady_state >= min_time_at_steady_state:
        print("FWZ-29 succeeded: Vehicle was at steady state for " + str(time_at_steady_state) + " seconds after exiting the geofence")
        is_successful = True
    else:
        print("FWZ-29 failed: Vehicle was at steady state for " + str(time_at_steady_state) + " seconds after exiting the geofence")

    return is_successful

###########################################################################################################
# FWZ-32: The time taken between when a TCR is sent from the vehicle to a TCM is received by 
#         the vehicle shall be less than 1 second.
###########################################################################################################
def check_tcm_response_time(bag, time_enter_geofence, tcr_rx_times, tcr_rx_reqids, reqid_v2x_timestamps):
    # reqid_v2x_timestamps: 0 is reqid; 1 is tcr receive time; 2-10 are FIRST tcm tx times for msgnums 0 to 9
    tcr_reqids = []
    tcr_reqids_times = []
    num = 0
    for topic, msg, t in bag.read_messages(topics=['/message/outgoing_geofence_request']): 
        #print(str(num) + ": " + str(msg.tcr_v01.reqid))
        tcr_reqids.append([int(i) for i in msg.tcr_v01.reqid.id])
        hex_string = ""
        for i in msg.tcr_v01.reqid.id:
            if i > 16:
                hex_string += hex(i)[-2:]
            else:
                hex_string += "0" + hex(i)[-1:]          
        if hex_string in tcr_rx_reqids:
            index = tcr_rx_reqids.index(hex_string)
            try:
                min_time = min(reqid_v2x_timestamps[index][2:]) # Min timestamp of V2XHub transmitting a TCM for a msgnum of that reqid
                max_time = max(reqid_v2x_timestamps[index][2:]) # Max timestamp of V2XHub transmitting a TCM for a msgnum of that reqid
                #print("TCR " + str(num) + " broadcasted at time " + str(t.to_sec()))
                time_cmv_to_v2x = reqid_v2x_timestamps[index][1] - float(t.to_sec())
                print("TCR " + str(num) + ": " + str(hex_string) + " received by V2XHub " + str(time_cmv_to_v2x) + " sec after cmv broadcast; TCM transmitted " + str(min_time) + " to " + str(max_time) + " afterwards")
                #print("Vehicle broadcasted TCR at " + str(t.to_sec()) + ", V2XHub RSU received TCR at " + str(reqid_v2x_timestamps[index][1]))
            except:
                print("TCR " + str(num) + ": " + str(reqid_v2x_timestamps[index]))
                #print("From tcr_rx_reqids: " + str(tcr_rx_reqids[index]))
        #else:
        #    print("TCR " + str(num) + ": " + str(hex_string) + " not received by V2XHub")

        tcr_reqids_times.append(t)
        num += 1

    #print("Now TCMs received: ")
    num = 0
    tcm_reqids = []
    has_received_unknown_TCM = False

    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']): 
        reqid = [int(i) for i in msg.tcm_v01.reqid.id]
        if reqid not in tcm_reqids:
            tcm_reqids.append(reqid)
            if reqid in tcr_reqids:
                index = tcr_reqids.index(reqid)
                duration_tcr_to_tcm = (t - tcr_reqids_times[index]).to_sec()
                hex_string = ""
                for i in msg.tcm_v01.reqid.id:
                    if i > 16:
                        hex_string += hex(i)[-2:]
                    else:
                        hex_string += "0" + hex(i)[-1:]               

                print("TCM " + str(num) + " was TCR # " + str(index) + ". Time from TCR to TCM was " + str(duration_tcr_to_tcm) + " seconds. Reqid: " + str(hex_string))
            else:
                if not has_received_unknown_TCM:
                    #print("RECEIVED TCM THAT WAS NOT A BROADCASTED TCR")
                    has_received_unknown_TCM = True
            
        num += 1

# Helper function: Converts time from HH::MM::SS format of a given day to its Unix Epoch Time
# Note: This is used because rosbags use Unix Epoch Time and some V2XHub and CARMA Cloud logs use HH::MM::SS EST format
def get_system_time(time):
    hours = float(time[0:2]) - 4.0 # The '4.0' is hardcoded since CarmaCloud time is 4 hours ahead of the EST time (at least for the initial data set analyzed)
    minutes = float(time[3:5])
    seconds = float(time[6:8])
    ms = float(time[9:12])

    #system_time = 1650513600.0 # HARDCODED (use for tests conducted on April 21st, 2022; this represents the epoch time at start of that day)
    system_time = 1650600000.0 # HARDCODED (use for tests conducted on April 22nd; this represents the epoch time at start of that day)
    system_time += 60*60*hours + 60*minutes + seconds + 0.001*ms

    return system_time

# Helper function for processing TCR receive and TCM send times from a CARMA Cloud tomcat log text file
# NOTE: Function developed for April 22nd version of CARMA Cloud tomcat logs; the text log ouputs may no longer
#       match the assumptions made in this function
# Inputs:
#   Source Folder: (String) Path to folder that contains tomcat log files
#   tomcat_log_files: (String) List of file names to be processed
#   empty_val: (integer) Integer value that indicates an element in the returned lists are empty
# Outputs:
#   all_cc_data: List with each row being [Received TCR reqid, TCR receive time, duration until TCM msgnum 1 sent, duration until TCM msgnum 2 sent, etc....]
#        Note: List is in order of received TCR reqids
#   cc_tcr_rx_reqids: List of each received TCR reqid in order that CARMA Cloud received it
def process_tomcat_logs(source_folder, tomcat_log_files, empty_val):
    # Each row of all_cc_data will be [TCR reqid, TCR receive time, duration until TCM msgnum 1 sent, duration until TCM msgnum 2 sent, ....etc....]
    # Note: Populated in order of received TCR reqids; assumes 9 TCMs are sent for each TCR (this is the number used for Freight WZ testing)
    all_cc_data = []

    # List of each TCR received by CARMA Cloud in order (lowercase)
    cc_tcr_rx_reqids = []

    # Process each file contained within the provided tomcat_log_files list
    for filename in tomcat_log_files:
        print("Processing tomcat log file: " + str(filename))

        # Parse each line in the tomcat log file and update all_cc_data and cc_tcr_rx_reqids based on TCR- and TCM-related logs
        with open(str(source_folder) + "/" + filename, newline='') as f:
            while(True):
                line = f.readline()

                if not line:
                    break
                else:
                    if "TCR" in line:
                        # Process a line that describes a received TCR by CARMA Cloud (from V2XHub)

                        tcr_reqid = line[97:113]
                        time = line[7:19]
                        system_time = get_system_time(time) # Convert HH::MM::SS to Unix Epoch Time

                        # NOTE: This assumes there are 9 TCMs being sent out for this test
                        all_cc_data.append([str(tcr_reqid).lower(), system_time, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val])
                        if tcr_reqid in cc_tcr_rx_reqids:
                            print("WARNING: Duplicate TCR reqid found in tomcat log file " + str(filename) +": " + str(tcr_reqid))

                        cc_tcr_rx_reqids.append(tcr_reqid.lower())
                        #print(str(tcr_reqid.lower()) + " added to cc_tcr_rx_reqids")

                    if "TCM 404" in line:
                        # Process a line that describes a TCM sent out by CARMA Cloud (to V2XHub)

                        tcm_reqid = (line[109:125]).lower()
                        time = line[7:19]
                        system_time = get_system_time(time) # Convert HH::MM::SS to Unix Epoch Time
                        msgnum = int(line[177:178])

                        index = cc_tcr_rx_reqids.index(tcm_reqid)
                        #print("New TCM: " + str(tcm_reqid) + " msgnum " + str(msgnum) + " at time " + str(system_time) + " in response to received TCR #" + str(index))

                        if all_cc_data[index][msgnum+1] == empty_val:
                            all_cc_data[index][msgnum+1] = system_time - all_cc_data[index][1]
                        else:
                            print("Duplicate TCM: " + str(tcm_reqid) + " msgnum " + str(msgnum) + " at time " + str(system_time) + " for TCR #" + str(index))

    min_times = []
    count_below_one_sec = 0
    min_time = 9999
    max_time = 0.0
    for row in all_cc_data:
        if min(row[2:]) != empty_val:
            min_times.append(min(row[1:]))
            if min(row[2:]) < min_time:
                min_time = min(row[1:])
            if max(row[2:]) != empty_val:
                if max(row[2:]) > max_time:
                    max_time = max(row[2:])
            if min(row[2:]) <= 1.0:
                count_below_one_sec += 1
    avg_time_cc = sum(min_times) / len(min_times)
    pct_below_one_sec = (float(count_below_one_sec / len(min_times))) * 100.0

    variance = sum([((x - avg_time_cc) ** 2) for x in min_times]) / len(min_times)
    std_dev = variance ** 0.5

    print("Average duration between CARMA Cloud receiving TCR from CS and sending first TCM: " + str(avg_time_cc))
    print("Max duration: " + str(max_time))
    print("Min duration: " + str(min_time))
    print("Standard deviation: " + str(std_dev))
    print("Percentage below 1 sec: " + str(pct_below_one_sec) + "%")

    return all_cc_data, cc_tcr_rx_reqids

def process_v2x_receive_and_broadcast_logs(source_folder, tcr_receive_csv_filenames, tcm_broadcast_csv_filenames, mobop_receive_csv_filenames, cc_tcr_rx_reqids, all_cc_data, empty_val):
    # Read in V2XHub TCR Receiving Data from CSV Files
    tcr_data = []
    for filename in tcr_receive_csv_filenames:
        with open(str(source_folder) + "/rsu_tcr_receive_data/" + filename, newline='') as f:
            reader = csv.reader(f)
            tcr_data += list(reader)[1:]

    # Read in V2XHub TCM Transmission Data from CSV Files
    tcm_data = []
    for filename in tcm_broadcast_csv_filenames:
        with open(str(source_folder) + "/" + filename, newline='') as f:
            reader = csv.reader(f)
            tcm_data += list(reader)[1:]
    
    # Create merged list of relevant V2XHub TCR Receiving & TCM Broadcasting data:
    # reqid_v2x_timestamps is a list of lists: For each list, index 0 is reqid; 1 is tcr receive time; 2-10 are duration until FIRST tcm tx time for msgnums 0 to 9
    reqid_v2x_timestamps = [] 
    tcr_rx_times = [float(i[0]) for i in tcr_data[1:]] # Create list of times (in order) of each received TCR
    tcr_rx_reqids = [str(i[2]) for i in tcr_data[1:]] # Create list of reqids (in order) of each received TCR
    for i in range(0, len(tcr_rx_times)):
        # Initial creation of each list in reqid_v2x_timestamps with first two indexes set to true values; all other indices set to temp '99999'
        reqid_v2x_timestamps.append([tcr_rx_reqids[i], tcr_rx_times[i], empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val, empty_val])

    for i in range(0, len(tcm_data)):
        reqid = tcm_data[i][2]
        if reqid in tcr_rx_reqids:
            index = tcr_rx_reqids.index(reqid)
            msgnum = tcm_data[i][3]
            if reqid_v2x_timestamps[index][1 + int(msgnum)] == empty_val:
                time_tcm_broadcasted = tcm_data[i][0]
                time_tcr_received = reqid_v2x_timestamps[index][1]

                # Add duration between received TCR and this TCM msgnum broadcast to reqid_v2x_timestamps
                reqid_v2x_timestamps[index][1 + int(msgnum)] = float(time_tcm_broadcasted) - time_tcr_received

    # Initial processing of reqid_v2x_timestamps to gather time duration data for communication flow between V2XHub TCR Receive <-> V2XHub TCM Broadcast
    for row in reqid_v2x_timestamps:
        if row[0] in cc_tcr_rx_reqids:
            print("---")
            v2x_tcr_receive_time = row[1]
            cc_index = cc_tcr_rx_reqids.index(row[0])
            cc_row = all_cc_data[cc_index]
            cc_tcr_receive_time = cc_row[1]
            duration_v2x_to_cc_tcr = cc_tcr_receive_time - v2x_tcr_receive_time
            
            formatted_cc_row = [ float(elem) for elem in cc_row[1:] ]
            formatted_cc_row = [ '%.2f' % elem for elem in cc_row[1:] ]

            print("TCR Reqid " + str(row[0]) + " took " + str(duration_v2x_to_cc_tcr) + " between V2XHub RSU receiving it and CC receiving it.")

            formatted_total_rtt_row = [float(elem) for elem in row[1:]]
            formatted_total_rtt_row = [ '%.2f' % elem for elem in row[1:] ]
            print("TCR -> TCM total times V2X " + str(formatted_total_rtt_row))

            print("CC Receive to CC Send TCM: " + str(formatted_cc_row))

            # For each msgnum, subtract the duration_v2x_to_cc_tcr and the corresponding time that it was within CC
            for i in range(2, len(row)):
                row[i] = row[i] - (duration_v2x_to_cc_tcr + cc_row[i])

            formatted_v2x_row = [float(elem) for elem in row[1:]]
            formatted_v2x_row = [ '%.2f' % elem for elem in row[1:] ]

            print("V2X Receive to TCM broad: " + str(formatted_v2x_row))
        else:
            print("TCR Reqid " + str(row[0]) + " not received by Carma Cloud")
            continue
    
    # Process received MobilityOperation  TCM acknowledgement messages
    mo_data = []
    for filename in mobop_receive_csv_filenames:
        with open(str(source_folder) + "/rsu_mobop_receive_data/" + filename, newline='') as f:
            reader = csv.reader(f)
            mo_data += list(reader)[1:]
    
    #[reqid, num_broadcast_1, num_broadcast_2, num_broadcast_3, ..]
    #[reqid, time_receive_1, time_receive_2, time_receive_3, ...]
    ack_v2x_timestamps = []
    tcm_v2x_tx_before_ack_counts = []
    tcm_v2x_tx_after_ack_counts = []
    tcm_v2x_tx_times = []
    tcm_v2x_tx_frequencies = []
    for i in range(0, len(reqid_v2x_timestamps)):
        ack_v2x_timestamps.append([reqid_v2x_timestamps[i][0], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        tcm_v2x_tx_before_ack_counts.append([reqid_v2x_timestamps[i][0], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        tcm_v2x_tx_after_ack_counts.append([reqid_v2x_timestamps[i][0], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        tcm_v2x_tx_times.append([reqid_v2x_timestamps[i][0], [], [], [], [], [], [], [], [], []])
        tcm_v2x_tx_frequencies.append([reqid_v2x_timestamps[i][0], 0, 0, 0, 0, 0, 0, 0, 0, 0])

    for i in range(1, len(mo_data)):
        try:
            reqid = mo_data[i][3]
            msgnum = mo_data[i][4]
            time_ack = float(mo_data[i][0])
            index = tcr_rx_reqids.index(reqid)
            if ack_v2x_timestamps[index][int(msgnum)] == 0:
                ack_v2x_timestamps[index][int(msgnum)] = time_ack
        except:
            continue

    # Now create a [reqid, num_broadcast_tcm1_before_ack, num_broadcast_tcm2_before_ack, ...]
    for i in range(0, len(tcm_data)):
        # Check if TCM reqid matches a received TCR req id
        if tcm_data[i][2] in tcr_rx_reqids:
            index = tcr_rx_reqids.index(tcm_data[i][2])
            msgnum = tcm_data[i][3]
            time = float(tcm_data[i][0]) # Time that V2XHub RSU broadcasted TCM

            # If TCM was broadcasted before the time of the received TCM Ack, increase TCM tx number for that msgnum
            tcm_v2x_tx_times[index][int(msgnum)].append(time)

            # No ACK was ever received
            if ack_v2x_timestamps[index][int(msgnum)] == 0:
                tcm_v2x_tx_before_ack_counts[index][int(msgnum)] += 1

            # TCM was broadcasted before ACK was received
            elif time < ack_v2x_timestamps[index][int(msgnum)]:
                tcm_v2x_tx_before_ack_counts[index][int(msgnum)] += 1

            # TCM was broadcasted after ACK was received
            else:
                tcm_v2x_tx_after_ack_counts[index][int(msgnum)] += 1

    # Calculate the TCM tx frequencies for each TCM reqid's msgnum:
    total_freq = 0.0
    min_freq = 9999999
    max_freq = 0.0
    freq_count = 0
    for i in range(0, len(tcm_v2x_tx_frequencies)):
        for msgnum in range(1, 10):
            tx_count = tcm_v2x_tx_before_ack_counts[i][msgnum] + tcm_v2x_tx_after_ack_counts[i][msgnum]
            tx_time_diffs = 0.0
            for k in range(1, len(tcm_v2x_tx_times[i][msgnum])):
                tx_time_diffs += tcm_v2x_tx_times[i][msgnum][k] - tcm_v2x_tx_times[i][msgnum][k-1]
            if tx_count >= 2:
                tcm_v2x_tx_frequencies[i][msgnum] = 1.0 / (float(tx_time_diffs) / float(tx_count-1))
                total_freq += tcm_v2x_tx_frequencies[i][msgnum]
                freq_count += 1

                if tcm_v2x_tx_frequencies[i][msgnum] < min_freq:
                    min_freq = tcm_v2x_tx_frequencies[i][msgnum]
                if tcm_v2x_tx_frequencies[i][msgnum] > max_freq:
                    max_freq = tcm_v2x_tx_frequencies[i][msgnum]

    print("Quantity TCM msgnum pubs after ACK first received for that msgnum: ")
    num = 0
    for i in tcm_v2x_tx_after_ack_counts:
        if max(i[1:]) >= 0:
            print("Before ACKs received: " + str(tcm_v2x_tx_before_ack_counts[num]))
            print("After ACKs received: " + str(i))
            print("TCM tx frequencies: " + str(tcm_v2x_tx_frequencies[num]))
        num += 1
    
    avg_freq_tcm_tx = total_freq / float(freq_count)
    print("Average frequency of TCM broadcasts: " + str(avg_freq_tcm_tx))
    print("Min frequency: " + str(min_freq))
    print("Max frequency: " + str(max_freq))

    return tcr_rx_times, tcr_rx_reqids, reqid_v2x_timestamps

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

    white_truck_bag_files = ["Silver_Truck_April20-April27/_2022-04-22-13-50-06.bag",
                            "Silver_Truck_April20-April27/_2022-04-22-13-41-54.bag",
                            "Silver_Truck_April20-April27/_2022-04-22-14-11-26.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-15-38-11.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-16-50-46.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-17-10-20.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-17-44-17.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-18-54-03.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-19-06-33.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-21-16-09.bag",
                            "Silver_Truck_April20-April27/_2022-04-25-21-29-52.bag"]

    
    tomcat_log_files = ["carmacloud-20220422.txt"]

    # In this script, any list element with a value that matches 'empty_val' is ignored
    empty_val = 99999

    # Process the CARMA Cloud tomcat log files to produce the two following data structures
    #   all_cc_data: List with each row being [Received TCR reqid, TCR receive time, duration until TCM msgnum 1 sent, duration until TCM msgnum 2 sent, etc....]
    #        Note: List is in order of received TCR reqids
    #   cc_tcr_rx_reqids: List of each received TCR reqid in order that CARMA Cloud received it
    all_cc_data, cc_tcr_rx_reqids = process_tomcat_logs(source_folder, tomcat_log_files, empty_val)

    tcr_receive_csv_filenames = ["merged-tcr-rx-0421-to-0425.csv"]
    tcm_broadcast_csv_filenames = ["merged-tcm-tx-0421-to-0425.csv"]
    mobop_receive_csv_filenames = ["merged-mobop-rx-0421-to-0425.csv"]
    tcr_rx_times, tcr_rx_reqids, reqid_v2x_timestamps = process_v2x_receive_and_broadcast_logs(source_folder, tcr_receive_csv_filenames, tcm_broadcast_csv_filenames, mobop_receive_csv_filenames, cc_tcr_rx_reqids, all_cc_data, empty_val)

    # Concatenate all Freight Work Zone bag file lists into one list
    WZ_bag_files = white_truck_bag_files 

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in WZ_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in white_truck_bag_files:
            print("White Truck Freight Work Zone Test Case")
        else:
            print("Unknown bag file being processed.")
            
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
        #if (not found_geofence_times):
        #    print("Could not find geofence entrance and exit times in bag file.")
        #    continue

        # Get the rosbag times associated with the starting engagement and ending engagement for the Basic Travel use case test
        print("Getting engagement times at " + str(datetime.datetime.now()))
        time_test_start_engagement, time_test_end_engagement, found_test_times = get_test_case_engagement_times(bag, time_enter_geofence, time_exit_geofence)
        print("Started engagement at " + str(time_test_start_engagement.to_sec()))
        print("Ended engagement at " + str(time_test_end_engagement.to_sec()))
        if (not found_test_times):
            print("Could not find test case engagement start and end times in bag file.")
            continue
        
        print("Spent " + str((time_test_end_engagement - time_test_start_engagement).to_sec()) + " seconds engaged.")

        original_speed_limit_ms = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit_ms) + " m/s")

        # OPTIONAL: Generate Plots:

        # Generate vehicle speed plot for the rosbag
        #generate_speed_plot(bag, time_test_start_engagement, time_test_end_engagement, bag_file)

        # Generate cross-track error plot for the rosbag
        #generate_crosstrack_plot(bag, time_test_start_engagement, time_test_end_engagement, bag_file)

        # Initialize success flags
        fwz_1_result = False
        fwz_2_result = False
        fwz_3_result = False
        fwz_4_result = False
        fwz_5_result = False
        fwz_6_result = False
        fwz_7_result = False
        fwz_8_result = False
        fwz_9_result = False
        fwz_10_result = False
        fwz_11_result = False
        fwz_12_result = False
        fwz_13_result = False
        fwz_14_result = False
        fwz_15_result = False
        fwz_16_result = False
        fwz_17_result = False
        fwz_18_result = False
        fwz_19_result = False
        fwz_20_result = False
        fwz_21_result = False
        fwz_22_result = False
        fwz_23_result = False
        fwz_24_result = False
        fwz_25_result = False
        fwz_26_result = False
        fwz_27_result = False
        fwz_28_result = False
        fwz_29_result = False
        fwz_30_result = False
        fwz_31_result = False
        fwz_32_result = False

        # Metric FWZ-1 (geofenced area is a part of the initial route)
        # Metric FWZ-8 (final route does not include the closed lanelets))
        closed_lanelets = [10801, 10802]
        fwz_1_result, fwz_8_result = check_geofence_in_initial_route(bag, closed_lanelets)

        
        fwz_2_result, time_start_steady_state_before_geofence = check_start_steady_state_before_geofence(bag, time_test_start_engagement, time_enter_geofence, original_speed_limit_ms)

        advisory_speed_limit_ms = 11.176 # 11.176 m/s is 25 mph
        fwz_7_result = check_in_geofence_speed_limits(bag, time_enter_geofence, time_exit_geofence, advisory_speed_limit_ms)

        restricted_lanelets = [20801, 20802]
        fwz_9_result = check_restricted_lane_tcm_received(bag, time_test_start_engagement, time_test_end_engagement, restricted_lanelets)
        
        expected_number_tcms = 9
        fwz_10_result = check_tcm_acknowledgements(bag, time_test_start_engagement, time_test_end_engagement, expected_number_tcms)

        fwz_12_result = check_reroute_duration(bag, time_test_start_engagement, time_test_end_engagement)

        num_expected_lanechanges = 4
        fwz_15_result = check_lanechange_duration(bag, time_test_start_engagement, time_test_end_engagement, num_expected_lanechanges)
        
        fwz_23_result, time_begin_deceleration_in_geofence = check_time_to_begin_deceleration(bag, time_test_start_engagement, time_test_end_engagement, advisory_speed_limit_ms)
        
        workzone_lanelet_id = 30801
        fwz_24_result = check_speed_before_workzone(bag, time_test_start_engagement, time_test_end_engagement, workzone_lanelet_id, advisory_speed_limit_ms)

        fwz_25_result = check_deceleration_for_geofence(bag, time_test_start_engagement, time_test_end_engagement, time_begin_deceleration_in_geofence, advisory_speed_limit_ms)

        fwz_26_result, time_begin_acceleration_after_geofence = check_time_to_begin_acceleration(bag, time_begin_deceleration_in_geofence, time_test_end_engagement, original_speed_limit_ms)

        fwz_27_result = check_acceleration_after_geofence(bag, time_begin_acceleration_after_geofence, time_test_end_engagement, original_speed_limit_ms)

        fwz_29_result = check_steady_state_after_geofence(bag, time_begin_acceleration_after_geofence, time_test_end_engagement, original_speed_limit_ms)

        check_tcm_response_time(bag, time_enter_geofence, tcr_rx_times, tcr_rx_reqids, reqid_v2x_timestamps)
        

        # Get vehicle type that this bag file is from
        vehicle_name = "Unknown"
        if bag_file in white_truck_bag_files:
            vehicle_name = "White Truck"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        vehicle_role = "Freight Work Zone"

        # Write simple pass/fail results to .csv file for appropriate row:
        #csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
        #                             fwz_1_result, fwz_2_result, fwz_3_result, fwz_4_result, fwz_5_result,
        #                             fwz_6_result, fwz_7_result, fwz_8_result, fwz_9_result, fwz_10_result,
        #                             fwz_11_result, fwz_12_result, fwz_13_result, fwz_14_result, fwz_15_result,
        #                             fwz_16_result, fwz_17_result, fwz_18_result, fwz_19_result, fwz_20_result,
        #                             fwz_21_result, fwz_22_result, fwz_23_result, fwz_24_result, fwz_25_result,
        #                             fwz_26_result, fwz_27_result, fwz_28_result, fwz_29_result, fwz_30_result,
        #                             fwz_31_result, fwz_32_result])

        
    #sys.stdout = orig_stdout
    #text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()