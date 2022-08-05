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
#   python3.7 analyze_TIM_test_rosbags.py <path to folder containing TIM test case .bag files>

# Helper Function: Get times associated with the system entering the geofence and exiting the geofence
def get_geofence_entrance_and_exit_times(bag):
    # Initialize geofence entrance and exit times
    time_enter_active_geofence = rospy.Time()
    time_exit_active_geofence = rospy.Time()

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
# Note: Assumes that all lanelets in the route share the same speed limit prior to the first geofence TIM message being processed.
def get_route_original_speed_limit(bag, time_test_start_engagement):
    # Initialize the return variable
    original_speed_limit = 0.0

    # Find the speed limit associated with the first lanelet when the system first becomes engaged
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_test_start_engagement):
        original_speed_limit = msg.speed_limit
        break

    return original_speed_limit

# Helper function: Begin with the time that the vehicle exits the active geofence according to  
#                  /guidance/active_geofence topic. This adjusts the time to be based on /guidance/route_state
#                  in order to be more accurate
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
    found_test_case_engagement_times = False
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
    if not found_test_case_engagement_times:
        if time_last_engaged >= time_exit_active_geofence:
            time_stop_engagement = time_last_engaged
            found_test_case_engagement_times = True
    
    return time_start_engagement, time_stop_engagement, found_test_case_engagement_times

###########################################################################################################
# TIM B-1: Geofenced area is a part of the initial route plan.
# TIM B-11: The vehicle receives a message from the CM vehicle that includes the closed lane ahead. The 
#           vehicle processes this closed lane information.
# RWM B-1: Geofenced area is a part of the initial route plan.
# RWM B-11: The vehicle receives a message from CC that includes the closed lane ahead. The vehicle processes 
#           this closed lane information.
###########################################################################################################
def check_geofence_route_metrics(bag, closed_lanelets):
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
        print("B-1 succeeded; all closed lanelets " + str(closed_lanelets) + " were in the initial route")
    else:
        print("B-1 failed: not all closed lanelets " + str(closed_lanelets) + " were in the initial route.")

    if (map_is_updated_for_closed_lane):
        print("B-11 succeeded: no closed lanelets " + str(closed_lanelets) + " were in the re-routed route.")
    else:
        print("B-11 failed: at least 1 closed lanelet " + str(closed_lanelets) + " was in the re-routed route.")

    return initial_route_includes_closed_lane, map_is_updated_for_closed_lane

###########################################################################################################
# TIM B-10: The vehicle receives a message from the CM vehicle that includes the new speed limit ahead. 
#           The vehicle processes this new speed limit.
# RWM B-10: The vehicle receives a message from CARMA Cloud that includes the new speed limit ahead. 
#           The vehicle processes this new speed limit.
###########################################################################################################
def check_in_geofence_speed_limits(bag, time_enter_geofence, time_exit_geofence, advisory_speed_limit):
    # Check that a TrafficControlMessage was published using the correct advisory speed limit
    advisory_speed_limit_mph = advisory_speed_limit / 0.44704 # Conversion from m/s to mph
    has_communicated_advisory_speed_limit = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        if (msg.tcmV01.params.detail.choice == 12 and msg.tcmV01.params.detail.maxspeed == advisory_speed_limit_mph):
            has_communicated_advisory_speed_limit = True
            break
        elif (msg.tcmV01.params.detail.choice == 12 and msg.tcmV01.params.detail.maxspeed != advisory_speed_limit_mph):
            has_communicated_advisory_speed_limit = False
            break

    epsilon = 0.01
    has_correct_geofence_lanelet_speed_limits = True
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_geofence, end_time = time_exit_geofence):
        # Failure if the current lanelet is one of the geofence lanelets and its speed limit doesn't match the advisory speed limit
        if (abs(msg.speed_limit - advisory_speed_limit) >= epsilon):
            print("Lanelet ID " + str(msg.lanelet_id) + " has speed limit of " + str(msg.speed_limit) + " m/s. " + \
                  "Does not match advisory speed limit of " + str(advisory_speed_limit) + " m/s.")
            has_correct_geofence_lanelet_speed_limits = False
            break
    
    if (has_communicated_advisory_speed_limit and has_correct_geofence_lanelet_speed_limits):
        print("B-10 succeeded; system processed an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = True
    else:
        print("B-10 failed; system did not process an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = False

    return is_successful

###########################################################################################################
# RWM B-28: The information communicated by CC is closed area, what the new speed limit is in the closed 
#           area, and what the minimum gap limits are in the closed area.
###########################################################################################################
def get_RWM_TCM_data(bag):
    # Check that TCM Messages are received for closed lane, max speed, and minimum headway (minimum gap)
    has_closed_lane = False
    has_advisory_speed = False
    has_minimum_gap = False
    minimum_gap = 0.0
    advisory_speed = 0.0
    time_first_msg_received = rospy.Time()
    is_successful = False
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_geofence_control']):
        time_first_msg_received = t
        if msg.tcmV01.params.detail.choice == 5 and msg.tcmV01.params.detail.closed == 1:
            has_closed_lane = True
        elif msg.tcmV01.params.detail.choice == 13:
            has_minimum_gap = True
            minimum_gap = msg.tcmV01.params.detail.minhdwy
        elif msg.tcmV01.params.detail.choice == 12:
            has_advisory_speed = True
            advisory_speed = msg.tcmV01.params.detail.maxspeed

        if has_closed_lane and has_advisory_speed and has_minimum_gap:
            print("TCM Messages Received: Closed Lane; Advisory Speed: " + str(advisory_speed) +  \
                " mph; Minimum Gap: " + str(minimum_gap) + " meters")
            is_successful = True
            break
    
    # Print out route state for each new lanelet:
    id = 0
    print("/guidance/route_state lanelet change times:")
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state']):
        if msg.lanelet_id != id:

            print("Time: " + str(t.to_sec()) + "; Lanelet: " + str(msg.lanelet_id) + "; Speed Limit: " + str(msg.speed_limit))
            id = msg.lanelet_id

    
    return minimum_gap, advisory_speed, time_first_msg_received, is_successful

###########################################################################################################
# TIM B-28: The information communicated by the CM vehicle is closed area, and what the new 
#           speed limit and gap limits are in the closed area.
###########################################################################################################
# Example Params: "lat:39.23371506,lon:-77.96981812,downtrack:1,uptrack:80,min_gap:6,
#                  advisory_speed:15,event_reason:MOVE OVER LAW,event_type:CLOSED"
def get_TIM_mobility_operation_data(bag):
    # Initialize the return variables
    minimum_gap = 0.0
    advisory_speed = 0.0
    event_type = ""
    time_first_msg_received = rospy.Time()
    has_correct_data = False

    # Parse the first received TIM use case Mobility Operation message
    for topic, msg, t in bag.read_messages(topics=['/message/incoming_mobility_operation']):
        #print(msg.strategy)
        if (msg.strategy == "carma3/Incident_Use_Case"):
            time_first_msg_received = t
            print("Received carma3/Incident_Use_Case strategy_params: " + str(msg.strategy_params))

            # Parse the strategy_params string:
            params = (msg.strategy_params).split(',')

            minimum_gap_text = params[4].split(':')
            advisory_speed_text = params[5].split(':')
            event_type_text = params[7].split(':')

            if (minimum_gap_text[0] == "min_gap"):
                minimum_gap = float(minimum_gap_text[1])
                has_correct_data = True
            else:
                has_correct_data = False
                break

            if (advisory_speed_text[0] == "advisory_speed"):
                advisory_speed = float(advisory_speed_text[1])
                has_correct_data = True
            else:
                has_correct_data = False
                break

            if (event_type_text[0] == "event_type" and event_type_text[1] == "CLOSED"):
                event_type = event_type_text[1]
                has_correct_data = True
            else:
                has_correct_data = False
                break
            
            # Only parse the first TIM Mobility Operation message
            break
    
    if (has_correct_data):
        print("B-28 succeeded; TIM MobilityOperation message received with advisory speed limit " + str(advisory_speed) + " mph, " \
            + "minimum gap " + str(minimum_gap) + " meters, and \"CLOSED\" event type.")
        is_successful = True
    else:
        print("B-28 failed; no TIM MobilityOperation message received with all required information (advisory speed limit, minimum gap, " \
            + " and \"CLOSED\" event type.")
        is_successful = False

    return minimum_gap, advisory_speed, event_type, time_first_msg_received, is_successful

###########################################################################################################
# TIM B-2: Amount of time that the vehicle is going at a steady state (e.g. same lane, constant speed) 
#          before it receives any messages.
# RWM B-2: Amount of time that the vehicle is going at steady state (e.g. same lane, constant speed) before 
#          it receives the first CC message.
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
            print("B-2 succeeded; reached steady state " + str(time_between_steady_state_and_msg) + " seconds before receiving first TIM or TCM message.")
        else:
            is_successful = False
            if (time_between_steady_state_and_msg > 0):
                print("B-2 failed; reached steady state " + str(time_between_steady_state_and_msg) + " seconds before receiving first TIM or TCM message.")
            else:
                print("B-2 failed; reached steady state " + str(-time_between_steady_state_and_msg) + " seconds after receiving first TIM or TCM message.")
    else:
        print("B-2 failed; vehicle never reached steady state during rosbag recording.")
        is_successful = False

    return is_successful

###########################################################################################################
# TIM B-13: Ending point of the trajectory associated with lane merging, relative to the beginning of the geofence.
# RWM B-13: Ending point of the trajectory associated with lane merging, relative to the beginning of the geofence.
###########################################################################################################
def check_lane_merge_before_geofence(bag, time_start_engagement, time_enter_geofence):
    # (feet) Maximum distance that lane merge trajectory can end before geofence entrance
    max_distance_from_geofence = 50.0 
    # (feet) Minimum distance that lane merge trajectory can end before geofence entrance
    min_distance_from_geofence = 0.0

    """
    # DEBUG Statements: Maneuver Plan Output
    for topic, msg, t in bag.read_messages(topics=['/guidance/final_maneuver_plan'], start_time = time_start_engagement, end_time = time_enter_geofence):
        print("************************")
        print("Maneuvers in plan: " + str(len(msg.maneuvers)))
        for maneuver in msg.maneuvers:
            if (maneuver.type == 0):
                print("Lane Keeping: " + str(maneuver.lane_following_maneuver.start_time) + " to " + str(maneuver.lane_following_maneuver.end_time))
                print("Speed: " + str(maneuver.lane_following_maneuver.start_speed) + " to " + str(maneuver.lane_following_maneuver.end_speed))
                print("Distance: " + str(maneuver.lane_following_maneuver.start_dist) + " to " + str(maneuver.lane_following_maneuver.end_dist))
                print("Lanelet: " + str(maneuver.lane_following_maneuver.lane_id))
            elif (maneuver.type == 1):
                print("Lane Change: " + str(maneuver.lane_change_maneuver.start_time) + " to " + str(maneuver.lane_change_maneuver.end_time))
                print("Distance: " + str(maneuver.lane_change_maneuver.start_dist) + " to " + str(maneuver.lane_change_maneuver.end_dist))
                print("Speed: " + str(maneuver.lane_change_maneuver.start_speed) + " to " + str(maneuver.lane_change_maneuver.end_speed))
                print("Lanelet: " + str(maneuver.lane_change_maneuver.starting_lane_id) + " to " + str(maneuver.lane_change_maneuver.ending_lane_id))
            elif (maneuver.type == 5):
                print("StopAndWait: " + str(maneuver.stop_and_wait_maneuver.start_time) + " to " + str(maneuver.stop_and_wait_maneuver.end_time))
    """

    # Get the location (in Map Frame) of the start of the geofence
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_enter_geofence):
        start_geofence_x = msg.pose.position.x
        start_geofence_y = msg.pose.position.y
        break

    # For each trajectory plan (prior to entering the geofence) that contains the end of a lane-change, check that
    #     the point associated with the end of that lane merge is 0-50 feet prior to the beginning of the geofence:
    # Note: Currently assumes the only lane change prior to the geofence is the one immediately before the start of the geofence
    total_distance_from_geofence = 0.0 # feet
    count_success_lane_merge_point = 0
    count_fail_lane_merge_point = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_start_engagement, end_time = time_enter_geofence):
        #print("*************************************************")
        total_distance = 0.0
        has_found_lane_merge_end_point = False
        prev_point = msg.trajectory_points[0]
        for i in range(1, len(msg.trajectory_points)): 
            current_point = msg.trajectory_points[i]

            # Calculations for Debug Statement
            distance_between_lane_change_points = ((msg.trajectory_points[i].x - prev_point.x)**2 + (msg.trajectory_points[i].y - prev_point.y)**2) ** 0.5
            dt = msg.trajectory_points[i].target_time - prev_point.target_time
            if (dt.to_sec() <= 0.001):
                prev_point = current_point
                continue
            current_pt_speed = distance_between_lane_change_points / dt.to_sec() # Speed in m/s
            total_distance += distance_between_lane_change_points
            #print(str(current_point.planner_plugin_name) + ": " + str(current_point.target_time) + ", speed: " + str(current_pt_speed) + " m/s, distance: " +str(total_distance))


            # If previous point was planned by a lane change plugin and current point was not, the previous point was the final lane merge point
            if ((prev_point.planner_plugin_name == "UnobstructedLaneChangePlugin" or prev_point.planner_plugin_name == "CooperativeLaneChangePlugin") and \
                (current_point.planner_plugin_name != "UnobstructedLaneChangePlugin" and current_point.planner_plugin_name != "CooperativeLaneChangePlugin")):
                
                # Get end of lane merge coordinates (In Map Frame)
                end_lane_merge_x = prev_point.x
                end_lane_merge_y = prev_point.y
                has_found_lane_merge_end_point = True
            
            # Otherwise, if current point was planned by a lane change plugin and is the last trajectory plan point, it is the final lane merge point
            elif ((current_point.planner_plugin_name == "UnobstructedLaneChangePlugin" or current_point.planner_plugin_name == "CooperativeLaneChangePlugin") and \
                   i == len(msg.trajectory_points) - 1):

                # Get end of lane merge coordinates (In Map Frame)
                end_lane_merge_x = current_point.x
                end_lane_merge_y = current_point.y
                has_found_lane_merge_end_point = True

            # Evaluate the distance from the end of the lane merge to the start of the geofence if the end point has been found
            if (has_found_lane_merge_end_point):
                distance_to_geofence_meters = ((end_lane_merge_x - start_geofence_x)**2 + (end_lane_merge_y - start_geofence_y)**2)**0.5 # In meters
                distance_to_geofence_feet = distance_to_geofence_meters * 3.28 # Conversion from meters to feet

                if (max_distance_from_geofence >= distance_to_geofence_feet >= min_distance_from_geofence):
                    count_success_lane_merge_point += 1
                    #print("Successful lane-merge trajectory; ends " + str(distance_to_geofence_meters) + " meters before geofence.")
                else:
                    count_fail_lane_merge_point += 1 # Point was not on a preceding lanelet
                    #print("Failed lane-merge trajectory; ends " + str(distance_to_geofence_meters) + " meters before geofence.")

                total_distance_from_geofence += distance_to_geofence_feet
                break

            # Update previous point
            prev_point = msg.trajectory_points[i]

            """
                # Check that the lane merge end point is within 0-50 feet before geofence start AND in a preceding lanelet
                # TODO: Add this logic back in once route state bug is fixed and the current lanelet can be identified
                if (max_distance_from_geofence >= distance_to_geofence_feet >= min_distance_from_geofence):
                    end_lane_merge_lanelet_id = tpp.lane_id
                    
                    # Check that the end-of-lane-merge lanelet ID precedes the start-of-geofence lanelet ID in the shortest path:
                    for i in range(0, len(rerouted_shortest_path)):
                        if (end_lane_merge_lanelet_id == rerouted_shortest_path[i]):
                            end_lane_merge_idx = i
                        if (start_geofence_lanelet_id == rerouted_shortest_path[i]):
                            start_geofence_idx = i
                    
                    if (end_lane_merge_idx < start_geofence_idx):
                        # TODO: Track the successful end-of-lane-merge points
                        count_success_lane_merge_point += 1
                        print("Successful lane-merge trajectory; ends " + str(distance_to_geofence_meters) + " meters before geofence.")
                    else:
                        # TODO: Track the unsuccessful end-of-lane-merge points
                        count_fail_lane_merge_point += 1 # Point was not on a preceding lanelet
                        print("Failed lane-merge trajectory; ends " + str(distance_to_geofence_meters) + " meters before geofence.")
                """

    count_traj_plans_with_lane_merge = count_success_lane_merge_point + count_fail_lane_merge_point

    if count_traj_plans_with_lane_merge > 0:
        average_distance_from_geofence = total_distance_from_geofence / float(count_traj_plans_with_lane_merge)

    is_successful = False
    if (count_traj_plans_with_lane_merge == 0):
        print("B-13 failed; found no trajectories with a lane merge prior to entering the geofence.")
        is_successful = False
    elif (count_fail_lane_merge_point == 0):
        print("B-13 succeeded; succeeded on " + str(count_success_lane_merge_point) + " of " + str(count_traj_plans_with_lane_merge) + " lane-merge trajectory plans." \
            + " Average lane merge end point distance from start of geofence was " + str(average_distance_from_geofence) + " feet.")
        is_successful = True   
    else:
        print("B-13 failed; succeeded on " + str(count_success_lane_merge_point) + " of " + str(count_traj_plans_with_lane_merge) + " lane-merge trajectory plans." \
            + " Average lane merge end point distance from start of geofence was " + str(average_distance_from_geofence) + " feet.")
        is_successful = False

    return is_successful  

###########################################################################################################
# TIM B-15: The actual trajectory to prepare for the geofence will include a deceleration section and 
#           the average deceleration amount shall be no less than 1 m/s^2.
# RWM B-15: The actual trajectory to prepare for the geofence will include a deceleration section and 
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
            print("B-15 succeeded; average deceleration above 1.0 m/s^2 occurred before geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = True
        elif average_deceleration >= min_average_deceleration and not deceleration_completed_before_geofence:
            print("B-15 failed; average deceleration above 1.0 m/s^2 occurred after geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
        elif average_deceleration <= min_average_deceleration and deceleration_completed_before_geofence:
            print("B-15 failed; a deceleration below 1.0 m/s^2 occurred before the geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
        elif average_deceleration <= min_average_deceleration and not deceleration_completed_before_geofence:
            print("B-15 failed; a deceleration below 1.0 m/s^2 occurred after the geofence entrance: " + str(average_deceleration) + " m/s^2")
            is_successful = False
    elif has_found_start_of_decel:
        print("B-15 failed; did not find end of deceleration phase")
        is_successful = False
    else:
        print("B-15 failed; did not find start of deceleration phase")
        is_successful = False

    return is_successful

###########################################################################################################
# TIM B-20: After leaving the geofenced area, the planned trajectory for a lane change back to the 
#           original lane will start within the first 30 feet.
# RWM B-20:  After leaving the geofenced area, the planned trajectory for a lane change back to the 
#           original lane will start within the first 30 feet.
###########################################################################################################
def check_lane_change_after_geofence(bag, time_exit_geofence):
    max_distance_from_geofence_end = 30.0 # (feet) Max distance from end of geofence for first lane change point
    min_distance_from_geofence_end = 0.0 # (feet) Minimum distance from end of geofence for first lane change point
    is_successful = False

    # Get the location (in Map Frame) of the end of the geofence
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_exit_geofence):
        exit_geofence_x = msg.pose.position.x
        exit_geofence_y = msg.pose.position.y
        break

    # Get the position of the first planned lane change trajectory plan point after exiting the geofence (if one exists)
    has_lane_change = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_exit_geofence):
        # Get the first lane change point's position
        for tpp in msg.trajectory_points:
            if(tpp.planner_plugin_name == "UnobstructedLaneChangePlugin" or tpp.planner_plugin_name == "CooperativeLaneChangePlugin"):
                start_lane_change_x = tpp.x
                start_lane_change_y = tpp.y
                has_lane_change = True
                break
        
        # First trajectory plan with a lane change has been found, can exit loop
        if(has_lane_change):
            break

    # If a trajectory plan with a lane change has been found, find the distance between the start of the lane change and the end of the geofence
    if(has_lane_change):
        distance_from_geofence_end_meters = ((start_lane_change_x - exit_geofence_x)**2 + (start_lane_change_y - exit_geofence_y)**2) ** 0.5
        distance_from_geofence_end_feet = distance_from_geofence_end_meters * 3.28 # Conversion from meters to feet

        # If the distance between the geofence exit and the start of the lane change meets the metric criteria, set the 'is_successful' flag
        if(max_distance_from_geofence_end >= distance_from_geofence_end_feet >= min_distance_from_geofence_end):
            is_successful = True

    # Print success/failure statement and return success flag
    if (is_successful):
        print("B-20 succeeded; lane change began " + str(distance_from_geofence_end_feet) + " feet after exiting the geofence")
    else:
        if (has_lane_change):
            print("B-20 failed; lane change began " + str(distance_from_geofence_end_feet) + " feet after exiting the geofence")
        else:
            print("B-20 failed; no lane change was found after exiting the geofence.")

    return is_successful

###########################################################################################################
# TIM B-21: The actual trajectory will start calling for acceleration back to the original speed limit 
#           no more than 30 feet away from the end of the geo-fenced area.
# RWM B-21: The actual trajectory will start calling for acceleration back to the original speed limit 
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
            print("B-21 succeeded; vehicle acceleration began " + str(distance_start_accel_after_geofence) + " feet after exiting the geofence.")
            is_successful = True
        else:
            print("B-21 failed; vehicle acceleration began " + str(distance_start_accel_after_geofence) + " feet after exiting the geofence.")
            is_successful = False
    else:
        print("B-21 failed; no vehicle acceleration occurred after exiting the geofence.")
        is_successful = False

    return is_successful


###########################################################################################################
# TIM B-22: The actual trajectory back to normal operations will include an acceleration portion and 
#           the average acceleration over the entire acceleration time shall be no less than 1 m/s^2.
# RWM B-22: The actual trajectory back to normal operations will include an acceleration portion and 
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
# TIM B-23: The planned route must end with the CP vehicle having been at steady state, after all other 
#           maneuvers, for at least 10 seconds.
# RWM B-23: The planned route must end with the CP vehicle having been at steady state, after all other 
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
        print("B-23 succeeded; system reached continuous steady state for " + str(max_steady_state_duration) + " seconds after geofence-triggered maneuvers.")
        is_successful = True
    else:
        if has_steady_state:
            print("B-23 failed; system reached continuous steady state for " + str(steady_state_duration) + " seconds after geofence-triggered maneuvers. " \
                + " At least " + str(min_steady_state_time) + " seconds required.")
            is_successful = False
        if not has_steady_state:
            print("B-23 failed; system did not reach steady state after geofence-triggered maneuvers.")
            is_successful = False

    return is_successful

###########################################################################################################
# TIM B-24: The entire scenario will satisfy all previous criteria using any of the speeds given here for 
#           the "regular speed limit" (i.e. the speed limit when not in the geo-fence). (25 mph)
# RWM B-24: The entire scenario will satisfy all previous criteria using any of the speeds given here for 
#           the "regular speed limit" (i.e. the speed limit when not in the geo-fence). (25 mph)
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
        print("B-24 failed; speed limit was not always " + str(original_speed_limit) + " m/s before OR after the geofence.")
        is_successful = False
    elif(not is_correct_speed_limit_before_geofence):
        print("B-24 failed; speed limit was not always " + str(original_speed_limit) + " m/s before the geofence.")
        is_successful = False
    elif(not is_correct_speed_limit_after_geofence):
        print("B-24 failed; speed limit was not always " + str(original_speed_limit) + " m/s after the geofence.")
        is_successful = False
    else:
        print("B-24 succeeded; speed limit was always " + str(original_speed_limit) + " m/s before AND after the geofence.")
        is_successful = True

    return is_successful

###########################################################################################################
# TIM B-25: The speed limit given by the CM vehicle shall be 10 mph less 
#           than the regular speed limit.
# RWM B-25: The speed limit given by CARMA Cloud shall be 10 mph less 
#           than the regular speed limit.
###########################################################################################################
def check_advisory_speed_limit(bag, time_start_engagement, advisory_speed_limit, original_speed_limit):
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
        print("B-25 succeeded; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")
        is_successful = True
    else:
        print("B-25 failed; received advisory speed limit " + str(advisory_speed_limit) + " m/s (" + str(advisory_speed_limit_mph) \
            + " mph), which is not " + str(speed_limit_offset) + " m/s (" + str(speed_limit_offset_mph) + " mph) below the original speed limit " \
               + "of " + str(original_speed_limit) + " m/s (" + str(original_speed_limit_mph) + " mph)")       
        is_successful = False
    
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
                                 "B-1 Result", "B-2 Result", "B-3 Result", "B-4 Result", "B-5 Result", "B-6 Result", 
                                 "B-7 Result", "B-8 Result", "B-9 Result", "B-10 Result","B-11 Result", "B-12 Result", 
                                 "B-13 Result", "B-14 Result", "B-15 Result", "B-16 Result", "B-17 Result", "B-18 Result", 
                                 "B-19 Result", "B-20 Result", "B-21 Result", "B-22 Result", "B-23 Result", "B-24 Result", 
                                 "B-25 Result", "B-26 Result", "B-27 Result", "B-28 Result"])
    
    # Create list of TIM Black Pacifica bag files to be processed
    TIM_black_pacifica_bag_files = [] 
    
    # Create list of TIM Ford Fusion bag files to be processed
    TIM_ford_fusion_bag_files = ["_2021-06-24-18-03-35_down-selected.bag",
                                "_2021-06-24-18-14-20_down-selected.bag",
                                "_2021-06-24-18-20-00_down-selected.bag",
                                "_2021-06-24-18-26-11_down-selected.bag",
                                "_2021-06-24-18-31-59_down-selected.bag"]
    
    # Create list of TIM Blue Lexus bag files to be processed
    TIM_blue_lexus_bag_files = [] 

    # Concatenate all TIM bag files into one list
    TIM_bag_files = TIM_black_pacifica_bag_files + TIM_ford_fusion_bag_files + TIM_blue_lexus_bag_files

    # Create list of RWM Black Pacifica bag files to be processed
    RWM_black_pacifica_bag_files = ["_2021-06-23-13-10-28_down-selected.bag",
                                    "_2021-06-23-13-20-05_down-selected.bag",
                                    "_2021-06-23-13-42-56_down-selected.bag",
                                    "_2021-06-23-13-52-29_down-selected.bag",
                                    "_2021-06-23-14-02-22_down-selected.bag",
                                    "_2021-06-23-14-17-32_down-selected.bag"] 
    
    # Create list of RWM Ford Fusion bag files to be processed
    RWM_ford_fusion_bag_files = ["_2021-06-22-19-22-48_down-selected.bag",
                                 "_2021-06-22-19-29-24_down-selected.bag",
                                 "_2021-06-22-19-38-00_down-selected.bag",
                                 "_2021-06-22-19-45-31_down-selected.bag",
                                 "_2021-06-22-19-53-39_down-selected.bag",
                                 "_2021-06-24-17-04-56_down-selected.bag"]
    
    # Create list of RWM Blue Lexus bag files to be processed
    RWM_blue_lexus_bag_files = [] 

    # Concatenate all RWM bag files into one list
    RWM_bag_files = RWM_black_pacifica_bag_files + RWM_ford_fusion_bag_files + RWM_blue_lexus_bag_files

    TIM_and_RWM_bag_files = TIM_bag_files + RWM_bag_files

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in TIM_and_RWM_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        if bag_file in TIM_bag_files:
            print("TIM Test Case")
        elif bag_file in RWM_bag_files:
            print("RWM Test Case")
        
        # Print processing progress to terminal (all other print statements are re-directed to outputted .txt file):
        sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(TIM_and_RWM_bag_files.index(bag_file) + 1) + " of " + str(len(TIM_and_RWM_bag_files)) + ")")
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

        # Get the rosbag times associated with the starting engagement and ending engagement for the TIM use case test (15-25 seconds)
        print("Getting engagement times at " + str(datetime.datetime.now()))
        time_test_start_engagement, time_test_end_engagement, found_test_times = get_test_case_engagement_times(bag, time_enter_geofence, time_exit_geofence)
        print("Got engagement times at " + str(datetime.datetime.now()))
        if (not found_test_times):
            print("Could not find test case engagement start and end times in bag file.")
            continue
        
        # Debug Statements
        print("Engagement starts at " + str(time_test_start_engagement))
        print("Entered Geofence at " + str(time_enter_geofence))
        print("Exited Geofence at " + str(time_exit_geofence))
        print("Engagement ends at " + str(time_test_end_engagement))
        print("Time spent in geofence: " + str((time_exit_geofence - time_enter_geofence).to_sec()) + " seconds")
        print("Time spent engaged: " + str((time_test_end_engagement - time_test_start_engagement).to_sec()) + " seconds")

        # Pre-processed data for data analysis:
        closed_lanelets = [24078]
        print("Assuming Closed Lanelets: " + str(closed_lanelets))

        original_speed_limit = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit) + " m/s")

        # Update the exit geofence time to be based off of /guidance/route_state for improved accuracy
        time_exit_geofence = adjust_geofence_exit_time(bag, time_exit_geofence, original_speed_limit)
        print("Adjusted geofence exit time (based on /guidance/route_state): " + str(time_exit_geofence))

        # Initialize results 
        b_1_result = None
        b_2_result = None
        b_4_result = None
        b_10_result = None
        b_11_result = None
        b_13_result = None
        b_15_result = None
        b_20_result = None
        b_21_result = None
        b_22_result = None
        b_23_result = None
        b_24_result = None
        b_25_result = None
        b_28_result = None

        # Metrics B-28
        if bag_file in TIM_bag_files:
            minimum_gap, advisory_speed_limit, event_type, time_received_first_msg, b_28_result = get_TIM_mobility_operation_data(bag)
        elif bag_file in RWM_bag_files:
            minimum_gap, advisory_speed_limit, time_received_first_msg, b_28_result = get_RWM_TCM_data(bag)
        
        # Convert advisory speed limit from B-28 to m/s for future metric evaluations
        advisory_speed_limit = advisory_speed_limit * 0.44704 # Conversion from mph to m/s
        
        # Metrics B-1 and B-11
        b_1_result, b_11_result = check_geofence_route_metrics(bag, closed_lanelets)

        # Metrics B-10
        b_10_result = check_in_geofence_speed_limits(bag, time_test_start_engagement, time_exit_geofence, advisory_speed_limit)

        # Metrics B-2
        b_2_result = check_steady_state_before_first_received_message(bag, time_test_start_engagement, time_received_first_msg, original_speed_limit)

        # Metrics B-4 (Currently B-4 is evaluated using the speed plot from Volpe's generate_plots.py script)
        b_4_result = None

        # Metrics B-13
        b_13_result = check_lane_merge_before_geofence(bag, time_test_start_engagement, time_enter_geofence)

        # Metrics B-15
        b_15_result = check_deceleration_before_geofence(bag, time_enter_geofence, original_speed_limit, advisory_speed_limit)

        # Metrics B-20
        b_20_result = check_lane_change_after_geofence(bag, time_exit_geofence)

        # Metrics B-21
        b_21_result = check_acceleration_distance_after_geofence(bag, time_exit_geofence)

        # Metrics B-22
        b_22_result = check_acceleration_rate_after_geofence(bag, time_exit_geofence, original_speed_limit, advisory_speed_limit)

        # Metrics B-23
        b_23_result = check_steady_state_after_geofence(bag, time_exit_geofence, time_test_end_engagement, original_speed_limit)

        # Metrics B-24
        b_24_result = check_speed_limit_when_not_in_geofence(bag, time_test_start_engagement, time_enter_geofence, time_exit_geofence, time_test_end_engagement, original_speed_limit)
        
        # Metrics B-25
        b_25_result = check_advisory_speed_limit(bag, time_test_end_engagement, advisory_speed_limit, original_speed_limit)

        
        # Get vehicle type that this bag file is from
        vehicle_name = "Unknown"
        if bag_file in TIM_black_pacifica_bag_files or RWM_black_pacifica_bag_files:
            vehicle_name = "Black Pacifica"
        elif bag_file in TIM_ford_fusion_bag_files or RWM_ford_fusion_bag_files:
            vehicle_name = "Ford Fusion"
        elif bag_file in TIM_blue_lexus_bag_files or RWM_blue_lexus_bag_files:
            vehicle_name = "Blue Lexus"
        else:
            vehicle_name = "N/A"

        # Get test type that this bag file is for
        test_type = "Unknown"
        if bag_file in TIM_bag_files:
            test_type = "TIM"
        elif bag_file in RWM_bag_files:
            test_type = "RWM"

        # Write simple pass/fail results to .csv file for appropriate row:
        csv_results_writer.writerow([bag_file, vehicle_name, test_type,
                                     b_1_result, b_2_result, "B-3", b_4_result, "B-5", "B-6", "B-7", "B-8", "B-9", b_10_result, 
                                     b_11_result, "B-12", b_13_result, "B-14", b_15_result, "B-16", "B-17", "B-18", "B-19", b_20_result, 
                                     b_21_result, b_22_result, b_23_result, b_24_result, b_25_result, "B-26", "B-27", b_28_result])
        
    sys.stdout = orig_stdout
    text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()