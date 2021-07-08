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

# Usage:
# python analyze_TIM_test_rosbags.py <path to folder containing TIM test case .bag files>

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
def get_TIM_test_case_engagement_times(bag, time_enter_active_geofence, time_exit_active_geofence):
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
# TIM B-11: The map is updated to reflect the closed lane included in the received geofence.
###########################################################################################################
# TODO: Check if the received Mobility Operation msg was for a lanelet in the original 
#       shortest path using its lat/long/uptrack/downtrack
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
# TIM B-10: Lanelet(s) affected by active geofence have had their speed limits adjusted properly
###########################################################################################################
# TODO: Currently only checks that the TrafficControlMessage has been published. Once route_state bug is 
#       fixed, future runs can be validated for their in-geofence speed limits as well.
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
    
    # Uncomment this after bug is fixed that enables /guidance/route_state to be published after
    # receiving a TIM Mobility Operation message that results in a map update
    """
    # During time period that vehicle is within the geofence, check that the lanelet speed limits match the advisory speed limit
    has_updated_map_speed_limit = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_geofence): #, end_time = time_exit_geofence):
        if (msg.speed_limit == advisory_speed_limit):
            has_updated_map_speed_limit = True
        elif (msg.speed_limit != advisory_speed_limit):
            # TODO: What to do if speed limit does not match advisory speed limit?
            has_updated_map_speed_limit = False
            print("Failure: Lanelet ID " + str(msg.lanelet_id) + " has speed limit " + str(msg.speed_limit) + " m/s, Advisory Speed Limit is " + str(advisory_speed_limit) + " m/s")
            break
    """
    
    if (has_communicated_advisory_speed_limit):
        print("B-10 succeeded; system processed an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = True
    else:
        print("B-10 failed; system did not process an advisory speed limit of " + str(advisory_speed_limit) + " m/s")
        is_successful = False

    return is_successful


def get_TCM_data(bag):
    # Check that TCM Messages are received for closed lane, max speed, and minimum headway (minimum gap)
    has_closed_lane = False
    has_advisory_speed = False
    has_minimum_gap = False
    time_first_msg_received = rospy.Time()
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
        print(msg.strategy)
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
# TIM B-2: Vehicle must be at steady state for 10 seconds prior to first received MobilityOperation message
###########################################################################################################
# TODO: Implement a more robust approach to determine when vehicle has reached steady state. Current implementation
#       determines start-of-steady-state as when vehicle speed is within a certain offset of the speed limit.
def check_steady_state_before_TIM_message(bag, time_start_engagement, time_received_first_message, speed_limit):
    # (m/s) Threshold offset of vehicle speed to speed limit to be considered at steady state
    threshold_speed_limit_offset = 0.44704 # 0.44704 m/s is 1 mph
    min_steady_state_speed = speed_limit - threshold_speed_limit_offset
    max_steady_state_speed = speed_limit + threshold_speed_limit_offset

    # (seconds) Minimum time between vehicle reaching steady state and first TIM MobilityOperation message being received
    min_time_between_steady_state_and_msg = 10.0

    # Get the time that the vehicle reaches within the set offset of the speed limit (while system is engaged)
    time_start_steady_state = 0.0
    has_reached_steady_state = False
    for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_start_engagement):
        current_speed = msg.speed * 0.277777 # Conversion from kph to m/s
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

def get_route_original_speed_limit(bag, time_test_start_engagement):
    original_speed_limit = 0.0
    for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_test_start_engagement):
        original_speed_limit = msg.speed_limit
        break

    return original_speed_limit

###########################################################################################################
# TIM B-4: After steady state, Trajectory Plan is +/- 2 mph of speed limit during lane keeping on straightaways
# TODO: Implement checks for sections where the road geometry will cause speed to decrease outside of speed threshold,
#       currently a trajectory plan will fail if it is at steady state and then decelerates due to a curve.
###########################################################################################################
def check_lane_keeping_steady_state_speed(bag, time_start_engagement, time_enter_geofence, time_exit_geofence, original_speed_limit, advisory_speed_limit):
    # (m/s) Threshold offset of trajectory point speed to the speed limit
    threshold_speed_limit_offset = 0.89408 # 0.89408 m/s is 2 mph
    min_steady_state_speed = original_speed_limit - threshold_speed_limit_offset
    max_steady_state_speed = original_speed_limit + threshold_speed_limit_offset

    # Check lane-keeping trajectory point speeds while vehicle is at steady state 
    # Note: Assumes trajectory has reached steady state when the current vehicle speed is within a threshold offset of the speed limit
    is_steady_state = False
    is_first_steady_state_trajectory = True
    count_successful_traj = 0
    count_failed_traj = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_start_engagement, end_time = time_enter_geofence):

        # Check if initial speed is within steady state speed threshold and that it's planned by InLaneCruisingPlugin (lane-keeping)
        if ((max_steady_state_speed >= msg.initial_longitudinal_velocity >= min_steady_state_speed) and msg.trajectory_points[0].planner_plugin_name == "InLaneCruisingPlugin"): 

            # Track the time of first steady state trajectory plan for debug statements
            if (is_first_steady_state_trajectory):
                time_first_steady_state_point = t
                is_first_steady_state_trajectory = False
            
            # Evaluate the whole lane-keeping portion of trajectory plan to ensure that it maintains steady state speeds
            is_steady_state = True
            is_first_trajectory_point = True
            for tpp in msg.trajectory_points:
                if is_first_trajectory_point:
                    prev_point = tpp
                    is_first_trajectory_point = False
                    continue

                # Evaluate the speed associated with the next point if next point is planned by InLaneCruisingPlugin (lane-keeping)
                if (tpp.planner_plugin_name == "InLaneCruisingPlugin"):
                    # Speed calculation: Speed = Distance / Time
                    distance = ((tpp.x - prev_point.x)**2 + (tpp.y - prev_point.y)**2) ** 0.5 # meters
                    dt = (tpp.target_time -prev_point.target_time).to_sec() # seconds
                    speed = distance / dt # m/s

                    # Update previous point
                    prev_point = tpp

                    # Check if point speed is within the steady state speed threshold
                    if (max_steady_state_speed >= speed >= min_steady_state_speed):
                        continue
                    else:
                        # Debug Statements
                        #print("Lane-keeping trajectory plan point outside of speed limit threshold at target time " + str(tpp.target_time))
                        #print("Speed Limit: " + str(original_speed_limit) + " m/s")
                        #print("Point Speed: " + str(speed) + " m/s")
                        #print("Time duration of continuous steady state trajectory points: " + str((tpp.target_time - time_first_steady_state_point).to_sec()) + " seconds")
                        is_steady_state = False
                        break

                # Stop evaluating this trajectory plan if the next point is not planned by InLaneCruisingPlugin (lane-keeping)
                else:
                    is_steady_state = False
                    break  
            
            if is_steady_state:
                count_successful_traj += 1
            else:
                count_failed_traj += 1
            
    # TODO: Add logic for after geofence; update print statements with this information

    # Print success/failure statement and return success flag
    count_total_traj = count_successful_traj + count_failed_traj
    if (count_total_traj == 0):
        print("B-4 failed; no trajectories were published.")
        is_successful = False
    elif(count_failed_traj >= 0):
        print("B-4 failed; steady state trajectory speed remained +/- 2 mph of speed limit during lane keeping on " \
            + str(count_successful_traj) + " of " + str(count_total_traj) + " trajectories.")
        is_successful = False
    else:
        print("B-4 succeeded; steady state trajectory speed remained +/- 2 mph of speed limit during lane keeping on " \
            + str(count_successful_traj) + " of " + str(count_total_traj) + " trajectories.")
        is_successful = True
    
    return is_successful

###########################################################################################################
# TIM B-13: End point of lane-merge trajectory plan is 0 to 50 feet before start of geofence
###########################################################################################################
def check_lane_merge_before_geofence(bag, time_start_engagement, time_enter_geofence):
    # (feet) Maximum distance that lane merge trajectory can end before geofence entrance
    max_distance_from_geofence = 50.0 
    # (feet) Minimum distance that lane merge trajectory can end before geofence entrance
    min_distance_from_geofence = 0.0

    # Get the lanelet associated with the start of the geofence
    # TODO: Re-introduce this logic once /guidance/route_state bug is fixed
    #for topic, msg, t in bag.read_messages(topics=['/guidance/route_state'], start_time = time_enter_active_geofence):
    #    start_geofence_lanelet_id = msg.lanelet_id
    #    break

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
    average_distance_from_geofence = total_distance_from_geofence / float(count_traj_plans_with_lane_merge)

    if (count_traj_plans_with_lane_merge == 0):
        print("B-13 failed; found no trajectories with a lane merge prior to entering the geofence.")
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
# TIM B-15: The planned trajectory to prepare for the geofence will include a deceleration section and 
#           the average deceleration amount shall be no less than 1 m/s^2.
###########################################################################################################
# TODO: Currently checks for deceleration only; must be updated to check whether vehicle actually 
#       reaches the target advisory speed limit prior to geofence
def check_deceleration_before_geofence(bag, time_start_engagement, time_enter_geofence):
    threshold_distance_to_geofence = 5.0 # Threshold in meters for trajectory plan point intersecting the geofence
    threshold_deceleration = 1 # Threshold for expected average deceleration in m/s^2
    # TODO: Tune this threshold_speed_decrease_to_geofence
    threshold_speed_decrease_to_geofence = 2.2352 # Threshold in m/s for trajectory speed decrease from start of plan to the geofence start (5 mph)
                                                  #           to be considered a deceleration
    threshold_current_speed_above_advisory_speed = 2.2352 # (m/s) If initial trajectory speed is at least this amount greater than the advisory
                                                          #       speed, then a deceleration point should have been found

    # Get the location (in Map Frame) of the start of the geofence
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_enter_geofence):
        start_geofence_x = msg.pose.position.x
        start_geofence_y = msg.pose.position.y
        break

    count_success_traj_decel = 0 
    count_fail_traj_decel = 0
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_start_engagement, end_time = time_enter_geofence):
        # Debug Statements
        #print("*******************************")
        #print("New Trajectory Plan; Size : " + str(len(msg.trajectory_points)) + "; Start: " + str(t))

        # First pass through trajectory plan to check if it intercepts the geofence start (if it does, save its speed)
        intercepts_geofence = False
        prev_point = msg.trajectory_points[0]
        for i in range(1, len(msg.trajectory_points)):

            # Get the distance between this point and the start of the geofence (in meters)
            distance_to_geofence_meters = ((msg.trajectory_points[i].x - start_geofence_x)**2 + (msg.trajectory_points[i].y - start_geofence_y)**2)**0.5 # In meters
            
            # If the trajectory plan point is within the set threshold distance of the geofence, get its speed
            if(distance_to_geofence_meters <= threshold_distance_to_geofence):
                # Get the trajectory plan speed at this geofence interception:
                distance = ((msg.trajectory_points[i].x - prev_point.x)**2 + (msg.trajectory_points[i].y - prev_point.y)**2) ** 0.5
                dt = msg.trajectory_points[i].target_time - prev_point.target_time

                # Debug Statements
                #print("Distance to geofence: " + str(distance_to_geofence_meters) + " meters")
                #print("Previous point time: " + str(prev_point.target_time.to_sec()) + "; " + str(prev_point.planner_plugin_name))
                #print("Current point time: " + str(msg.trajectory_points[i].target_time.to_sec()) + "; i=" + str(i) + "; " + str(msg.trajectory_points[i].planner_plugin_name))
                #print("Points distance: " + str(distance) + " meters")
                #print("dt: " + str(dt.to_sec()))

                # Current bug in trajectory plan; consecutive points can have same target time
                if (dt.to_sec() <= 0.01):
                    # Debug Statement
                    #print("Skipping trajectory. Traj point dt is " + str(dt.to_sec()))
                    continue
                
                speed_geofence_intercept = distance / dt.to_sec() # Speed in m/s

                # Debug Statement
                #print("Geofence Intercept Speed: " + str(speed_geofence_intercept) + " m/s")

                intercept_geofence_idx = i
                intercepts_geofence = True
                break
            
            # Update Previous Point:
            prev_point = msg.trajectory_points[i]

        # Second pass through trajectory plan to evaluate its deceleration leading up to the geofence
        if (intercepts_geofence):

            # Get the speed difference between the plan's initial speed and the point that intercepts the geofence
            delta_speed_to_geofence = msg.initial_longitudinal_velocity - speed_geofence_intercept # Speed difference in m/s
            
            # If there is a decrease in speed (beyond a threshold), evaluate the plan's deceleration leading up to the geofence
            # Note: Assumes that if there is not a large enough speed decrease up to the geofence, then the system 
            #       has already come close to the advisory speed limit
            # TODO: Make more robust. This approach will not work if the peak speed is in the middle of the trajectory plan
            if (delta_speed_to_geofence >= threshold_speed_decrease_to_geofence):

                # Find the trajectory point at which deceleration begins.
                # Note: Assumes deceleration begins at the first occurrence of the trajectory plan passing the deceleration threshold
                # TODO: Create a more robust approach to determine when deceleration has begun
                prev_point = msg.trajectory_points[0]
                prev_point_speed = msg.initial_longitudinal_velocity
                for i in range(1, intercept_geofence_idx + 1):
                    # Get speed of the current trajectory point
                    distance = ((msg.trajectory_points[i].x - prev_point.x)**2 + (msg.trajectory_points[i].y - prev_point.y)**2) ** 0.5
                    dt = msg.trajectory_points[i].target_time - prev_point.target_time
                    current_pt_speed = distance / dt.to_sec() # Speed in m/s

                    # Get the deceleration of the current trajectory point
                    current_pt_decel = (current_pt_speed - prev_point_speed) / dt
                    
                    # If this point passes the deceleration threshold, compute average deceleration from it to the geofence intersection
                    if (current_pt_decel >= threshold_deceleration):
                        average_decel = (prev_point_speed - speed_geofence_intercept) / (msg.trajectory_points[intercept_geofence_idx].target_time - prev_point.target_time)
                        if (average_decel >= threshold_deceleration):
                            count_success_traj_decel += 1
                            break
                        else:
                            count_fail_traj_decel += 1

                    # Update previous point
                    prev_point = msg.trajectory_points[i]
                    prev_point_speed = current_pt_speed
            
            # Only count as a failure if the initial trajectory speed is greater than a configured threshold above the advisory speed
            else:
                if (msg.initial_longitudinal_velocity >= threshold_current_speed_above_advisory_speed):
                    count_fail_traj_decel += 1

    # Get total number of trajectories that intercepted the start of the geofence
    count_traj_plans_intercept_geofence = count_success_traj_decel + count_fail_traj_decel

    # Determine success of metric from test data (fails if failed on more than 0 trajectory plans)
    if (count_traj_plans_intercept_geofence == 0):
        print("B-15 failed; no trajectory plans that intercept the geofence area were found.")
    elif (count_fail_traj_decel == 0):
        print("B-15 succeeded; a deceleration section with an average deceleration of at least 1 m/s^2  was found on " + str(count_success_traj_decel) + " of " \
            + str(count_traj_plans_intercept_geofence) + " trajectory plans reaching the geofence.")
        is_successful = True   
    else:
        print("B-15 failed; a deceleration section with an average deceleration of at least 1 m/s^2  was found on " + str(count_success_traj_decel) + " of " \
            + str(count_traj_plans_intercept_geofence) + " trajectory plans reaching the geofence.")
        is_successful = False

    return is_successful

###########################################################################################################
# TIM B-20: After leaving the geofenced area, the planned trajectory for a lane change back to the 
#           original lane will start within the first 30 feet.
###########################################################################################################
def check_lane_change_after_geofence(bag, time_exit_geofence, time_end_engagement):
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
# TIM B-21: The planned trajectory will start calling for acceleration back to the original speed limit 
#           no more than 30 feet away from the end of the geo-fenced area.
###########################################################################################################
def check_acceleration_distance_after_geofence(bag, time_exit_geofence, time_end_engagement, time_enter_geofence, advisory_speed_limit):
    max_distance_from_geofence_end = 30.0 # (feet) Max distance from end of geofence for first lane change point
    min_distance_from_geofence_end = 0.0 # (feet) Minimum distance from end of geofence for first lane change point
    min_acceleration = 1.0 # (m/s^2) Minimum acceleration in m/s^2 for a point to be considered the start of a plan's acceleration
    #min_consecutive_accel_points = 3

    # Get the location (in Map Frame) of the end of the geofence
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_exit_geofence):
        exit_geofence_x = msg.pose.position.x
        exit_geofence_y = msg.pose.position.y
        break

    # Find the first acceleration point in the trajectory plan (if it exists)
    # Note: An acceleration point is a point with acceleration above the minimum threshold (defined by the metric criteria) along consecutive points
    # TODO: Create a more robust approach for determining when an acceleration has begun
    has_found_acceleration_point = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_enter_geofence, end_time = time_end_engagement):
        #print("********************************")
        total_distance = 0
        is_first_trajectory_point = True
        for tpp in msg.trajectory_points:
            if is_first_trajectory_point:
                prev_point = tpp
                prev_point_speed = msg.initial_longitudinal_velocity
                is_first_trajectory_point = False
                continue
            
            # Obtain current point's speed
            distance = ((tpp.x - prev_point.x)**2 + (tpp.y - prev_point.y)**2) ** 0.5
            total_distance += distance
            dt = (tpp.target_time - prev_point.target_time).to_sec()
            current_pt_speed = distance / dt # Speed in m/s

            # Obtain current point's acceleration
            current_pt_accel = (current_pt_speed - prev_point_speed) / dt
            #print(str(tpp.planner_plugin_name) + ": " + str(tpp.target_time) + ", speed: " + str(current_pt_speed) + " m/s, " +str(current_pt_accel) + " m/s^2")

            # Check if current point's acceleration meets the minimum acceleration criteria
            if (current_pt_accel >= min_acceleration and current_pt_speed > advisory_speed_limit):
                time_start_accel_after_geofence = t
                time_between_accel_and_exit = time_exit_geofence - time_start_accel_after_geofence
                has_found_acceleration_point = True

                # Get the distance between the point and the end of the geofence
                distance_from_geofence_end_meters = ((tpp.x - exit_geofence_x)**2 + (tpp.y - exit_geofence_y)**2) ** 0.5
                distance_from_geofence_end_feet = distance_from_geofence_end_meters * 3.28 # Conversion from meters to feet
                
                # Check if previous point is within min/max distance from the end of geofence (since acceleration began with previous point)
                if (time_start_accel_after_geofence.to_sec() < time_exit_geofence.to_sec()):
                    print("B-21 failed: vehicle began accelerating " + str(distance_from_geofence_end_feet) + " feet (" + \
                         str(time_between_accel_and_exit.to_sec()) + " sec) before exiting geofence.")
                    is_successful = False
                elif (max_distance_from_geofence_end >= distance_from_geofence_end_feet >= min_distance_from_geofence_end):
                    print("B-21 succeeded: vehicle began accelerating " + str(distance_from_geofence_end_feet) + " feet after exiting geofence.")
                    is_successful = True
                else:
                    print("B-21 failed: vehicle began accelerating " + str(distance_from_geofence_end_feet) + " feet after exiting geofence.")
                    is_successful = False
                
                # Break from trajectory plan loop since the first acceleration point has been found
                break

            # Update previous point
            prev_point = tpp
            prev_point_speed = current_pt_speed
        
        # Break from rostopic loop since the first acceleration point has been found
        if (has_found_acceleration_point):
            break
    
    # If no acceleration point was found in the trajectory plans after the geofence, set failure flag
    if (not has_found_acceleration_point):
        print("B-21 failed: No trajectory plan acceleration point above " + str(min_acceleration) + " m/s^2 was found after exiting the geofence.")
        is_successful = False

    return is_successful


###########################################################################################################
# TIM B-22: The planned trajectory back to normal operations will include an acceleration portion and 
#           the average acceleration over the entire acceleration time shall be no less than 1 m/s^2.
###########################################################################################################
def check_acceleration_rate_after_geofence(bag, time_exit_geofence, time_end_engagement, original_speed_limit):
    # (m/s^2) Minimum threshold of instantaneous acceleration for a point to be considered the start of the acceleration phase
    min_instantaneous_acceleration = 0.4
    # (m/s^2) Minimum threshold for average acceleration during acceleration phase in trajectory plan
    min_average_acceleration = 1.0
    # (m/s) Threshold offset from speed limit; once the start speed is within this offset of the speed limit, no more plans will be evaluated
    threshold_speed_limit_offset = 2.2352 # 2.2352 m/s is 5 mph
    # (m/s) Once the start speed of a trajectory plan surpasses this speed, no more trajectory plans will be evaluated
    max_start_speed = original_speed_limit - threshold_speed_limit_offset

    # Get the location (in Map Frame) of the end of the geofence
    for topic, msg, t in bag.read_messages(topics=['/localization/current_pose'], start_time = time_exit_geofence):
        exit_geofence_x = msg.pose.position.x
        exit_geofence_y = msg.pose.position.y
        break

    count_success_traj_accel = 0
    count_fail_traj_accel = 0
    found_first_accel_traj = False
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_exit_geofence, end_time = time_end_engagement):
        # If the trajectory start speed is above the configured max start speed, assume the vehicle has reached steady state and stop evaluating trajectory plans
        #print("*******************")
        if (msg.initial_longitudinal_velocity >= max_start_speed):
            break
        
        # First pass through trajectory plan to determine whether it has a single occurrence of acceleration above the threshold along consecutive points
        has_acceleration_phase = False # Flag that is set if first point in acceleration phase is found
        prev_point = msg.trajectory_points[0]
        prev_point_speed = msg.initial_longitudinal_velocity
        for i in range(1, len(msg.trajectory_points)):    
            # Obtain the current point's speed
            distance = ((msg.trajectory_points[i].x - prev_point.x)**2 + (msg.trajectory_points[i].y - prev_point.y)**2) ** 0.5 # Distance in meters
            dt = (msg.trajectory_points[i].target_time - prev_point.target_time).to_sec() # Time in seconds
            current_pt_speed = distance / dt # Speed in m/s

            # Obtain the current point's acceleration
            current_pt_accel = (current_pt_speed - prev_point_speed) / dt # (m/s^2)

            # Debug Statement
            #print(str(msg.trajectory_points[i].planner_plugin_name) + "; " + str(current_pt_speed) + " m/s; " + str(current_pt_accel) + " m/s^2")

            # Check if current point's acceleration is the beginning of an acceleration period
            # Note: Beginning of acceleration period is the first point that has an acceleration above the configured minimum instantaneous acceleration
            # TODO: Create a more robust approach for determining when an acceleration has begun
            if (current_pt_accel >= min_instantaneous_acceleration and not has_acceleration_phase):
                start_accel_point_idx = i - 1 # Acceleration begins with the previous point
                start_accel_point_speed = prev_point_speed
                has_acceleration_phase = True # First acceleration point has been found 
            
            # If start of acceleration phase has already been found, find the end point of the acceleration phase for average acceleration calculation
            # Note: End of acceleration phase is defined by a single occurrence of deceleration (beyond a configured threshold) along consecutive points OR the last point in plan
            # TODO: Create more robust determination of the end of the acceleration phase that is less susceptible to noise
            elif (has_acceleration_phase):   
                # If the current point's acceleration is below 0 m/s^2, the end of the acceleration phase has been found
                # TODO: Tune this value?
                if (current_pt_accel <= 0.0 or current_pt_speed >= 0.95 * original_speed_limit):
                    end_accel_point_idx = i - 1 # Acceleration ends with the previous point
                    end_accel_point_speed = prev_point_speed

                    # Break from this trajectory plan since the start and end of the acceleration phase have been found
                    break
                
                # If currently at the last point in the plan, set this point as the end of the acceleration phase
                elif (i == len(msg.trajectory_points) - 1):
                    end_accel_point_idx = i
                    end_accel_point_speed = current_pt_speed

            # Update previous point
            prev_point = msg.trajectory_points[i]
            prev_point_speed = current_pt_speed
        
        # If the full acceleration phase has been found, calculate its average acceleration
        if (has_acceleration_phase):
            delta_speed = end_accel_point_speed - start_accel_point_speed # m/s
            dt = (msg.trajectory_points[end_accel_point_idx].target_time - msg.trajectory_points[start_accel_point_idx].target_time).to_sec() # Seconds
            average_accel = delta_speed / dt # m/s^2

            if (average_accel >= min_average_acceleration):
                count_success_traj_accel += 1
                #print("Success; avg accel: " + str(average_accel))
            else:
                count_fail_traj_accel += 1
                #print("Fail; avg accel: " + str(average_accel))

        
        # Trajectory Plan did not have a single occurrence of acceleration above the configured minimum instantaneous value
        else:

            # If the previous trajectories did have an acceleration phase, this one should have had one as well
            if found_first_accel_traj:
                print("Fail; no acceleration in trajectory plan.")
                count_fail_traj_accel = True
                continue

    # Print success/failure statement and return success flag
    count_traj_plans_with_accel = count_success_traj_accel + count_fail_traj_accel
    if (count_traj_plans_with_accel == 0):
        print("B-22 failed; no trajectory plans after exiting the geofence included an acceleration phase")
        is_successful = False
    elif (count_success_traj_accel == count_traj_plans_with_accel):
        print("B-22 succeeded; an acceleration section with an average acceleration of at least 1 m/s^2  was found on " + str(count_success_traj_accel) + " of " \
            + str(count_traj_plans_with_accel) + " trajectory plans after exiting the geofence.")
        is_successful = True
    else:
        print("B-22 failed; an acceleration section with an average acceleration of at least 1 m/s^2  was found on " + str(count_success_traj_accel) + " of " \
            + str(count_traj_plans_with_accel) + " trajectory plans after exiting the geofence.")
        is_successful = False

    return is_successful

###########################################################################################################
# TIM B-23: The planned route must end with the CP vehicle having been at steady state, after all other 
#           maneuvers, for at least 10 seconds.
###########################################################################################################
def check_steady_state_after_geofence(bag, time_exit_geofence, time_end_engagement, original_speed_limit):
    # (m/s) Threshold offset from speed limit; vehicle considered at steady state when its first traj plan point is within this offset of the speed limit
    threshold_speed_limit_offset = 1.1176 # 1.1176 m/s is 2.5 mph
    # (m/s) Minimum speed to be considered at steady state
    min_steady_state_speed = original_speed_limit - threshold_speed_limit_offset
    # (m/s) Maximum speed to be considered at steady state
    max_steady_state_speed = original_speed_limit + threshold_speed_limit_offset
    # (seconds) Minimum required threshold at steady state after completing all geofence-triggered maneuvers
    min_steady_state_time = 10.0


    # Get the start time of the vehicle completing all maneuvers:
    # Note: This means the first instance of lane-keeping after exiting the geofence and completing the final lane change
    has_begun_final_lane_change = False
    time_begin_final_lane_change = rospy.Time()
    for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_exit_geofence, end_time = time_end_engagement):
        #print("First: " + str(msg.trajectory_points[0].planner_plugin_name) + ", Last: " + str(msg.trajectory_points[-1].planner_plugin_name))
        if (msg.trajectory_points[0].planner_plugin_name == "UnobstructedLaneChangePlugin" or msg.trajectory_points[0].planner_plugin_name == "CooperativeLaneChangePlugin"):
            has_begun_final_lane_change = True
            time_begin_final_lane_change = t

    # If a lane change occurred after exiting the geofence, begin steady state evaluation
    if (has_begun_final_lane_change):

        # Get the start time of the vehicle reaching steady state (if one exists)
        has_steady_state = False
        for topic, msg, t in bag.read_messages(topics=['/guidance/plan_trajectory'], start_time = time_begin_final_lane_change, end_time = time_end_engagement):
            # Vehicle has reached steady state when its first trajectory point is for lane-keeping and within threshold range of steady state speed
            if (msg.trajectory_points[0].planner_plugin_name == "InLaneCruisingPlugin" and max_steady_state_speed >= msg.initial_longitudinal_velocity >= min_steady_state_speed):
                time_first_start_steady_state = t
                has_steady_state = True
                break
        
        # If the start time of steady state was found, find the longest duration of steady state time
        has_passed_steady_state_time_threshold = False
        if (has_steady_state):
            time_start_steady_state = time_first_start_steady_state # Tracks the current steady state start time
            has_passed_steady_state_time_threshold = False
            is_at_steady_state = True
            steady_state_duration = 0.0
            for topic, msg, t in bag.read_messages(topics=['/hardware_interface/vehicle_status'], start_time = time_first_start_steady_state, end_time = time_end_engagement):
                current_speed = msg.speed * 0.277777 # Conversion from kph to m/s

                # If system is entering steady state, reset the steady state start time:
                if ((max_steady_state_speed >= current_speed >= min_steady_state_speed) and not is_at_steady_state):
                    is_at_steady_state = True
                    time_start_steady_state = t

                # If system is maintaining steady state, check if it has passed the threshold time of continuous steady state:
                elif ((max_steady_state_speed >= current_speed >= min_steady_state_speed) and is_at_steady_state):
                    steady_state_duration = (t - time_start_steady_state).to_sec()
                    if (steady_state_duration >= min_steady_state_time):
                        has_passed_steady_state_time_threshold = True
                        break       
                
                # If system has exited steady state, reset the steady state flag
                elif  ((max_steady_state_speed <= current_speed or current_speed <= min_steady_state_speed) and is_at_steady_state):
                    is_at_steady_state = False

        if (has_passed_steady_state_time_threshold):
            print("B-23 succeeded; system reached continuous steady state for more than " + str(min_steady_state_time) + " seconds after geofence-triggered maneuvers.")
            is_successful = True
        else:
            if has_steady_state:
                print("B-23 failed; system reached continuous steady state for " + str(steady_state_duration) + " seconds after geofence-triggered maneuvers. " \
                    + " At least " + str(min_steady_state_time) + " seconds required.")
                is_successful = False
            if not has_steady_state:
                print("B-23 failed; system did not reach steady state after geofence-triggered maneuvers.")
                is_successful = False
    else:
        print("B-23 failed; system never completed final lane change after exiting geofence.")
        is_successful = False

    return is_successful

###########################################################################################################
# TIM B-24: The entire scenario will satisfy all previous criteria using any of the speeds given here for 
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
    # TODO: Current bug causes route_state topic to not be published after first TIM MobilityOperation message is received
    #       As a result, a portion of the before-geofence portion cannot be evaluated
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
    # TODO: Current bug causes route_state topic to not be published after first TIM MobilityOperation message is received
    #       As a result, the after-geofence portion cannot be evaluated
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
    TIM_ford_fusion_bag_files =["_2021-06-24-18-03-35_down-selected.bag",
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
        time_test_start_engagement, time_test_end_engagement, found_test_times = get_TIM_test_case_engagement_times(bag, time_enter_geofence, time_exit_geofence)
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

        # Metrics B-28
        if bag_file in TIM_bag_files:
            minimum_gap, advisory_speed_limit, event_type, time_received_first_TIM_msg, b_28_result = get_TIM_mobility_operation_data(bag)
        elif bag_file in RWM_bag_files:
            minimum_gap, advisory_speed_limit, time_received_first_msg, b_28_result = get_TCM_data(bag)
        
        # Convert advisory speed limit from B-28 to m/s for future metric evaluations
        advisory_speed_limit = advisory_speed_limit * 0.44704 # Conversion from mph to m/s
        
        # Metrics B-1 and B-11
        b_1_result, b_11_result = check_geofence_route_metrics(bag, closed_lanelets)

        # Metrics B-10
        b_10_result = check_in_geofence_speed_limits(bag, time_test_start_engagement, time_exit_geofence, advisory_speed_limit)

        # Metrics B-2
        b_2_result = check_steady_state_before_TIM_message(bag, time_test_start_engagement, time_received_first_msg, original_speed_limit)

        # Metrics B-4
        # TODO: B-4 cannot be properly measured without knowing whether a trajectory plan deceleration is due to a curve.
        b_4_result = check_lane_keeping_steady_state_speed(bag, time_test_start_engagement, time_enter_geofence, time_exit_geofence, original_speed_limit, advisory_speed_limit)

        # Metrics B-13
        b_13_result = check_lane_merge_before_geofence(bag, time_test_start_engagement, time_enter_geofence)

        # Metrics B-15
        b_15_result = check_deceleration_before_geofence(bag, time_test_start_engagement, time_enter_geofence)

        # Metrics B-20
        b_20_result = check_lane_change_after_geofence(bag, time_exit_geofence, time_test_end_engagement)

        # Metrics B-21
        b_21_result = check_acceleration_distance_after_geofence(bag, time_exit_geofence, time_test_end_engagement, time_enter_geofence, advisory_speed_limit)

        # Metrics B-22
        b_22_result = check_acceleration_rate_after_geofence(bag, time_exit_geofence, time_test_end_engagement, original_speed_limit)

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