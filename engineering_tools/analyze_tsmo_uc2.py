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

# Usage:
# python analyze_wz_rosbags.py <path to folder containing Workzone Use Case .bag files>

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

# Helper Function: Get start and end times of the period of engagement that includes the in-geofence section
# TODO remove unneeded geofence sections
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
# TSMO UC1-M1.1 : Vehicles should receive SPaT messages from RSU 
# once the vehicles enter the communication area until the vehicles depart the intersection box.
# Each vehicle processes this information and use it as inputs of the trajectory planning-related features 
# (i.e., TS, the current CARMA Platform path following feature).
# The SPaT message information shall include timestamp, cycle length, phase sequence, phase durations,
#    current phase ID, and the remaining of the current phase durantion.
#
# Expected: True
# TODO implement
###########################################################################################################

###
# Mike suggested. MAP messages received and processed and they contain intersection ids matching the expected values
###


###########################################################################################################
# TSMO UC2-M1.2 : The communication from infrastructure (RSU) to vehicle (OBU) should be within a certain frequency rates
# 
# Expected: 10 Hz ± 1 Hz for SPaT, 1 Hz ± 0.5 Hz for MA
# TODO implement
###########################################################################################################


###########################################################################################################
# TSMO UC2-M2.1 : Each vehicle should travel sequentially through the lanelets defined in its path.
# 
# Expected: True
# TODO implement. Note the expected set of lanelets changes based on the approach being used by the vehicle
###########################################################################################################

###########################################################################################################
# TSMO UC2-M2.2 : Each vehicle shall process the SPaT message 
# and adjust its speed such that it can enter the intersection box at a green phase with minimum or no stopping
# Stopping condition: speed < 0.1 meter/sec
#
# Expected: The average stopping time before the vehicle enter the intersection box < 4 sec w 
#           
# TODO implement
###########################################################################################################

###########################################################################################################
# TSMO UC2-M2.3 : Vehicles shall not enter the intersection box at yellow or red phases.
# 
# Expected: True
# TODO implement
###########################################################################################################

###########################################################################################################
# TSMO UC2-M2.4 : Each vehicle should switch away from the TS mode after entering the intersection box
# Note that the intersection box is the box per the HD map, which may not be exactly as physically seen on the road.
# Expected: True
# TODO implement
###########################################################################################################

###########################################################################################################
# TSMO UC2-M2.3 : The average deceleration\acceleration over any 1-second portion of the test time horizon
# shall be no greater than 3.0 m/s^2.
# 
# Expected: acceleration <= 3.0 m/s^2 deceleration >= -3.0 m/s^2
# TODO implement
###########################################################################################################

###
# Mike suggested. Operater override does not occur until the vehicle is at least fully inside the intersection box
###

###
# Mike suggested plot. Speed over time with matching signal state shown
###

###
# Mike suggested plots. Downtrack position over time with matching signal state shown
###

###
# Mike suggested plots. Acceleration over time with matching signal state shown
###

###
# Mike reach suggested plots. Crosstrack error overtime or just a violation count of the lane boundaries
###

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

    # TODO update this list
    csv_results_writer.writerow(["Bag Name", "Vehicle Name", "Test Type",
                                 "WZ-1 Result", "WZ-2 Result", "WZ-3 Result", "WZ-4 Result", "WZ-5 Result", "WZ-6 Result", 
                                 "WZ-7 Result", "WZ-8 Result", "WZ-9 Result", "WZ-10 Result","WZ-11 Result", "WZ-12 Result", 
                                 "WZ-13 Result", "WZ-14 Result", "WZ-15 Result", "WZ-16 Result", "WZ-17 Result", "WZ-18 Result", 
                                 "WZ-19 Result", "WZ-20 Result", "WZ-21 Result", "WZ-22 Result", "WZ-23 Result", "WZ-24 Result", "WZ-25 Result"])
    
    # TODO identify cases and assign bags to each case
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
        # TODO update bags groups
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
        sys.stdout = orig_stdout
        print("Processing bag file " + str(bag_file) + " (" + str(WZ_bag_files.index(bag_file) + 1) + " of " + str(len(WZ_bag_files)) + ")")
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


        # Get the rosbag times associated with the starting engagement and ending engagement for the Basic Travel use case test
        # TODO update to remove geofence information
        print("Getting engagement times at " + str(datetime.datetime.now()))
        time_test_start_engagement, time_test_end_engagement, found_test_times = get_test_case_engagement_times(bag, time_enter_geofence, time_exit_geofence)
        print("Got engagement times at " + str(datetime.datetime.now()))
        if (not found_test_times):
            print("Could not find test case engagement start and end times in bag file.")
            continue
        
        # Debug Statements
        print("Engagement starts at " + str(time_test_start_engagement.to_sec()))
        print("Engagement ends at " + str(time_test_end_engagement.to_sec()))
        print("Time spent engaged: " + str((time_test_end_engagement - time_test_start_engagement).to_sec()) + " seconds")

        original_speed_limit = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit) + " m/s")

        print_lanelet_entrance_times(bag, time_test_start_engagement)

        # Initialize results 
        # TODO updates results
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
        csv_results_writer.writerow([bag_file, vehicle_name, vehicle_role,
                                     wz_1_result, wz_2_result, wz_3_result, wz_4_result, wz_5_result, wz_6_result, wz_7_result,
                                     wz_8_result, wz_9_result, wz_10_result, wz_11_result, wz_12_result, wz_13_result, wz_14_result, 
                                     wz_15_result, wz_16_result, wz_17_result, wz_18_result, wz_19_result, wz_20_result,
                                     wz_21_result, wz_22_result, wz_23_result, wz_24_result, wz_25_result])
        
    sys.stdout = orig_stdout
    text_log_file_writer.close()
    return

if __name__ == "__main__":
    main()