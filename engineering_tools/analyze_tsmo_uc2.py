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

from cProfile import label
from cmath import phase
from inspect import TPFLAGS_IS_ABSTRACT
from pprint import pprint
from random import random
import sys
import csv
from time import tzset
import matplotlib.dates as mdates
from dateutil import parser
from tkinter.tix import LabelEntry
from turtle import color, left
from click import style
import matplotlib.pyplot as plt
import pytz
import scipy as sp
import rospy
import pandas as pd
import rosbag  # To import this, run the following command: "pip install --extra-index-url https://rospypi.github.io/simple/ rospy rosbag rospkg"
import datetime
import math
import numpy as np
import yaml
from rosbag.bag import Bag

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
"""
Required topics
- SPaT Data : /message/incoming_spat cav_msgs/SPAT
Other Required information
- Movement Ids from the Spat that are needed for the vehicle and scenario being tested (may need to be a hard coded mapping)
- Intersection Id for the spat. Since we have two intersections but only one is used in the test the id of interest should be hard coded into the analysis
  With the expected id we can verify that the required correct spat message is received.
Approach:
- Spat will provide the current light phase state by matching the movement id for the current scenario into the plan
- Spat fields can be checked for availability to ensure the listed fields in the metric are available
- Intersection id of the spat message can be checked to ensure it is the correct message being analyzed
"""


###
# Mike suggested. MAP messages received and processed and they contain intersection ids matching the expected values
# This when combined with above metric confirms all required v2i information is available
# TODO implement if extra time
###
"""
Required topics
- Map Data : /message/incoming_map cav_msgs/Map
Other Required Data:
- Map messages have intersection Ids. Since we have two intersections but only one is used in the test the id of interest should be hard coded into the analysis
  With the expected id we can verify that the required map message is received. 
"""


###########################################################################################################
# TSMO UC2-M1.2 : The communication from infrastructure (RSU) to vehicle (OBU) should be within a certain frequency rates
#
# Expected: 10 Hz ± 1 Hz for SPaT, 1 Hz ± 0.5 Hz for MA
###########################################################################################################
def plot_spats_maps_frequency(plot_title, vehicles_plot_data_list, y_axis_max):
    subplot_nums = len(vehicles_plot_data_list)
    fig, ax = plt.subplots() if subplot_nums == 1 else plt.subplots(subplot_nums)
    print(
        "Detected %s numbers of vehicle(s)' profile data. Plotting >>  %s "
        % (subplot_nums, plot_title)
    )
    colors = ["blue", "orange", "purple", "cyan", "black", "brown", "gray"]
    fig.suptitle(plot_title, fontsize=14)
    i = 0
    while i < subplot_nums:
        ax_local = ax if subplot_nums == 1 else ax[i]

        relative_spat_x_axis_idx = []
        spat_idx = vehicles_plot_data_list[i]["spat_idx"]
        for idx in spat_idx:
            relative_spat_x_axis_idx.append(idx.timestamp() - spat_idx[0].timestamp())

        spat_df = pd.DataFrame(
            {
                "Average SPAT Frequncy every 1s": vehicles_plot_data_list[i][
                    "spat_avg_frequency"
                ]
            },
            index=relative_spat_x_axis_idx,
        )

        spat_df_sum_avg = pd.DataFrame(
            {
                vehicles_plot_data_list[i]["v_name"]
                + ": Average sum Frequncy SPAT": vehicles_plot_data_list[i][
                    "spat_sum_avg_frequency"
                ]
            },
            index=relative_spat_x_axis_idx,
        )
        relative_map_x_axis_idx = []
        map_idx = vehicles_plot_data_list[i]["map_idx"]
        for idx in map_idx:
            relative_map_x_axis_idx.append(idx.timestamp() - map_idx[0].timestamp())

        map_df = pd.DataFrame(
            {
                "Average MAP Frequncy every 5s": vehicles_plot_data_list[i][
                    "map_avg_frequency"
                ]
            },
            index=relative_map_x_axis_idx,
        )
        map_df_sum_avg = pd.DataFrame(
            {
                vehicles_plot_data_list[i]["v_name"]
                + ": Sum Average MAP Sum Frequncy": vehicles_plot_data_list[i][
                    "map_sum_avg_frequency"
                ]
            },
            index=relative_map_x_axis_idx,
        )

        styles = ["-", "-"]
        print(relative_spat_x_axis_idx)
        print(relative_map_x_axis_idx)
        print(vehicles_plot_data_list)

        # Plot sum Avergage frequency
        spat_df_sum_avg.plot(ax=ax_local, style=styles[0], color=colors[0])
        map_df_sum_avg.plot(ax=ax_local, style=styles[1], color=colors[1])

        # Sliding window frequency average
        ax_local.scatter(
            map_df.index,
            map_df.values,
            s=20,
            color=colors[2],
            marker="^",
            label=vehicles_plot_data_list[i]["v_name"]
            + ": Average MAP Frequncy every 5s",
        )
        ax_local.scatter(
            spat_df.index,
            spat_df.values,
            s=20,
            color=colors[3],
            marker="s",
            label=vehicles_plot_data_list[i]["v_name"]
            + ": Average SPAT Frequncy every 1s",
        )
        ax_local.legend(loc="upper right")

        ax_local.set_ylabel(vehicles_plot_data_list[i]["y_label"])
        start_x, start_y = 0, 0
        ax_local.set_xlabel("Time (s)")
        stepsize_x, stepsize_y = 1, 1
        end_y = y_axis_max + stepsize_y
        ax_local.set_ylim(0, end_y)
        ax_local.set_yticks(np.arange(start_y, end_y, stepsize_y))
        end_x = max(relative_map_x_axis_idx[len(relative_map_x_axis_idx) - 1] , relative_spat_x_axis_idx[len(relative_spat_x_axis_idx) - 1]) + stepsize_x
        ax_local.set_xticks(np.arange(start_x, end_x, stepsize_x))
        i += 1


def show_spats_maps_frequency_profiles(vehicle_profiles_data):
    """
    Required topics
    - SPaT Data : /message/incoming_spat cav_msgs/SPAT
    - Map Data : /message/incoming_map cav_msgs/Map
    - Engagement status : /guidance/state cav_msgs/GuidanceState

    Approach:
    - After the first 3 Spat messages have been received consider communication stable enough for analysis (this may be before or after engagement)
    - Count the number of both spat and map starting from engagement to disengagement.
    Divide this number by the total time period of engagement to get total average frequency for each message
    - Additionally, create a 1s sliding window for spat and a 5s sliding window for map and verify that the average frequency within these windows never falls outside the specified ranges

    Plotting:
    - In addition to the pass/fail criterion plots should be generated of the frequency in the sliding windows at each timestep
    """
    vehicles_plot_data_list = []
    for vehicle_profile_data in vehicle_profiles_data:
        spat_nums_ts = [
            spat_num_ts for spat_num_ts in vehicle_profile_data.spat_nums_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(spat_nums_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            spat_nums_ts[len(spat_nums_ts) - 1]
        )
        spat_nums_idx = pd.date_range(date_time_1, date_time_5, freq="S")

        map_nums_ts = [
            map_num_ts for map_num_ts in vehicle_profile_data.map_nums_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(map_nums_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            map_nums_ts[len(map_nums_ts) - 1]
        )
        map_nums_idx = pd.date_range(date_time_1, date_time_5, freq="5S")

        v_name = vehicle_profile_data.vehicle_name

        # Calculate average frequency for SPAT in every 1 second
        spat_nums_ts_list = [
            spat_num_ts for spat_num_ts in vehicle_profile_data.spat_nums_ts.values()
        ]
        map_nums_ts_list = [
            map_num_ts for map_num_ts in vehicle_profile_data.map_nums_ts.values()
        ]
        # Calculate average frequency for SPAT in every 5 second
        map_avg_freq = []
        count = 0
        i=0
        map_num_sum_ts = 0
        while i < len(map_nums_idx):
            count += 1
            map_num_sum_ts += map_nums_ts_list[i]
            if count % 5 == 0:
                map_avg_freq.append(map_num_sum_ts / 5)
                map_num_sum_ts = 0
            i+=1

        # Calculate total average frequency for SPAT
        spat_sum_avg_frequency = [sum(spat_nums_ts_list) / (spat_nums_ts[len(spat_nums_ts)-1]-spat_nums_ts[0])]
        # Calculate total average frequency for SPAT
        if len(map_avg_freq) == 0:
            print("ERROR: Map list size = 0")
            return
        map_sum_avg_frequency = [sum(map_nums_ts_list) / (map_nums_ts[len(map_nums_ts)-1]-map_nums_ts[0])]

        vehicles_plot_data_dict = {
            "v_name": v_name,
            "spat_avg_frequency": spat_nums_ts_list,
            "spat_sum_avg_frequency": spat_sum_avg_frequency,
            "spat_idx": spat_nums_idx,
            "map_avg_frequency": map_avg_freq,
            "map_sum_avg_frequency": map_sum_avg_frequency,
            "map_idx": map_nums_idx,
            "y_label": "Frequency (HZ)",
        }
        vehicles_plot_data_list.append(vehicles_plot_data_dict)

    plot_title = "MAP and SPAT Frequency Profile"
    y_axis_max = max(max(spat_nums_ts_list), max(map_avg_freq))
    plot_spats_maps_frequency(plot_title, vehicles_plot_data_list, y_axis_max)


###########################################################################################################
# TSMO UC2-M2.1 : Each vehicle should travel sequentially through the lanelets defined in its path.
#
# Expected: True
# TODO implement. Note the expected set of lanelets changes based on the approach being used by the vehicle
###########################################################################################################

"""
Required topics
- Current lanelet data : /guidance/route_state cav_msgs/RouteState
- Engagement status : /guidance/state cav_msgs/GuidanceState

Approach: 
- Hard code the expected lanelet ids for each approach in each test scenario
- For a given run after the system is engaged, verify that the route_state reports the current lanelets in sequence following the same order as specified for the scenario and approach
"""

###########################################################################################################
#
# NOTE: This metric may get tweaked. Suggest tackling last

# TSMO UC2-M2.2 : Each vehicle shall process the SPaT message
# and adjust its speed such that it can enter the intersection box at a green phase with minimum or no stopping
# Specifically the vehicle should on average stop for less time then would be required by the baseline case of EET (Earliest entry time)
#
# Expected: Vehicle's actual stopping time is less then predicted from EET
#
# TODO implement
###########################################################################################################

"""
Required topics
- Not a topic, but will need knowledge of current test case being evaluated to determine expected behavior.
- Current lanelet and downtrack: /guidance/route_state cav_msgs/RouteState
- Engagement status : /guidance/state cav_msgs/GuidanceState
- Current_speed : /hardware_interface/vehicle/twist geometry_msgs/TwistStamped (http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistStamped.html)
- SPaT Data : /message/incoming_spat cav_msgs/SPAT

Other Required Data:
- Log file from lci_strategic_plugin with DEBUG logs enabled. This is needed to get the EET value
- Intersection and movement ids for identifying correct spat

Approach: 
- At the start of vehicle engagement look in the lci_strategic_plugin log file for the first occurrence of "earliest_entry_time" This will be the EET value. (from ROS_DEBUG_STREAM("earliest_entry_time: " << std::to_string(earliest_entry_time.toSec()) << ", with : " << earliest_entry_time - current_state.stamp  << " left at: " << std::to_string(current_state.stamp.toSec()));)
- Using the EET value determine if the light was red, yellow, or green from SPAT
- If the light was red at EET then find the next time when the light turns green
-- Save the delta between next green start and EET (will refer to as BaseCaseStopTime)
-- Based on the data determine how much time the vehicle was at a stop at the stop bar while engaged (will refer to as VehicleStopTime)
-- If VehicleStopTime < BaseCaseStopTime 
---- Record a Successes (algorithm performs better than base case)
- If the light was green at EET 
-- If the vehicle stops in this case,
---- Record failure
-- else 
---- Record success
- If the light was yellow at EET
-- If the vehicle stops 
---- Record success
-- If the vehicle does not stop
---- Record failure


"""

###########################################################################################################
# TSMO UC2-M2.3 : Vehicles shall not enter the intersection box at yellow or red phases.
#
# Expected: True
# TODO implement
###########################################################################################################

"""
Required topics
- Current lanelet data : /guidance/route_state cav_msgs/RouteState
- Engagement status : /guidance/state cav_msgs/GuidanceState
- SPaT Data : /message/incoming_spat cav_msgs/SPAT
Other Required Data:
- The intersection and movement ids expected for the vehicle and scenario being tested

Approach:
- Use the intersection id and movement id to identify the current phase of the light which is required
- Use the route_state to determine the current lanelet of the vehicle 
- While the vehicle is engaged, its reported lanelet should never be one inside the intersection while the phase state is not green
"""

###########################################################################################################
# TSMO UC2-M2.4 : Each vehicle should switch away from the TS mode after entering the intersection box
# Note that the intersection box is the box per the HD map, which may not be exactly as physically seen on the road.
# Expected: True
# TODO implement
###########################################################################################################

"""
Required topics
- Current lanelet data : /guidance/route_state cav_msgs/RouteState
- Engagement status : /guidance/state cav_msgs/GuidanceState
- Determine lci plugin is used in planning : /guidance/plan_trajectory cav_msgs/TrajectoryPlan

Approach:
- When the vehicle is engaged, gets its current lanelet from the route state
- Verify that when the lanelet is one inside the intersection the current planning trajectory is not coming from the lci tactical plugin
- To check the controlling tactical plugin look at the planner_plugin field of the first point on the current plan_trajectory
"""

###
# Mike suggested. Vehicle does not violate speed limit my more than 1 mph
# Good to confirm as this showed up as an issue multiple times during integration testing
# TODO implement if extra time
###
"""
Required topics
- Engagement status : /guidance/state cav_msgs/GuidanceState
- Current_speed : /hardware_interface/vehicle/twist geometry_msgs/TwistStamped
Other Required Data:
- Speed limit extracted from map

Approach:
- In TSMO UC2 all the speed limits will be the same so a simple conditional while the vehicle is engaged on twist.linear.x will suffice to check if the limit is exceeded
"""

TSC_phases_lookup = {
    0: "gray",
    1: "red",  # DARK
    2: "red",  # STOP_THEN_PROCEED
    3: "red",  # STOP_AND_REMAIN
    4: "gray",  # PRE_MOVEMENT
    5: "green",  # PERMISSIVE_MOVEMENT_ALLOWED
    6: "green",  # PROTECTED_MOVEMENT_ALLOWED
    7: "yellow",  # PERMISSIVE_CLEARANCE
    8: "yellow",  # PROTECTED_CLEARANCE
    9: "yellow",  # CAUTION_CONFLICTING_TRAFFIC
}


class vehicle_profile_data:
    """
    Keep track of relevant vehicle metrics when the vehicle is engaged for plotting purposes. The vehicle profile metrics include:
    1) vehicle current speed over time (unit of second) during engaged
    2) vehicle current acceleration over time (unit of second) during engaged
    3) vehicle current downtrack over time (unit of second) during engaged
    4) vehicle received spat message every 1 second during engaged
    5) vehicle received map message every 1 second during engaged
    5) Bag name: The name of bag that is generated by the vehicle
    6) The route the vehicle traversed when engaged
    7) Vehicle engaged timestamp
    8) Vehicle disengaged timestamp
    9) Vehicle distance to the traffic signal location it faced over time (unit of second) during engaged
    2) vehicle current deceleration over time (unit of second) during engaged
    """

    def __init__(self):
        self.cur_speeds_ts = {}
        self.cur_accels_ts = {}
        self.cur_downtracks_ts = {}
        self.speed_limits_ts = {}
        self.spat_phase_ts = {}
        self.spat_nums_ts = {}
        self.map_nums_ts = {}
        self.bag_name = ""
        self.route_name = ""
        self.vehicle_name = ""
        self.engage_start_ts = 0
        self.engage_end_ts = 0
        self.ds_tf_ts = {}
        self.cur_decels_ts = {}
        self.signal_group_id = 0
        self.intersection_id = 0

    def update_cur_speeds_ts(self, cur_speed, timestamp):
        """Update the vehicle current latest speed given the timestamp

        Args:
            cur_speed (double): Vehicle current speed over time
            timestamp (_type_): unit of nanosecond
        """
        timestamp_s = math.floor(timestamp.to_sec())
        self.cur_speeds_ts[timestamp_s] = cur_speed

    def update_cur_accels_ts(self, cur_accel, timestamp):
        """Update the vehicle current latest aceleration given the timestamp

        Args:
            cur_accel (double): Vehicle current acceleration over time
            timestamp (_type_): unit of nanosecond
        """
        timestamp_s = math.floor(timestamp.to_sec())
        self.cur_accels_ts[timestamp_s] = cur_accel

    def update_cur_decels_ts(self, cur_decel, timestamp):
        """Update the vehicle current latest decelerations given the timestamp

        Args:
            cur_decel (double): Vehicle current decelerations over time
            timestamp (_type_): unit of nanosecond
        """
        timestamp_s = math.floor(timestamp.to_sec())
        self.cur_decels_ts[timestamp_s] = cur_decel

    def update_cur_downtracks_ts(self, cur_downtrack, timestamp):
        """Update the vehicle latest downtrack given the timestamp

        Args:
            cur_downtrack (double): Vehicle current distance to vehicle starting engagement location
            timestamp (timestamp): unit of nanosecond
        """
        timestamp_s = math.floor(timestamp.to_sec())
        self.cur_downtracks_ts[timestamp_s] = cur_downtrack

    def update_speed_limit_ts(self, speed_limit, timestamp):
        timestamp_s = math.floor(timestamp.to_sec())
        self.speed_limits_ts[timestamp] = speed_limit

    def update_spat_phase_ts(self, spat_phase, timestamp):
        """Update the spat phases dictionary given the timestamp

        Args:
            spat_phase (_type_): string of 'red','green' or 'yellow'
            timestamp (_type_): unit of nanosecond
        """
        timestamp_s = math.floor(timestamp.to_sec())
        self.spat_phase_ts[timestamp_s] = spat_phase

    def update_spat_nums_ts(self, spat_num, timestamp):
        """Update the number of spats given the timestamp

        Args:
            spat_num (_type_): integer 1
            timestamp (_type_): unit of nanosecond
        """
        timestamp_ts = math.floor(timestamp.to_sec())
        if timestamp_ts in self.spat_nums_ts.keys():
            self.spat_nums_ts[timestamp_ts] = spat_num + self.spat_nums_ts[timestamp_ts]
        else:
            self.spat_nums_ts[timestamp_ts] = spat_num

    def update_ds_tf_ts(self, distance2tf, timestamp):
        """Update the vehicle distance to traffic signal location it faced given the timestamp

        Args:
            distance2tf (double): _description_
            timestamp (timestamp): unit of nanosecond
        """
        timestamp_ts = math.floor(timestamp.to_sec())
        self.ds_tf_ts[timestamp_ts] = distance2tf

    def update_map_num_ts(self, map_num, timestamp):
        timestamp_s = math.floor(timestamp.to_sec())
        """Update the number of maps given the timestamp.

        Args:
            map_num (_type_): integer between 1
            timestamp (_type_): unit of nanosecond
        """
        if timestamp_s in self.map_nums_ts.keys():
            self.spat_nums_ts[timestamp_s] = map_num + self.spat_nums_ts[timestamp_s]
        else:
            self.map_nums_ts[timestamp_s] = map_num

    def __profile__(self) -> str:
        """This function shows some profile information.

        Returns:
            str: summarized profile information.
        """
        cur_accel_info = ", ".join(
            [cur_accel_ts for cur_accel_ts in self.cur_accels_ts]
        )
        cur_speed_info = ", ".join(
            [cur_speed_ts for cur_speed_ts in self.cur_speeds_ts]
        )
        cur_track_info = ", ".join(
            [cur_downtrack_ts for cur_downtrack_ts in self.cur_downtracks_ts]
        )
        speed_limit_info = ", ".join(
            [speed_limit_ts for speed_limit_ts in self.speed_limits_ts]
        )
        spat_num_info = ", ".join([spat_tss for spat_tss in self.spat_ts])
        map_num_info = ", ".join([map_num_ts for map_num_ts in self.map_nums_ts])
        return f"{cur_accel_info} | {cur_speed_info} | {cur_track_info} | {speed_limit_info} | {spat_num_info} | {map_num_info}"

    def __prof_summary__(self) -> str:
        return f"Route name: {self.route_name}, vehicle from bag : {self.bag_name}, engaged start at timestamp {self.engage_start_ts}, engaged end at timestamp {self.engage_end_ts}"

    def __repr__(self) -> str:
        return f"{self.__profile__()}\n{self.__prof_summary__()}"

    def __str__(self) -> str:
        return self.__repr__()


def load_rosbags_to_vehicles_profile(bag_vehicle_names):
    """
    Get information about a bag as a vehicle profile object.
    """
    vehicles_profiles = []
    # loop bags to get relevant vehicles information and update the vehicles profile list
    for bag_name in bag_vehicle_names.keys():
        print("Loading bag (name = %s) ..." % bag_name)
        ENGAGED = 4
        is_engaged = False
        engage_start_ts = 0
        engage_end_ts = 0
        bag = Bag(bag_name, "r")
        # Vehicle engage timestamp
        for topic, msg, t in bag.read_messages(topics=["/guidance/state"]):
            # print(msg.state)
            if (not is_engaged) and msg.state == ENGAGED and engage_start_ts == 0:
                print("Vehicle is engaged at timestamp = ", t)
                engage_start_ts = t
                is_engaged = True
            elif is_engaged and msg.state != ENGAGED:
                print("Vehicle is disengaged at timestamp = ", t)
                engage_end_ts = t
                is_engaged = False
                break

        if (
            engage_start_ts == 0
            or engage_end_ts == 0
            or engage_start_ts > engage_end_ts
        ):
            print("Bad Bag file. Vehicle has never engaged.")
            return

        # Only interested in data when vehicle is engaged
        if (
            engage_start_ts != 0
            and engage_end_ts != 0
            and engage_start_ts <= engage_end_ts
        ):

            vpd = vehicle_profile_data()
            vpd.engage_start_ts = engage_start_ts
            vpd.engage_end_ts = engage_end_ts
            vpd.bag_name = bag_name
            vpd.vehicle_name = bag_vehicle_names[bag_name]

            # Traffic Signal Group and intersection ids
            for topic, msg, t in bag.read_messages(
                topics=["/environment/intersection_signal_group_ids"]
            ):
                vpd.signal_group_id = msg.data[1]
                vpd.intersection_id = msg.data[0]

            topics = [
                "/hardware_interface/vehicle/twist",
                "/message/incoming_spat",
                "/message/incoming_map",
                "/hardware_interface/velocity_accel_cov",
                "/guidance/route_state",
                "/guidance/lci_strategic_plugin/distance_remaining_to_tf",
            ]

            # topics_tmp = bag.get_type_and_topic_info()[1].keys()
            # for topic in topics_tmp:
            #     pprint(topic)
            for topic, msg, t in bag.read_messages(topics=topics):
                if t >= engage_start_ts and t <= engage_end_ts:
                    if topic == "/hardware_interface/vehicle/twist":
                        vehicle_cur_speed = msg.twist.linear.x
                        timestamp = t
                        vpd.update_cur_speeds_ts(vehicle_cur_speed, timestamp)
                        # print(msg)
                    elif topic == "/message/incoming_spat":
                        spat_phase = 0
                        for intersection_state in msg.intersection_state_list:
                            if intersection_state.id.id == vpd.intersection_id:
                                for movement_list in intersection_state.movement_list:
                                    if (
                                        movement_list.signal_group
                                        == vpd.signal_group_id
                                    ):
                                        spat_phase = movement_list.movement_event_list[
                                            0
                                        ].event_state.movement_phase_state
                        vpd.update_spat_nums_ts(1, t)
                        vpd.update_spat_phase_ts(spat_phase, t)
                        # print(msg)
                    elif topic == "/message/incoming_map":
                        # print(msg)
                        vpd.update_map_num_ts(1, t)
                    elif topic == "/hardware_interface/velocity_accel_cov":
                        # print(msg)
                        vehicle_cur_accel = msg.accleration
                        timestamp = t
                        vpd.update_cur_accels_ts(vehicle_cur_accel, timestamp)
                    elif (
                        topic
                        == "/guidance/lci_strategic_plugin/distance_remaining_to_tf"
                    ):
                        # print(msg)
                        vpd.update_ds_tf_ts(msg.data, t)
                    elif topic == "/guidance/route_state":
                        # print(msg.down_track)
                        vehicle_cur_downtrack = msg.down_track
                        vehicle_cur_speed_limit = msg.speed_limit
                        timestamp = t
                        vpd.update_cur_downtracks_ts(vehicle_cur_downtrack, timestamp)
                        vpd.update_speed_limit_ts(vehicle_cur_speed_limit, timestamp)
                        vpd.route_name = msg.routeID
        # Finished loading bag data into the vehicle profile object. Add the vehicle profile into the list
        # print(vpd.spat_nums_ts)
        # print(vpd.spat_phase_ts)
        # print(vpd.map_nums_ts)
        # print(vpd.ds_tf_ts)
        vehicles_profiles.append(vpd)

    return vehicles_profiles


def plot_vehicle_speed_accel_downtrack_with_tsc(
    vehicles_plot_data,
    tsc_bar_height,
    max_y_axis,
    y_axis_stepsize,
    plot_title,
):
    subplot_nums = len(vehicles_plot_data)
    print(
        "Detected %s numbers of vehicle(s)' profile data. Plot vehicle(s)' profile(s) with TSC. Plotting >>  %s "
        % (subplot_nums, plot_title)
    )
    fig, ax = plt.subplots() if subplot_nums == 1 else plt.subplots(subplot_nums)
    colors = ["blue", "black", "purple", "cyan", "orange", "brown"]
    fig.suptitle(plot_title, fontsize=14)
    i = 0
    while i < subplot_nums:
        # Vehicle info lines
        ts_idx = vehicles_plot_data[i][vehicles_plot_data[i]["profile_name"] + "_idx"]
        df = pd.DataFrame(vehicles_plot_data[i], index=ts_idx)
        relative_x_axis_idx = []
        for idx in ts_idx:
            relative_x_axis_idx.append(idx.timestamp() - ts_idx[0].timestamp())
        ax_local = ax if subplot_nums == 1 else ax[i]
        ax_local.plot(
            relative_x_axis_idx,
            df[vehicles_plot_data[i]["profile_name"]],
            color=colors[i],
            label=vehicles_plot_data[i]["v_name"],
        )
        ax_local.legend(loc="upper right")
        # tsc_start_phase = vehicles_plot_data[i]["tsc_start_phase"]
        tsc_start_y = vehicles_plot_data[i]["tsc_start_y"]

        # # Traffic signal with horizontal bars
        period_count_start = relative_x_axis_idx[0]
        period_count_start_end = relative_x_axis_idx[len(relative_x_axis_idx) - 1]
        start_phase_pos = 0
        is_green_label_displayed = False
        is_red_label_displayed = False
        is_yellow_label_displayed = False
        for phase in vehicles_plot_data[i]["tsc_phases"]:
            if TSC_phases_lookup[phase] in "yellow" and not is_yellow_label_displayed:
                is_yellow_label_displayed = True
                tsc_label =  "Yellow Phase"
            elif TSC_phases_lookup[phase] in "red" and not is_red_label_displayed:
                is_red_label_displayed = True
                tsc_label =  "Red Phase"
            elif TSC_phases_lookup[phase] in "green" and not is_green_label_displayed:
                is_green_label_displayed = True
                tsc_label =  "Green Phase"
            else:
                tsc_label = ""
            ax_local.broken_barh([(start_phase_pos,1)], (tsc_start_y, tsc_bar_height), facecolors=TSC_phases_lookup[phase], label= tsc_label if len(tsc_label) != 0 else "", alpha=vehicles_plot_data[i]["tsc_alpha"])
            start_phase_pos += 1
        ax_local.legend(loc="upper right")

        # x-axis and y-axis
        ax_local.grid(True)
        ax_local.set_xlabel("Time (s)")
        ax_local.set_ylabel(vehicles_plot_data[i]["y_label"])
        start_x, start_y = 0, 0
        stepsize_x = 1  # unit of second
        end_x, end_y = (
            max(period_count_start, period_count_start_end) + stepsize_x,
            max_y_axis + y_axis_stepsize,
        )
        ax_local.set_xticks(np.arange(start_x, end_x, stepsize_x))
        ax_local.set_yticks(np.arange(start_y, end_y, y_axis_stepsize))
        i += 1


###########################################################################################################
# Additional Requested Plots: Speed over time with matching signal state shown
###########################################################################################################
def show_vehicles_speed_profiles(vehicle_profiles_data):
    """
    Required topics
    - Engagement status : /guidance/state
    - Current_speed : /hardware_interface/vehicle/twist
    - SPaT Data : /message/incoming_spat

    Other Required Data:
    - Intersection id and movement id for the spat message

    Approach:
    - While the vehicle is engaged plot the current speed twist.linear.x
    - Also plot the signal phase as red, green, yellow bars on the same graph

    NOTE: This plot should be generated in two forms 1 time for the individual vehicles and once as a single plot combing the data from two vehicles
    """
    vehicles_plot_data_list = []
    for vehicle_profile_data in vehicle_profiles_data:
        cur_speed_idx = [
            cur_speed_idx for cur_speed_idx in vehicle_profile_data.cur_speeds_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(cur_speed_idx[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            cur_speed_idx[len(cur_speed_idx) - 1]
        )
        cur_speed_ts_idx = pd.date_range(date_time_1, date_time_5, freq="S")
        cur_speed_list = [
            cur_speed for cur_speed in vehicle_profile_data.cur_speeds_ts.values()
        ]

        spat_phases_ts = [
            spat_phase_ts for spat_phase_ts in vehicle_profile_data.spat_phase_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(spat_phases_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            spat_phases_ts[len(spat_phases_ts) - 1]
        )
        phase_list = [
            cur_phase for cur_phase in vehicle_profile_data.spat_phase_ts.values()
        ]

        ds_tf_ts = [
            value for value in vehicle_profile_data.ds_tf_ts.values()
        ]

        v_name = vehicle_profile_data.vehicle_name

        tsc_start_y = 0
        tsc_bar_height = 15
        tsc_alpha = 0.5  # opacity of bars
        vehicles_plot_data_dict = {
            "v_name": v_name,
            "profile_name": "speed",
            "speed": cur_speed_list,
            "speed_idx": cur_speed_ts_idx,
            "y_label": "Speed (m/s)",
            "tsc_phases": phase_list,
            "tsc_start_y": tsc_start_y,
            "tsc_alpha": tsc_alpha,
        }
        vehicles_plot_data_list.append(vehicles_plot_data_dict)

    max_y_axis = max(
        max(vehicles_plot_data_list[0]["speed"]), tsc_bar_height + tsc_start_y
    )
    y_axis_stepsize = 1
    plot_title = "Vehicle Speed Profile"
    plot_vehicle_speed_accel_downtrack_with_tsc(
        vehicles_plot_data_list,
        tsc_bar_height,
        max_y_axis,
        y_axis_stepsize,
        plot_title,
    )


###########################################################################################################
# TSMO UC2-M2.3 : The average deceleration\acceleration over any 1-second portion of the test time horizon
# shall be no greater than 3.0 m/s^2.
#
# Expected: acceleration <= 3.0 m/s^2 deceleration >= -3.0 m/s^2
###########################################################################################################

"""
Required topics
- Engagement status : /guidance/state cav_msgs/GuidanceState
- Current_speed : /hardware_interface/vehicle/twist (use to compute accel) geometry_msgs/TwistStamped
- Vehicle accel : /hardware_interface/velocity_accel_cov (use as alternative source of accel) automotive_platform_msgs/VelocityAccelCov (https://github.com/astuff/automotive_autonomy_msgs/blob/master/automotive_platform_msgs/msg/VelocityAccelCov.msg)

Approach:
- While the vehicle is engaged, create a 1s sliding window and take the start and end speed for each timestep of the window and compute the average acceleration
- Additionally, verify that the abs(/hardware_interface/velocity_accel_cov reported value) never exceeds 3.0 for more than 1 timestep at any point during engagement
"""

###########################################################################################################
# Additional Requested Plots: Acceleration over time with matching signal state shown
###########################################################################################################
def show_vehicles_accel_profiles(vehicle_profiles_data):
    """
    Required topics
    - Engagement status : /guidance/state
    - Current_speed : /hardware_interface/vehicle/twist (use to compute accel)
    - Vehicle accel : /hardware_interface/velocity_accel_cov (use as alternative source of accel)
    - SPaT Data : /message/incoming_spat
    Other Required Data:
    - Intersection id and movement id for the spat message

    Approach:
    - While the vehicle is engaged plot the current acceleration
    - Also plot the signal phase as red, green, yellow bars on the same graph

    NOTE: This plot should be generated in two forms 1 time for the individual vehicles and once as a single plot combing the data from two vehicles
    """
    vehicles_plot_data_list = []
    for vehicle_profile_data in vehicle_profiles_data:
        cur_accels_ts = [
            cur_accels_ts for cur_accels_ts in vehicle_profile_data.cur_speeds_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(cur_accels_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            cur_accels_ts[len(cur_accels_ts) - 1]
        )
        cur_accel_ts_idx = pd.date_range(date_time_1, date_time_5, freq="S")
        cur_accel_list = [
            cur_accel for cur_accel in vehicle_profile_data.cur_accels_ts.values()
        ]

        spat_phases_ts = [
            spat_phase_ts for spat_phase_ts in vehicle_profile_data.spat_phase_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(spat_phases_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            spat_phases_ts[len(spat_phases_ts) - 1]
        )
        phase_list = [
            cur_phase for cur_phase in vehicle_profile_data.spat_phase_ts.values()
        ]

        ds_tf_ts = [
            value for value in vehicle_profile_data.ds_tf_ts.values()
        ]

        v_name = vehicle_profile_data.vehicle_name

        tsc_start_y = 0
        tsc_bar_height = 3
        tsc_alpha = 0.3  # opacity of bars
        vehicles_plot_data_dict = {
            "v_name": v_name,
            "profile_name": "accel",
            "accel": cur_accel_list,
            "accel_idx": cur_accel_ts_idx,
            "y_label": "Accel (m/s^2)",
            "tsc_phases": phase_list,
            "tsc_start_y": tsc_start_y,
            "tsc_alpha": tsc_alpha,
        }
        vehicles_plot_data_list.append(vehicles_plot_data_dict)
    max_y_axis = max(
        max(vehicles_plot_data_list[0]["accel"]), tsc_bar_height + tsc_start_y
    )
    y_axis_stepsize = 1
    plot_title = "Vehicle Acceleration Profile"
    plot_vehicle_speed_accel_downtrack_with_tsc(
        vehicles_plot_data_list,
        tsc_bar_height,
        max_y_axis,
        y_axis_stepsize,
        plot_title,
    )


###########################################################################################################
# Additional Requested Plots: Downtrack position over time with matching signal state shown
###########################################################################################################
def show_vehicles_downtrack_profiles(vehicle_profiles_data):
    """
    Required topics
    - Engagement status : /guidance/state
    - Current downtrack data : /guidance/route_state
    - SPaT Data : /message/incoming_spat
    Other Required Data:
    - Intersection id and movement id for the spat message

    Approach:
    - While the vehicle is engaged plot the current downtrack distance reported by route_state
    - Identify where the stop bar is in terms of downtrack distance (this might change per run)
    - At the position where the stop bar is (y-axis) draw a set of red/green/yellow rectangles showing the current signal phase

    NOTE: This plot should be generated in two forms 1 time for the individual vehicles and once as a single plot combing the data from two vehicles
    """
    vehicles_plot_data_list = []
    for vehicle_profile_data in vehicle_profiles_data:
        cur_downtrack_ts = [
            cur_downtrack_ts
            for cur_downtrack_ts in vehicle_profile_data.cur_downtracks_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(cur_downtrack_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            cur_downtrack_ts[len(cur_downtrack_ts) - 1]
        )
        cur_downtrack_ts_idx = pd.date_range(date_time_1, date_time_5, freq="S")
        cur_downtrack_list = [
            cur_downtrack
            for cur_downtrack in vehicle_profile_data.cur_downtracks_ts.values()
        ]

        spat_phases_ts = [
            spat_phase_ts for spat_phase_ts in vehicle_profile_data.spat_phase_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(spat_phases_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            spat_phases_ts[len(spat_phases_ts) - 1]
        )
        phase_list = [
            cur_phase for cur_phase in vehicle_profile_data.spat_phase_ts.values()
        ]

        ds_tf_ts = [
            value for value in vehicle_profile_data.ds_tf_ts.values()
        ]
        # print(ds_tf_ts)
        # print(cur_downtrack_list)

        v_name = vehicle_profile_data.vehicle_name

        # Traffic Signal bars config
        tsc_start_y = max(ds_tf_ts)
        tsc_bar_height = round(max(cur_downtrack_list)/10)
        tsc_alpha = 1  # opacity of bars
        vehicles_plot_data_dict = {
            "v_name": v_name,
            "profile_name": "downtrack",
            "downtrack": cur_downtrack_list,
            "downtrack_idx": cur_downtrack_ts_idx,
            "y_label": "Downtrack (m)",
            "tsc_phases": phase_list,
            "tsc_start_y": tsc_start_y,
            "tsc_alpha": tsc_alpha,
        }
        vehicles_plot_data_list.append(vehicles_plot_data_dict)

    max_y_axis = max(
        max(vehicles_plot_data_list[0]["downtrack"]), tsc_bar_height + tsc_start_y
    )
    y_axis_stepsize = round(max_y_axis/10)
    plot_title = "Vehicle Downtrack Profile"
    plot_vehicle_speed_accel_downtrack_with_tsc(
        vehicles_plot_data_list,
        tsc_bar_height,
        max_y_axis,
        y_axis_stepsize,
        plot_title,
    )

def show_vehicles_distance2tf_profiles(vehicle_profiles_data):
    """
    Required topics
    - Engagement status : /guidance/state
    - Current downtrack data : /guidance/route_state
    - SPaT Data : /message/incoming_spat
    Other Required Data:
    - Intersection id and movement id for the spat message

    Approach:
    - While the vehicle is engaged plot the current downtrack distance reported by route_state
    - Identify where the stop bar is in terms of downtrack distance (this might change per run)
    - At the position where the stop bar is (y-axis) draw a set of red/green/yellow rectangles showing the current signal phase

    NOTE: This plot should be generated in two forms 1 time for the individual vehicles and once as a single plot combing the data from two vehicles
    """
    vehicles_plot_data_list = []
    for vehicle_profile_data in vehicle_profiles_data:
        ds_tf_timestamp= [
            key
            for key in vehicle_profile_data.ds_tf_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(ds_tf_timestamp[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            ds_tf_timestamp[len(ds_tf_timestamp) - 1]
        )
        ds_tf_ts_idx = pd.date_range(date_time_1, date_time_5, freq="S")        
        ds_tf_ts = [
            value for value in vehicle_profile_data.ds_tf_ts.values()
        ]

        spat_phases_ts = [
            spat_phase_ts for spat_phase_ts in vehicle_profile_data.spat_phase_ts.keys()
        ]
        date_time_1 = datetime.datetime.utcfromtimestamp(spat_phases_ts[0])
        date_time_5 = datetime.datetime.utcfromtimestamp(
            spat_phases_ts[len(spat_phases_ts) - 1]
        )
        phase_list = [
            cur_phase for cur_phase in vehicle_profile_data.spat_phase_ts.values()
        ]

        v_name = vehicle_profile_data.vehicle_name

        # Traffic Signal bars config
        tsc_start_y = max(ds_tf_ts)
        tsc_bar_height = round(tsc_start_y/10)
        tsc_alpha = 1  # opacity of bars
        vehicles_plot_data_dict = {
            "v_name": v_name,
            "profile_name": "distance2tf",
            "distance2tf": ds_tf_ts,
            "distance2tf_idx": ds_tf_ts_idx,
            "y_label": "Distance (m)",
            "tsc_phases": phase_list,
            "tsc_start_y": tsc_start_y,
            "tsc_alpha": tsc_alpha,
        }
        vehicles_plot_data_list.append(vehicles_plot_data_dict)

    max_y_axis = max(
        max(vehicles_plot_data_list[0]["distance2tf"]), tsc_bar_height + tsc_start_y
    )
    y_axis_stepsize = round(max_y_axis/10)
    plot_title = "Vehicle Distance to Traffic Signal"
    plot_vehicle_speed_accel_downtrack_with_tsc(
        vehicles_plot_data_list,
        tsc_bar_height,
        max_y_axis,
        y_axis_stepsize,
        plot_title,
    )



def show_vehicles_profiles(vehicle_profiles_data):
    show_vehicles_speed_profiles(vehicle_profiles_data)
    # show_vehicles_accel_profiles(vehicle_profiles_data)
    show_vehicles_downtrack_profiles(vehicle_profiles_data)
    show_vehicles_distance2tf_profiles(vehicle_profiles_data)
    show_spats_maps_frequency_profiles(vehicle_profiles_data)


def load_bags():
    """Read the bags given the bags paths. Populate the vehicle profile data python object, and add the object to a list of vehicle profiles.

    Returns:
        _type_: list of vehicle_profile_data
    """
    bag_vehicle_names = {
        "data/CC-RG_BP_E1_G2_2022-04-01-20-58-58.bag": "DOT-45245",  # Black Pacifica: DOT-45245
        "data/CC-RG_BL_E1_R30_2022-04-01-20-27-36.bag": "DOT-45255"  # Blue Lexus: DOT-45255
    }
    print(
        "Found %d number(s) of engaged vehicle(s) profiles in the bags in TSMO uc2 use case."
        % len(bag_vehicle_names)
    )
    vehicle_profiles_data = load_rosbags_to_vehicles_profile(bag_vehicle_names)
    return vehicle_profiles_data


def show_bags():
    """
    Plot the vehicle information from the ROS bags generated by vehicles
    """
    vehicle_profiles_data = load_bags()
    show_vehicles_profiles(vehicle_profiles_data)
    plt.legend(loc="upper right")
    plt.show()


# Main Function; run all tests from here
# TODO: The contents of this main function provide some basic structure for loading data, but need not be followed if not applicable
def main():
    if len(sys.argv) < 2:
        print(
            "Need 1 arguments: process_bag.py <path to source folder with .bag files> "
        )
        exit()

    source_folder = sys.argv[1]

    # Re-direct the output of print() to a specified .txt file:
    orig_stdout = sys.stdout
    current_time = datetime.datetime.now()
    text_log_filename = "Results_" + str(current_time) + ".txt"
    text_log_file_writer = open(text_log_filename, "w")
    sys.stdout = text_log_file_writer

    # Create .csv file to make it easier to view overview of results (the .txt log file is still used for more in-depth information):
    csv_results_filename = "Results_" + str(current_time) + ".csv"
    csv_results_writer = csv.writer(open(csv_results_filename, "w"))

    # TODO update this list
    csv_results_writer.writerow(
        [
            "Bag Name",
            "Vehicle Name",
            "Test Type",
            "WZ-1 Result",
            "WZ-2 Result",
            "WZ-3 Result",
            "WZ-4 Result",
            "WZ-5 Result",
            "WZ-6 Result",
            "WZ-7 Result",
            "WZ-8 Result",
            "WZ-9 Result",
            "WZ-10 Result",
            "WZ-11 Result",
            "WZ-12 Result",
            "WZ-13 Result",
            "WZ-14 Result",
            "WZ-15 Result",
            "WZ-16 Result",
            "WZ-17 Result",
            "WZ-18 Result",
            "WZ-19 Result",
            "WZ-20 Result",
            "WZ-21 Result",
            "WZ-22 Result",
            "WZ-23 Result",
            "WZ-24 Result",
            "WZ-25 Result",
        ]
    )

    # TODO identify cases and assign bags to each case
    # Create list of Red Light Workzone Black Pacifica bag files to be processed
    black_pacifica_red_bag_files = []

    # Create list of Red Light Workzone Ford Fusion bag files to be processed
    ford_fusion_red_bag_files = [
        "_2021-09-22-16-00-28-red.bag",
        "_2021-09-22-19-03-26-red.bag",
    ]

    # Create list of Red Light Workzone Blue Lexus bag files to be processed
    blue_lexus_red_bag_files = []

    # Create list of Green Light Workzone Black Pacifica bag files to be processed
    black_pacifica_green_bag_files = []

    # Create list of Green Light Workzone Ford Fusion bag files to be processed
    ford_fusion_green_bag_files = []

    # Create list of Green Light Workzone Blue Lexus bag files to be processed
    blue_lexus_green_bag_files = []

    # Concatenate all Basic Travel bag files into one list
    red_light_bag_files = (
        black_pacifica_red_bag_files
        + ford_fusion_red_bag_files
        + blue_lexus_red_bag_files
    )
    green_light_bag_files = (
        black_pacifica_green_bag_files
        + ford_fusion_green_bag_files
        + blue_lexus_green_bag_files
    )
    WZ_bag_files = red_light_bag_files + green_light_bag_files

    # Loop to conduct data anlaysis on each bag file:
    for bag_file in WZ_bag_files:
        print("*****************************************************************")
        print("Processing new bag: " + str(bag_file))
        # TODO update bags groups
        if bag_file in black_pacifica_red_bag_files:
            print(
                "Black Pacifica Red Light Workzone Test Case"
            )  # TODO update conditionals here
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
        print(
            "Processing bag file "
            + str(bag_file)
            + " ("
            + str(WZ_bag_files.index(bag_file) + 1)
            + " of "
            + str(len(WZ_bag_files))
            + ")"
        )
        sys.stdout = text_log_file_writer

        # Process bag file if it exists and can be processed, otherwise skip and proceed to next bag file
        try:
            print("Starting To Process Bag at " + str(datetime.datetime.now()))
            bag_file_path = str(source_folder) + "/" + bag_file
            bag = rosbag.Bag(bag_file_path)
            print("Finished Processing Bag at " + str(datetime.datetime.now()))
        except:
            print("Skipping " + str(bag_file) + ", unable to open or process bag file.")
            continue

        # Get the rosbag times associated with the starting engagement and ending engagement for the Basic Travel use case test
        # TODO update to remove geofence information
        print("Getting engagement times at " + str(datetime.datetime.now()))
        # TODO get engagement times
        # time_test_start_engagement, time_test_end_engagement, found_test_times = get_test_case_engagement_times(bag)
        print("Got engagement times at " + str(datetime.datetime.now()))
        found_test_times = True  # TODO remove these placeholder values
        time_test_start_engagement = 0.0
        time_test_end_engagement = 1.0
        if not found_test_times:
            print(
                "Could not find test case engagement start and end times in bag file."
            )
            continue

        # Debug Statements
        print("Engagement starts at " + str(time_test_start_engagement.to_sec()))
        print("Engagement ends at " + str(time_test_end_engagement.to_sec()))
        print(
            "Time spent engaged: "
            + str((time_test_end_engagement - time_test_start_engagement).to_sec())
            + " seconds"
        )

        # TODO implement
        original_speed_limit = 1.0  # TODO remove placeholder
        # original_speed_limit = get_route_original_speed_limit(bag, time_test_start_engagement) # Units: m/s
        print("Original Speed Limit is " + str(original_speed_limit) + " m/s")

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

        # TODO call metric functions

        # Write simple pass/fail results to .csv file for appropriate row:
        csv_results_writer.writerow(
            [
                bag_file,
                vehicle_name,
                vehicle_role,
                wz_1_result,
                wz_2_result,
                wz_3_result,
                wz_4_result,
                wz_5_result,
                wz_6_result,
                wz_7_result,
                wz_8_result,
                wz_9_result,
                wz_10_result,
                wz_11_result,
                wz_12_result,
                wz_13_result,
                wz_14_result,
                wz_15_result,
                wz_16_result,
                wz_17_result,
                wz_18_result,
                wz_19_result,
                wz_20_result,
                wz_21_result,
                wz_22_result,
                wz_23_result,
                wz_24_result,
                wz_25_result,
            ]
        )

    sys.stdout = orig_stdout
    text_log_file_writer.close()
    return


if __name__ == "__main__":
    # main()
    show_bags()
