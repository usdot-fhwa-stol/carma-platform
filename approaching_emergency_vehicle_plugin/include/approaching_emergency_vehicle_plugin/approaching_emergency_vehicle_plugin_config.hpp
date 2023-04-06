#pragma once

/*
 * Copyright (C) 2022-2023 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

#include <iostream>
#include <vector>

namespace approaching_emergency_vehicle_plugin
{

  /**
   * \brief Stuct containing the algorithm configuration values for approaching_emergency_vehicle_plugin
   */
  struct Config
  {
    double passing_threshold = 15.0;               // (Seconds) If the estimated duration until an ERV passes the ego vehicle is below this, the
                                                   // ERV is considered to be actively passing the ego vehicle.

    double approaching_threshold = 60.0;           // (Seconds) If the estimated duration until an ERV passes the ego vehicle is below this, the
                                                   // ERV is considered to be approaching the ego vehicle.

    double finished_passing_threshold = 152.4;     // (Meters) A threshold; an actively-passing ERV is considered to have completed passing the ego vehicle when
                                                   // its distance in front of the ego vehicle reaches this value.

    double min_lane_following_duration_before_lane_change = 3.0; // (Seconds) The minimum duration of lane following that must be planned before a lane change when in the 
                                                                 // MOVING_OVER_FOR_APPROACHING_ERV state.

    double bsm_processing_frequency = 1.0;         // (Hz) The rate that incoming BSMs from a specific ERV will be processed by this plugin.

    double speed_limit_reduction_during_passing = 4.4704; // (m/s) The amount that the speed limit of a lanelet will be reduced by when planning a maneuver when an
                                                          // ERV is actively passing the ego vehicle.

    double minimum_reduced_speed_limit = 2.2352;   // (m/s) The minimum speed limit that a lanelet's speed limit will be reduced to when an ERV is actively passing
                                                   // the ego vehicle.

    double default_speed_limit = 2.2352;           // (m/s) The default speed limit used when a lanelet does not have a specified speed limit in the map.

    double reduced_speed_buffer = 1.1176;          // (m/s) A buffer value; if the ego vehicle speed is within this speed of its target speed when slowing down for an actively passing
                                                   // ERV, then this plugin will state in its approaching ERV status message that the ego vehicle has finished slowing down.

    double timeout_check_frequency = 2.0;          // (Hz) The frequency at which this plugin will check whether a timeout has occurred for the 
                                                   // currently-tracked ERV.
  
    double timeout_duration = 5.0;                 // (Seconds) If no BSM has been received from the currently-tracked ERV, than the ERV will no longer be tracked by
                                                   // this plugin. 

    double minimal_plan_duration = 15.0;           // (Seconds) The minimal duration of a generated maneuver plan.

    double buffer_distance_before_stopping = 45.0; // (Meters) The distance that the beginning of a stop_and_wait maneuver can be extended by.

    double stopping_accel_limit_multiplier = 0.5;  // (m/s^2) Multiplier for the acceleration limit which will be used for formulating the stopping maneuver
                                                   // NOTE: This multiplier should be lower than the value used by the plugin that will be used to bring the vehicle to a stop to provide buffer in planning

    double vehicle_acceleration_limit = 2.0;       // (m/s^2) The vehicle acceleration limit configured for the CARMA System

    double route_end_point_buffer = 10.0;          // (Meters) The distance from the route end point in which the trajectory planner will attempt to stop the ego vehicle for a stop and wait maneuver.

    double approaching_erv_status_publication_frequency = 1.0;  // (Hz) The frequency at which this plugin will publish status updates to the Web UI that describe the estimated time until an approaching ERV
                                                                // passes the ego vehicle, and a description of the ego vehicle's path plan in response to the approaching ERV.
    
    double warning_broadcast_frequency = 1.0;      // (Hz) The frequency at which this plugin will broadcast EmergencyVehicleResponse warning messages to the currently-tracked ERV
                                                   // when the ego vehicle is in the approaching ERV's path but is unable to change lanes.

    int max_warning_broadcasts = 5;                // The maximum number of times that an EmergencyVehicleResponse warning message will be broadcasted to an ERV if an
                                                   // EmergencyVehicleAck message is not received from the ERV.

    double vehicle_length = 4.0;                   // (Meters) The length of the host vehicle from its front bumper to its rear bumper.

    std::string lane_following_plugin = "inlanecruising_plugin"; // (No Units) The tactical plugin being used for lane following.

    std::string lane_change_plugin = "cooperative_lanechange";   // (No Units) The tactical plugin being used for lane changes.

    std::string stop_and_wait_plugin = "stop_and_wait_plugin";   // (No Units) The tactical plugin being used for stop and wait maneuvers.

    std::string vehicle_id = "DEFAULT_VEHICLE_ID"; // The ego vehicle's static ID, which is its license plate

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "approaching_emergency_vehicle_plugin::Config { " << std::endl
           << "passing_threshold: " << c.passing_threshold << std::endl
          //  << "do_not_move_over_threshold: " << c.do_not_move_over_threshold << std::endl
           << "approaching_threshold: " << c.approaching_threshold << std::endl
           << "finished_passing_threshold: " << c.finished_passing_threshold << std::endl
           << "min_lane_following_duration_before_lane_change: " << c.min_lane_following_duration_before_lane_change << std::endl
           << "bsm_processing_frequency: " << c.bsm_processing_frequency << std::endl
           << "speed_limit_reduction_during_passing: " << c.speed_limit_reduction_during_passing << std::endl
           << "minimum_reduced_speed_limit: " << c.minimum_reduced_speed_limit << std::endl
           << "default_speed_limit: " << c.default_speed_limit << std::endl
           << "reduced_speed_buffer: " << c.reduced_speed_buffer << std::endl
           << "timeout_check_frequency: " << c.timeout_check_frequency << std::endl
           << "timeout_duration: " << c.timeout_duration << std::endl
           << "minimal_plan_duration: " << c.minimal_plan_duration << std::endl
           << "buffer_distance_before_stopping: " << c.buffer_distance_before_stopping << std::endl 
           << "stopping_accel_limit_multiplier: " << c.stopping_accel_limit_multiplier << std::endl 
           << "vehicle_acceleration_limit: " << c.vehicle_acceleration_limit << std::endl 
           << "route_end_point_buffer: " << c.route_end_point_buffer << std::endl
           << "approaching_erv_status_publication_frequency: " << c.approaching_erv_status_publication_frequency << std::endl
           << "warning_broadcast_frequency: " << c.warning_broadcast_frequency << std::endl 
           << "max_warning_broadcasts: " << c.max_warning_broadcasts << std::endl 
           << "lane_following_plugin: " << c.lane_following_plugin << std::endl
           << "lane_change_plugin: " << c.lane_change_plugin << std::endl
           << "stop_and_wait_plugin: " << c.stop_and_wait_plugin << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // approaching_emergency_vehicle_plugin