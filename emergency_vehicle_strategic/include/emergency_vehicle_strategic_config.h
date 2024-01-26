#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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

/*
 * Developed by the JFL Solutions LLC.
 * Author: Fang Zhou
 */
#include <iostream>

namespace emergency_vehicle_strategic
{

  /**
   * \brief Stuct containing the algorithm configuration values for the emergency_vehicle_strategic
   */
  struct EmergencyVehicleStrategicPluginConfig
  {
    /**
     * 
     * vehicleID (str):           
     *  The default vehicle ID.
     * 
     * lane_change_desired_steps (int): 
     *  Desired strategic planning steps to finish lane change. 
     * 
     * maxCrosstrackError (float, m):    
     *  Maximum threshold value of cross track difference 
     *  when determining vehicle lane positions.
     * 
     * time_step (float, s):            
     *  Time length of each strategic control step (default 15s).
     * 
     * lane_change_speed_adjustment (float): 
     *  A float ratio (0~1) to adjust vehicle current speed
     *  when emergency vehicle is detected.
     * 
     * maxLaneChangeDist (float, m):
     *  Maximum maneuver plan distance for lane change.
     * 
     * stopping_deceleration (float, m/s^2):
     *  The desired deceleration when performing emergency stopping.
     * 
     * lane_change_recheck_time (float, s):
     *  A brief time for host to follow current lane at a reduced speed 
     *  before check again for lane change to the right.
     * 
     */ 

    bool detecting_emergency_in_front   = false; // bool indicator left for front emergency vehicle detection
    std::string vehicleID               = "default_id";
    int lane_change_desired_steps       = 1;    
    double maxCrosstrackError           = 2.0;  // m
    double time_step                    = 15.0; // s
    double lane_change_speed_adjustment = 0.7; 
    double reduced_lane_follow_speed    = 1.0; 
    double maxLaneChangeDist            = 22.0; // m
    double stopping_deceleration        = 2.5;  // m/s^2
    double lane_change_recheck_time     = 1.5;  // s
    // parameters for lane change 
    double lane_width                   = 3.5;  // m; width of the lane
    double vehicle_length               = 5.0;    // m; Note: 20m with trailer; 7m if only tractor; 5m for Lincoln 
    double em_lane_ctd_check_ratio      = 1.0;  // ratio to determine successful lane change to emergency lane
    double em_lane_maintain_ratio       = 2.0;    // ratio to determine how long the host travel in emergency lane before stop

    double left_path_safety_distance    = 40.0;    // m; Note: 20m with trailer; 7m if only tractor; 5m for Lincoln 

    double following_lanelets_number    = 3;

    friend std::ostream& operator<<(std::ostream& output, const EmergencyVehicleStrategicPluginConfig& c)
    {
      output << "EmergencyVehicleStrategicPluginConfig { " << std::endl
            << "detecting_emergency_in_front: " << c.detecting_emergency_in_front << std::endl
            << "vehicleID: " << c.vehicleID << std::endl
            << "lane_change_desired_steps: " << c.lane_change_desired_steps << std::endl
            << "maxCrosstrackError: " << c.maxCrosstrackError << std::endl
            << "time_step: " << c.time_step << std::endl
            << "lane_change_speed_adjustment: " << c.lane_change_speed_adjustment << std::endl
            << "maxLaneChangeDist: " << c.maxLaneChangeDist << std::endl
            << "stopping_deceleration: " << c.stopping_deceleration << std::endl
            << "lane_change_recheck_time: " << c.lane_change_recheck_time << std::endl
            << "lane_width: " << c.lane_width << std::endl
            << "vehicle_length: " << c.vehicle_length << std::endl
            << "em_lane_ctd_check_ratio: " << c.em_lane_ctd_check_ratio << std::endl
            << "em_lane_maintain_ratio: " << c.em_lane_maintain_ratio << std::endl
            << "reduced_lane_follow_speed: " << c.reduced_lane_follow_speed << std::endl
            << "left_path_safety_distance: " << c.left_path_safety_distance << std::endl
            << "following_lanelets_number: " << c.following_lanelets_number << std::endl


            << "}" << std::endl; 
      return output;
    }
  };

}//emergency_vehicle_strategic