#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
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

    double do_not_move_over_threshold = 20.0;      // (Seconds) If the estimated duration until an ERV passes the ego vehicle is below this, the
                                                   // ERV will no longer attempt to change lanes in reaction to the approaching ERV

    double approaching_threshold = 60.0;           // (Seconds) If the estimated duration until an ERV passes the ego vehicle is below this, the
                                                   // ERV is considered to be approaching the ego vehicle.

    double bsm_processing_frequency = 1.0;         // (Hz) The rate that incoming BSMs from a specific ERV will be processed by this plugin.

    double speed_reduction_during_passing = 10.0;  // (m/s) The amount that the target speed of maneuvers will be reduced below the speed limit when an
                                                   // ERV is actively passing the ego vehicle.

    double minimum_reduced_speed = 10.0;           // (m/s) The minimum target speed that will be assigned to maneuvers when an ERV is actively passing
                                                   // the ego vehicle.

    double timeout_check_frequency = 2.0;          // (Hz) The frequency at which this plugin will check whether a timeout has occurred for the 
                                                   // currently-tracked ERV.
  
    double timeout_duration = 5.0;                 // (Seconds) If no BSM has been received from the currently-tracked ERV, than the ERV will no longer be tracked by
                                                   // this plugin. 

    std::string vehicle_id = "DEFAULT_VEHICLE_ID"; // The ego vehicle's static ID, which is its license plate

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "approaching_emergency_vehicle_plugin::Config { " << std::endl
           << "passing_threshold: " << c.passing_threshold << std::endl
           << "do_not_move_over_threshold: " << c.do_not_move_over_threshold << std::endl
           << "approaching_threshold: " << c.approaching_threshold << std::endl
           << "bsm_processing_frequency: " << c.bsm_processing_frequency << std::endl
           << "speed_reduction_during_passing: " << c.speed_reduction_during_passing << std::endl
           << "minimum_reduced_speed: " << c.minimum_reduced_speed << std::endl
           << "timeout_check_frequency: " << c.timeout_check_frequency << std::endl
           << "timeout_duration: " << c.timeout_duration << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // approaching_emergency_vehicle_plugin