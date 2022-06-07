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

namespace carma_wm_ctrl
{

  /**
   * \brief Config struct
   */
  struct Config
  {
    int ack_pub_times = 1; // The number of times it publishes Geofence Acknowledgement.
    double max_lane_width = 4.0; // Max lane width in meters within which geofence points are associated to a lanelet as those points are guaranteed to apply to a single lane
    double traffic_control_request_period = 1.0; //Period in seconds between traffic control requests after route selection
    std::vector<double> intersection_coord_correction = {}; // Every element corresponds to intersection_id of every two elements (x,y) in intersection_coord_correction (id must be [0, +65535] ranges)
    std::vector<int64_t> intersection_ids_for_correction = {}; //Every 2 element describes coordinate correction [delta_x, delta_y] for each intersection_id in intersection_ids_for_correction in same order
    double config_limit = 6.67; //config speed limit in m/s
    std::string vehicle_id = "CARMA"; 
    std::string participant = "vehicle:car";
    
    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "WMBroadcaster::Config { " << std::endl
           << "ack_pub_times: " << c.ack_pub_times << std::endl
           << "max_lane_width: " << c.max_lane_width << std::endl
           << "traffic_control_request_period: " << c.traffic_control_request_period << std::endl
           << "intersection_coord_correction.size(): " << c.intersection_coord_correction.size() << std::endl
           << "intersection_ids_for_correction.size(): " << c.intersection_ids_for_correction.size() << std::endl
           << "vehicle_id: " << c.vehicle_id << std::endl
           << "participant: " << c.participant << std::endl
           << "config_limit: " << c.config_limit << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // carma_wm_ctrl