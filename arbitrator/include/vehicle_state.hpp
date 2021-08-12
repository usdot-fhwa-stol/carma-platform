#ifndef __ARBITRATOR_INCLUDE_VEHICLE_STATE_HPP__
#define __ARBITRATOR_INCLUDE_VEHICLE_STATE_HPP__

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

#include <lanelet2_core/Forward.h>

namespace arbitrator
{
/**
 * \brief Struct defining the vehicle state required for maneuver planning
 */
struct VehicleState
{
  ros::Time stamp;                         // Time stamp of position data used to populate struct
  double x = 0;                            // Vehicle x axis position in map frame (m)
  double y = 0;                            // Vehicle y axis position in map frame (m)
  double downtrack = 0;                    // Vehicle route downtrack (m)
  double velocity = 0;                     // Vehicle logitudinal velocity
  lanelet::Id lane_id = lanelet::InvalId;  // Vehicle lane id based on downtrack
};
}  // namespace arbitrator

#endif  //__ARBITRATOR_INCLUDE_VEHICLE_STATE_HPP__