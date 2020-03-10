#pragma once
/*
 * Copyright (C) 2020 LEIDOS.
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
#include <lanelet2_core/primitives/Point.h>
#include "GeofenceSchedule.h"

namespace carma_wm_ctrl
{
/**
 * @brief An object representing a geofence use for communications with CARMA Cloud
 *
 * TODO: This is currently a place holder class which needs to be updated based on the final geofence specification
 */
class Geofence
{
public:
  uint32_t id_;  // Unique id of this geofence. TODO use id matching geofence standard

  GeofenceSchedule schedule;  // The schedule this geofence operates with

  //// TODO Add attributes provided by geofences
};
}  // namespace carma_wm_ctrl