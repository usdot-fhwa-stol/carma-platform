#pragma once

/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <unordered_set>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <basic_autonomy/basic_autonomy.h>

#include "platooning_tactical_plugin_config.h"

namespace platooning_tactical_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using DebugPublisher = std::function<void(const carma_debug_msgs::TrajectoryCurvatureSpeeds&)>;
/**
 * \brief Convenience class for pairing 2d points with speeds
 */ 
struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

/**
 * \brief Class containing primary business logic for the Platooning Tactical Plugin
 * 
 */ 
class PlatooningTacticalPlugin
{
public:
  /**
   * \brief Constructor
   * 
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
   */ 
  PlatooningTacticalPlugin(carma_wm::WorldModelConstPtr wm, PlatooningTacticalPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher);

  /**
   * \brief Service callback for trajectory planning
   * 
   * \param req The service request
   * \param resp The service response
   * 
   * \return True if success. False otherwise
   */ 
  bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

  /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();


  cav_msgs::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

private:
  carma_wm::WorldModelConstPtr wm_;
  PlatooningTacticalPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;

  cav_msgs::Plugin plugin_discovery_msg_;
  carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg_;
  DebugPublisher debug_publisher_;

  cav_msgs::VehicleState ending_state_before_buffer; //state before applying extra points for curvature calculation that are removed later
};
};  // namespace platooning_tactical_plugin