#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <ros/ros.h>
#include <string>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/Plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include <carma_wm/Geometry.h>

#include <lanelet2_extension/regulatory_elements/CarmaTrafficLight.h>
#include "wz_strategic_plugin/wz_state_transition_table.h"

namespace wz_strategic_plugin
{
class WzStrategicPlugin
{
public:
  /**
   * \brief Constructor
   *
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   */
  WzStrategicPlugin(carma_wm::WorldModelConstPtr wm, WzStrategicPluginConfig config);

  /**
   * \brief Service callback for arbitrator maneuver planning
   * \param req Plan maneuver request
   * \param resp Plan maneuver response with a list of maneuver plan
   * \return If service call successed
   */
  bool planManeuverCb(const cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp);

  /**
   * \brief Returns the current plugin discovery message reflecting system status
   *
   * \return cav_msgs::Plugin The plugin discovery message
   */
  cav_msgs::Plugin getDiscoveryMsg();

private:
  /**
   * \brief Struct representing a vehicle state for the purposes of planning
   */
  struct VehicleState
  {
    ros::Time stamp;      // Timestamp of this state data
    double downtrack;     // The downtrack of the vehicle along the route at time stamp
    double speed;         // The speed of the vehicle at time stamp
    lanelet::Id lane_id;  // The current lane id of the vehicle at time stamp
  }

  /**
   * \brief Compose a lane keeping maneuver message based on input params
   *
   * \param start_dist Start downtrack distance of the current maneuver
   * \param end_dist End downtrack distance of the current maneuver
   * \param start_speed Start speed of the current maneuver
   * \param target_speed Target speed pf the current maneuver, usually it is the lanelet speed limit
   * \param start_time The starting time of the maneuver
   * \param end_time The ending time of the maneuver
   * \param lane_ids List of lanelet IDs that the current maneuver traverses. Message expects these to be contiguous and
   * end to end
   * 
   * \return A lane keeping maneuver message which is ready to be published
   */
  cav_msgs::Maneuver
  composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed,
                                      ros::Time start_time, ros::Time end_time, std::vector<lanelet::Id> lane_ids);

  cav_msgs::Maneuver composeStopAndWaitManeuverMessage(double current_dist, double& end_dist, double start_speed,
                                                       lanelet::Id& starting_lane_id, lanelet::Id& ending_lane_id,
                                                       ros::Time time, double& time_to_stop);

  cav_msgs::Maneuver composeIntersectionTransitMessage(double& start_dist, double& end_dist, double& start_speed,
                                                       double& target_speed, ros::Time start_time,
                                                       lanelet::Id& starting_lane_id);

  bool supportedLightState(lanelet::CarmaTrafficLightState state) const;

  int traffic_light_interpreter(boost::optional<lanelet::CarmaTrafficLightState> state);

  double estimate_distance_to_stop(double v, double a);

  double estimate_time_to_stop(double d, double v);

  double findSpeedLimit(const lanelet::ConstLanelet& llt);

  carma_wm::WorldModelConstPtr wm_;

  WzStrategicPluginConfig config_;

  // Plugin discovery message
  cav_msgs::Plugin plugin_discovery_msg_;

  // TODO make intersection variables members

  std::string planning_strategic_plugin_ = "WorkZonePlugin";
  std::string intersection_transit_planning_tactical_plugin_ = "IntersectionTransitPlugin";

  // Cache variables for storing the current intersection state between state machine transitions
  boost::optional<double> intersection_speed_;
  boost::optional<double> intersection_end_downtrack_;

  WorkZoneStateTransitionTable transition_table_;
};
}  // namespace wz_strategic_plugin