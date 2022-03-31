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
#include <carma_wm/CARMAWorldModel.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/optional.hpp>
#include "stop_and_wait_config.h"
#include <basic_autonomy/basic_autonomy.h>
#include <basic_autonomy/helper_functions.h>

namespace stop_and_wait_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
/**
 * \brief Convenience class for pairing 2d points with speeds
 */
struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

class StopandWait
{
public:
  /**
   * \brief Constructor
   */
  StopandWait(carma_wm::WorldModelConstPtr wm, StopandWaitConfig config,
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
   * \brief Method meant to be called periodically to trigger plugin discovery behavior
   */
  bool spinCallback();

  /**
   * \brief General entry point to begin the operation of this class
   */
  void run();

  /**
   * \brief Converts a set of requested STOP_AND_WAIT maneuvers to point speed limit pairs.
   *
   * \param maneuvers The list of maneuvers to convert
   * \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to
   * the vehicle position or earlier. If the first maneuver exceeds this then it's downtrack will be shifted to this
   * value.
   *
   * ASSUMPTION: Since the vehicle is trying to stop the assumption made is that the speed limit is irrelevant. 
   * ASSUMPTION: The provided maneuver lies on the route shortest path
   * 
   * \return List of centerline points paired with speed limits. All output points will have speed matching state.logitudinal_velocity
   */
  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                  const carma_wm::WorldModelConstPtr& wm,
                                                  const cav_msgs::VehicleState& state);
  /**
   * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of
   * trajectory points for trajectory planning
   *
   * \param points The set of points that define the current lane the vehicle is in and are defined based on the request
   * planning maneuvers. These points must be in the same lane as the vehicle and must extend in front of it though it
   * is fine if they also extend behind it. \param state The current state of the vehicle
   * \param initial_speed Returns the initial_speed used to generate the trajectory
   * \return A list of trajectory points to send to the carma planning stack
   */
  std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
      const std::vector<PointSpeedPair>& points, double starting_downtrack, double starting_speed, double stop_location,
      double stop_location_buffer, ros::Time start_time, double stopping_acceleration, double& initial_speed);

  /**
   * \brief Helper method to split a list of PointSpeedPair into separate point and speed lists
   */
  void splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                            std::vector<double>* speeds) const;

  std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_from_points_times_orientations(
      const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times,
      const std::vector<double>& yaws, ros::Time startTime);

private:

  double epsilon_ = 0.001; //small constant to compare double

  // pointer to the actual wm object
  carma_wm::WorldModelConstPtr wm_;
  StopandWaitConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
  
  cav_msgs::Plugin plugin_discovery_msg_;
  
};
}  // namespace stop_and_wait_plugin
