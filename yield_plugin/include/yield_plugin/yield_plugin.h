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
#include "yield_config.h"
#include <unordered_set>
#include <carma_wm/WorldModel.h>
#include <carma_wm/collision_detection.h>
#include <trajectory_utils/quintic_coefficient_calculator.h>

namespace yield_plugin
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

/**
 * \brief Class containing primary business logic for the In-Lane Cruising Plugin
 * 
 */ 
class YieldPlugin
{
public:
  /**
   * \brief Constructor
   * 
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
   */ 
  YieldPlugin(carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher);

  /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();

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
   * \brief trajectory is modified to safely avoid obstacles on the road
   * \param original_tp original trajectory plan without object avoidance
   * \param current_speed_ current speed of the vehicle
   * \return modified trajectory plan
   */
  cav_msgs::TrajectoryPlan update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, double current_speed_);

  /**
   * \brief calculate quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of polynomial for given input
   */
  double polynomial_calc(std::vector<double> coeff, double x);

  /**
   * \brief calculate derivative of quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of derivative polynomial for given input
   */
  double polynomial_calc_d(std::vector<double> coeff, double x);

  /**
   * \brief calculates the maximum speed in a set of tajectory points
   * \param trajectory_points trajectory points
   * \return maximum speed
   */
  double max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points) ;

  /**
   * \brief calculates distance between trajectory points in a plan
   * \param trajectory_plan input trajectory plan 
   * \return vector of relative distances between trajectory points
   */                     
  std::vector<double> get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan);  


  //temppp
  
  
private:

  carma_wm::WorldModelConstPtr wm_;
  YieldPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;

  cav_msgs::Plugin plugin_discovery_msg_;
  geometry_msgs::Vector3 host_vehicle_size;
  double current_speed_;

};
};  // namespace yield_plugin
