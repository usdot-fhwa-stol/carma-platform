#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <cav_msgs/MobilityOperation.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <basic_autonomy/basic_autonomy.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <functional>
#include "inlanecruising_config.h"
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

namespace inlanecruising_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using DebugPublisher = std::function<void(const carma_debug_msgs::TrajectoryCurvatureSpeeds&)>;
using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

/**
 * \brief Class containing primary business logic for the In-Lane Cruising Plugin
 * 
 */ 
class InLaneCruisingPlugin
{
public:
  /**
   * \brief Constructor
   * 
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
   * \param debug_publisher Callback which will publish a debug message. The callback defaults to no-op.
   */ 
  InLaneCruisingPlugin(carma_wm::WorldModelConstPtr wm, InLaneCruisingPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher, DebugPublisher debug_publisher=[](const auto& msg){});

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

  /**
   * \brief set the yield service
   * 
   * \param yield_srv input yield service
   */
  void set_yield_client(ros::ServiceClient& client);

   /**
   * \brief verify if the input yield trajectory plan is valid
   * 
   * \param yield_plan input yield trajectory plan
   *
   * \return true or falss
   */
  bool validate_yield_plan(const cav_msgs::TrajectoryPlan& yield_plan);

  /**
   * \brief parse strategy parameters
   * 
   * \param strategy_params input string of strategy params from mob op msg
   */
  void parse_strategy_params(const std::string& strategy_params);

  /**
   * \brief calculate the time vehicle will stop with optimal decelarion
   *
   * \param stop_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   *
   * \return the time vehicle will stop with optimal decelarion
   */
  double calc_estimated_stop_time(double stop_dist, double current_speed);

  /**
   * \brief calculate the speed, right before the car starts to decelerate for stopping
    *
   * \param stop_time time the vehicle must stop
   *
   * \param stop_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   *
   * \return speed value
   */
  double calc_stop_speed(double stop_time, double stop_dist, double current_speed);

  /**
   * \brief Determine the speed profile case fpr approaching an intersection
   * 
   * \param stop_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   *
   * \param schedule_stop_time scheduled stop time
   *
   * \param speed_limit speed limit
   *
   * \return integer case number
   */
  int determine_case(double stop_dist, double current_speed, double schedule_stop_time, double speed_limit);

  /**
   * \brief callback function for mobility operation
   * 
   * \param msg input mobility operation msg
   */
  void mob_op_cb(const cav_msgs::MobilityOperationConstPtr& msg);


  cav_msgs::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later
  
  // CARMA Streets Variakes
  // timestamp for msg received from carma streets
  uint32_t street_msg_timestamp_ = 0.0;
  // scheduled stop time
  uint32_t scheduled_stop_time_ = 0.0;
  // scheduled enter time
  uint32_t scheduled_enter_time_ = 0.0;
  // scheduled depart time
  uint32_t scheduled_depart_time_ = 0.0;
  // scheduled latest depart time
  uint32_t scheduled_latest_depart_time_ = 0.0;
  // flag to show if the vehicle is allowed in intersection
  bool is_allowed_int_ = false;



  
private:

  carma_wm::WorldModelConstPtr wm_;
  InLaneCruisingPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
  ros::ServiceClient yield_client_;

  cav_msgs::Plugin plugin_discovery_msg_;
  DebugPublisher debug_publisher_;
  carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg_;
  std::string stop_controlled_intersection_strategy_ = "Carma/stop_controlled_intersection";
  std::string previous_strategy_params_ = "";
  // approximate speed limit 
  double speed_limit_ = 100.0;

};



};  // namespace inlanecruising_plugin