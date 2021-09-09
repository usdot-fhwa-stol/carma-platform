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
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <functional>
#include <stop_controlled_intersection_config.h>
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <cav_msgs/Maneuver.h>
#include <basic_autonomy/basic_autonomy.h>
#include <basic_autonomy/helper_functions.h>


/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                        ((mvr).type == cav_msgs::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))

namespace stop_controlled_intersection_transit_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

/**
 * \brief Class containing primary business logic for the Stop Controlled Intersection Tactical Plugin
 * 
 */

class StopControlledIntersectionTacticalPlugin
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
  StopControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm, StopControlledIntersectionTacticalPluginConfig config,
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

  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                             const carma_wm::WorldModelConstPtr& wm,
                                                             const cav_msgs::VehicleState& state);
  
  std::vector<PointSpeedPair> create_case_one_speed_profile(const carma_wm::WorldModelConstPtr& wm, const cav_msgs::Maneuver& maneuver,
                                                            std::vector<lanelet::BasicPoint2d> route_geometry_points, double starting_speed);
    
  std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time); 
  
    
    /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();

  private:

  carma_wm::WorldModelConstPtr wm_;
  StopControlledIntersectionTacticalPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
 
  cav_msgs::Plugin plugin_discovery_msg_;
  carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg_;

  std::string stop_controlled_intersection_strategy_ = "Carma/stop_controlled_intersection";

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with
  

};
};