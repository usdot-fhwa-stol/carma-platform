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
#include <light_controlled_intersection_config.h>
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

namespace light_controlled_intersection_transit_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

/**
 * \brief Class containing primary business logic for the Light Controlled Intersection Tactical Plugin
 * 
 */

class LightControlledIntersectionTacticalPlugin
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
  LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm,const LightControlledIntersectionTacticalPluginConfig& config,
                         const PublishPluginDiscoveryCB &plugin_discovery_publisher);

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
   * \brief Converts a set of requested light controlled intersection maneuvers to point speed limit pairs. 
   * 
   * \param maneuvers The list of maneuvers to convert
   * 
   * \param wm Pointer to intialized world model for semantic map access
   * * \param state The current state of the vehicle
   * 
   * \return List of centerline points paired with target speeds TODO: to be implemented later
   */
  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                             const carma_wm::WorldModelConstPtr& wm,
                                                             const cav_msgs::VehicleState& state);
  
   /**
   * \brief Creates a speed profile according to case one of the light controlled intersection, where the vehicle accelerates and then decelerates into the intersection. 
   * \param wm Pointer to intialized world model for semantic map access
   * 
   * \return List of centerline points paired with target speeds: TODO: to be implemented later
   */
  std::vector<PointSpeedPair> create_case_one_speed_profile();
  
     /**
   * \brief Creates a speed profile according to case two of the light controlled intersection, 
   * where the vehicle first accelerates then cruises and finally decelerates into the intersection. 
   * 
   * \return List of centerline points paired with speed limits TODO: to be implemented later
   */
  std::vector<PointSpeedPair> create_case_two_speed_profile();

  /**
   * \brief Creates a speed profile according to case three of the light controlled intersection, 
   * where the vehicle decelerates and then accelerates into the intersection. 
   * 
   * \return List of centerline points paired with speed limits TODO: to be implemented later
   */
  std::vector<PointSpeedPair> create_case_three_speed_profile();

    /**
   * \brief Creates a speed profile according to case four of the light controlled intersection, 
   * where the vehicle first decelerates then cruises and finally accelerates into the intersection. 
   * 
   * \return List of centerline points paired with speed limits TODO: to be implemented later
   */
  std::vector<PointSpeedPair> create_case_four_speed_profile();

   /**
   * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning
   * 
   * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
   *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
   * \param state The current state of the vehicle
   * \param state_time The abosolute time which the provided vehicle state corresponds to
   * 
   * \return A list of trajectory points to send to the carma planning stack TODO: to be implemented later
   */ 
  std::vector<cav_msgs::TrajectoryPlanPoint> compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time); 
  
    
    /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();

      /**
    * \brief Function to find speed limit of a lanelet
    *
    * \param llt inout lanelet
    *
    * \return speed limit value
    */
  double findSpeedLimit(const lanelet::ConstLanelet& llt) const;

    /**
   * \brief calculate the speed, right before the car starts to decelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param speed_limit speed_limit of vehicle
   *
   * \return speed value
   */
  double calcSpeedBeforeDecel(double entry_time, double entry_dist, double current_speed, double speed_limit) const;

  /**
   * \brief calculate the speed, right before the car starts to accelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param speed_limit speed_limit of vehicle
   *
   * \return speed value
   */
  double calcSpeedBeforeAccel(double entry_time, double entry_dist, double current_speed, double speed_limit) const;

    /**
   * \brief Determine the speed profile case fpr approaching an intersection
   * 
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   *
   * \param schedule_entry_time scheduled stop time
   *
   * \param speed_limit speed limit
   *
   * \return integer case number
   */
  int determineSpeedProfileCase(double entry_dist, double current_speed, double schedule_entry_time, double speed_limit);

  /**
   * \brief calculate the time vehicle will enter to intersection with optimal deceleration
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param speed_limit speed_limit of vehicle
   *
   * \return the time vehicle will stop with optimal decelarion
   */
  double calcEstimatedEntryTimeLeft(double entry_dist, double current_speed, double speed_limit) const;

   /**
   * \brief Helper method which calls carma_wm::WorldModel::getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only,
   * bounds_inclusive) and throws and exception if the returned list of lanelets is empty. See the referenced method for additional
   * details on parameters.
   */
  std::vector<lanelet::ConstLanelet> getLaneletsBetweenWithException(double start_downtrack, double end_downtrack,
                                                                     bool shortest_path_only = false,
                                                                     bool bounds_inclusive = true) const;

  ////////// VARIABLES ///////////
  // CARMA Streets Variables
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

  // approximate speed limit 
  double speed_limit_ = 100.0;

  // downtrack of host vehicle
  double current_downtrack_ = 0.0;

  private:

  carma_wm::WorldModelConstPtr wm_;
  LightControlledIntersectionTacticalPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
 
  cav_msgs::Plugin plugin_discovery_msg_;
  carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg_;

  std::string light_controlled_intersection_strategy_ = "Carma/light_controlled_intersection";

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with
};
};