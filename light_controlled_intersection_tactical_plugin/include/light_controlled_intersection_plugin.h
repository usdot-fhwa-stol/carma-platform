#pragma once

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
using GeneralTrajConfig = basic_autonomy::waypoint_generation::GeneralTrajConfig;
using DetailedTrajConfig = basic_autonomy::waypoint_generation::DetailedTrajConfig;

enum SpeedProfileCase {
    ACCEL_CRUISE_DECEL = 1,
    ACCEL_DECEL = 2,
    DECEL_ACCEL = 3,
    DECEL_CRUISE_ACCEL = 4,
};

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
   * \brief Creates a speed profile according to case one or two of the light controlled intersection, where the vehicle accelerates (then cruises if needed) and decelerates into the intersection. 
   * \param wm world_model pointer
   * \param List of centerline points paired with speed limits whose speeds are to be modified:
   * \param start_dist starting downtrack of the maneuver to be planned (excluding buffer points)
   * \param end_dist ending downtrack of the maneuver to be planned (excluding buffer points)
   * \param remaining_time time interval left for scheduled entry into the intersection
   * \param starting_speed starting speed at the start of the maneuver
   * \param speed_before_decel highest speed desired between acceleration and decelaration
   * \param departure_speed ending speed of the maneuver a.k.a entry speed into the intersection
   * NOTE: Cruising speed profile is applied (case 1) if speed before deceleration is higher than speed limit. Otherwise Case 2.
   * NOTE: when applying the speed profile, the function ignores buffer points beyond start_dist and end_dist. Internally uses: config_.back_distance and speed_limit_
   */
  void apply_accel_cruise_decel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double end_dist, 
                                    double remaining_time, double starting_speed, double speed_before_decel, double departure_speed);

   /**
   * \brief Creates a speed profile according to case three or four of the light controlled intersection, where the vehicle decelerates (then cruises if needed) and accelerates into the intersection. 
   * \param wm world_model pointer
   * \param List of centerline points paired with speed limits whose speeds are to be modified:
   * \param start_dist starting downtrack of the maneuver to be planned (excluding buffer points)
   * \param end_dist ending downtrack of the maneuver to be planned (excluding buffer points)
   * \param remaining_time time interval left for scheduled entry into the intersection
   * \param starting_speed starting speed at the start of the maneuver
   * \param speed_before_accel highest speed desired between deceleration and acceleration
   * \param departure_speed ending speed of the maneuver a.k.a entry speed into the intersection
   * NOTE: Cruising speed profile is applied (case 4) if speed before acceleration is lower than minimum speed allowed. Otherwise Case 3.
   * NOTE: when applying the speed profile, the function ignores buffer points beyond start_dist and end_dist. Internally uses: config_.back_distance and min_speed_allowed_
   */
  void apply_decel_cruise_accel_speed_profile(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double end_dist, 
                                    double remaining_time, double starting_speed, double speed_before_accel, double departure_speed);

  /**
   * \brief Apply optimized target speeds to the trajectory determined for fixed-time and actuated signals.
   *        Based on TSMO USE CASE 2. Chapter 2. Trajectory Smoothing
   * \param maneuver Maneuver associated that has starting downtrack and desired entry time
   * \param starting_speed Starting speed of the vehicle
   * \param points The set of points with raw speed limits whose speed profile to be changed.
   *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
   *              
   */
  void apply_optimized_target_speed_profile(const cav_msgs::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds);

  /**
  * \brief Creates geometry profile to return a point speed pair struct for INTERSECTION_TRANSIT maneuver types (by converting it to LANE_FOLLOW)
  * \param maneuvers The list of maneuvers to convert to geometry points and calculate associated raw speed limits
  *  \param max_starting_downtrack The maximum downtrack that is allowed for the first maneuver. This should be set to the vehicle position or earlier.
  *                               If the first maneuver exceeds this then it's downtrack will be shifted to this value. 
  * \param wm Pointer to intialized world model for semantic map access
  * \param ending_state_before_buffer reference to Vehicle state, which is state before applying extra points for curvature calculation that are removed later
  * \param state The vehicle state at the time the function is called
  * \param general_config Basic autonomy struct defined to load general config parameters from tactical plugins
  * \param detailed_config Basic autonomy struct defined to load detailed config parameters from tactical plugins
  * \return A vector of point speed pair struct which contains geometry points as basicpoint::lanelet2d and speed as a double for the maneuver
  * NOTE: This function is a slightly modified version of the same function in basic_autonomy library and currently only plans for the first maneuver
  */
  std::vector<PointSpeedPair> create_geometry_profile(const std::vector<cav_msgs::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
                                                                   cav_msgs::VehicleState &ending_state_before_buffer,const cav_msgs::VehicleState& state,
                                                                   const GeneralTrajConfig &general_config, const DetailedTrajConfig &detailed_config);

    /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   */ 

  void onSpin();

    /**
   * \brief calculate the speed, right before the car starts to decelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return speed value
   */
  double calcSpeedBeforeDecel(double entry_time, double entry_dist, double current_speed, double departure_speed) const;

  /**
   * \brief calculate the speed, right before the car starts to accelerate for timed entry into the intersection
    *
   * \param entry_time time the vehicle must stop
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return speed value
   */
  double calcSpeedBeforeAccel(double entry_time, double entry_dist, double current_speed, double departure_speed) const;

    /**
   * \brief Determine the speed profile case for approaching an intersection. 
   *        It internally utilizes config_.min_speed and speed_limit_ to determine upper and lower speed bounds
   * 
   * \param estimated_entry_time estimated time to enter the intersection without speed modification
   * 
   * \param scheduled_entry_time scheduled time to enter the intersection
   *
   * \param speed_before_decel speed before starting to decelerate (applicable in case 1, 2)
   * 
   * \param  speed_before_accel speed before starting to accelerate (applicable in case 1, 2)
   *
   * \return integer case number
   */
  SpeedProfileCase determineSpeedProfileCase(double estimated_entry_time, double scheduled_entry_time, double speed_before_decel, double speed_before_accel);


  /**
   * \brief calculate the time vehicle will enter to intersection with optimal deceleration
   *
   * \param entry_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param departure_speed speed to get into the intersection
   *
   * \return the time vehicle will stop with optimal decelarion
   */
  double calcEstimatedEntryTimeLeft(double entry_dist, double current_speed, double departure_speed) const;

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

  cav_msgs::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

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