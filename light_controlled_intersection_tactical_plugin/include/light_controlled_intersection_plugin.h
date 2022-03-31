#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
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

enum TSCase {
  CASE_1 = 1,
  CASE_2 = 2,
  CASE_3 = 3,
  CASE_4 = 4,
  CASE_5 = 5,
  CASE_6 = 6,
  CASE_7 = 7,
  CASE_8 = 8,
};

struct TrajectoryParams
{
  double a1_ = 0;
  double v1_ = 0;
  double x1_ = 0;

  double a2_ = 0;
  double v2_ = 0;
  double x2_ = 0;

  double a3_ = 0;
  double v3_ = 0;
  double x3_ = 0;
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
   * \param points_and_target_speeds of centerline points paired with speed limits whose speeds are to be modified:
   * \param start_dist starting downtrack of the maneuver to be planned (excluding buffer points) in m
   * \param remaining_dist distance for the maneuver to be planned (excluding buffer points) in m
   * \param starting_speed starting speed at the start of the maneuver in m/s
   * \param departure_speed ending speed of the maneuver a.k.a entry speed into the intersection m/s
   * \param tsp trajectory smoothing parameters

   * NOTE: Cruising speed profile is applied (case 1) if speed before deceleration is higher than speed limit. Otherwise Case 2.
   * NOTE: when applying the speed profile, the function ignores buffer points beyond start_dist and end_dist. Internally uses: config_.back_distance and speed_limit_
   */
  void apply_trajectory_smoothing_algorithm(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist, 
                                    double starting_speed, double departure_speed, TrajectoryParams tsp);

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
   * \brief Given a Lanelet, find it's associated Speed Limit
   *
   * \param llt Constant Lanelet object
   *
   * \throw std::invalid_argument if the speed limit could not be retrieved
   *
   * \return value of speed limit in mps
   */
  double findSpeedLimit(const lanelet::ConstLanelet& llt) const;
  
    /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   */ 

  void onSpin();

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
  double speed_limit_ = 11.176; //25mph by default
  boost::optional<TSCase> last_case_;
  boost::optional<bool> is_last_case_successful_;
  cav_msgs::TrajectoryPlan last_trajectory_;

  cav_msgs::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

  // downtrack of host vehicle
  double current_downtrack_ = 0.0;

  double last_successful_ending_downtrack_;         // if algorithm was successful, this is traffic_light_downtrack
  double last_successful_scheduled_entry_time_;     // if algorithm was successful, this is also scheduled entry time (ET in TSMO UC2 Algo)

  private:

  carma_wm::WorldModelConstPtr wm_;
  LightControlledIntersectionTacticalPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
 
  cav_msgs::Plugin plugin_discovery_msg_;
  carma_debug_msgs::TrajectoryCurvatureSpeeds debug_msg_;
  std::vector<double> last_final_speeds_;

  std::string light_controlled_intersection_strategy_ = "Carma/light_controlled_intersection";

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with
};
};