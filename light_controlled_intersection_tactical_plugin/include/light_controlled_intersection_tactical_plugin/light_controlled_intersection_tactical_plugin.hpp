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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/geometry.hpp>
#include <boost/shared_ptr.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <autoware_msgs/msg/lane.hpp>
#include <carma_debug_ros2_msgs/msg/trajectory_curvature_speeds.hpp>
#include <basic_autonomy_ros2/basic_autonomy.hpp>
#include <basic_autonomy_ros2/helper_functions.hpp>

#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types. Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evalutes to the desired field
 */
#define GET_MANEUVER_PROPERTY(mvr, property)\
        (((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN ? (mvr).intersection_transit_left_turn_maneuver.property :\
            ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ? (mvr).intersection_transit_right_turn_maneuver.property :\
                ((mvr).type == carma_planning_msgs::msg::Maneuver::INTERSECTION_TRANSIT_STRAIGHT ? (mvr).intersection_transit_straight_maneuver.property :\
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))

namespace light_controlled_intersection_tactical_plugin
{
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

  struct TrajectoryParams {
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
   * \brief  Class containing primary business logic for the Light Controlled Intersection Tactical Plugin
   * 
   */
  class LightControlledIntersectionTacticalPlugin
  {

  private:    
    // World Model object
    carma_wm::WorldModelConstPtr wm_;

    // Config for this object
    Config config_;

    // Logger for this object
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

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

    double speed_limit_ = 11.176; // Approximate speed limit; 25 mph by default
    boost::optional<TSCase> last_case_;
    boost::optional<bool> is_last_case_successful_;
    carma_planning_msgs::msg::TrajectoryPlan last_trajectory_;

    carma_planning_msgs::msg::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later

    double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with

    // downtrack of host vehicle
    double current_downtrack_ = 0.0;

    double last_successful_ending_downtrack_;         // if algorithm was successful, this is traffic_light_downtrack
    double last_successful_scheduled_entry_time_;     // if algorithm was successful, this is also scheduled entry time (ET in TSMO UC2 Algo)

    carma_planning_msgs::msg::Plugin plugin_discovery_msg_;
    carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds debug_msg_;
    std::vector<double> last_final_speeds_;

    std::string light_controlled_intersection_strategy_ = "signalized"; // Strategy carma-streets is sending. Could be more verbose but needs to be changed on both ends

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
    void applyTrajectorySmoothingAlgorithm(const carma_wm::WorldModelConstPtr& wm, std::vector<PointSpeedPair>& points_and_target_speeds, double start_dist, double remaining_dist, 
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
    void applyOptimizedTargetSpeedProfile(const carma_planning_msgs::msg::Maneuver& maneuver, const double starting_speed, std::vector<PointSpeedPair>& points_and_target_speeds);

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
    std::vector<PointSpeedPair> createGeometryProfile(const std::vector<carma_planning_msgs::msg::Maneuver> &maneuvers, double max_starting_downtrack,const carma_wm::WorldModelConstPtr &wm,
                                                                        carma_planning_msgs::msg::VehicleState &ending_state_before_buffer,const carma_planning_msgs::msg::VehicleState& state,
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
    double findSpeedLimit(const lanelet::ConstLanelet& llt, const carma_wm::WorldModelConstPtr &wm) const;

    FRIEND_TEST(LCITacticalPluginTest, applyTrajectorySmoothingAlgorithm);
    FRIEND_TEST(LCITacticalPluginTest, applyOptimizedTargetSpeedProfile);
    FRIEND_TEST(LCITacticalPluginTest, createGeometryProfile);
    FRIEND_TEST(LCITacticalPluginTest, planTrajectoryCB);
    FRIEND_TEST(LCITacticalPluginTest, setConfig);

  public:

    /*!
    * \brief LightControlledIntersectionTacticalPlugin constructor
    */
    LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm, const Config& config,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);

    /**
     * \brief Function to process the light controlled intersection tactical plugin service call for trajectory planning
     * \param req The service request
     * \param resp The service response
     */ 
    void planTrajectoryCB( 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

    /**
     * \brief Setter function to set a new config for this object
     * \param config The new config to be used by this object
     */ 
    void setConfig(const Config& config);
  };

} // light_controlled_intersection_tactical_plugin