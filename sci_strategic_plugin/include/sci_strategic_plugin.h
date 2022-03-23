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
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/MobilityOperation.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>
#include <bsm_helper/bsm_helper.h>
#include <carma_wm/Geometry.h>
#include <lanelet2_core/Forward.h>
#include <gtest/gtest_prod.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <cav_msgs/BSM.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include "sci_strategic_plugin_config.h"

namespace sci_strategic_plugin
{
  enum TurnDirection {
            Straight,
            Right,
            Left
    };

  /**
  * \brief Anonymous function to extract maneuver end speed which can not be optained with GET_MANEUVER_PROPERY calls due to it missing in stop and wait plugin
  * \param mvr input maneuver
  * \return end speed
 */ 
double getManeuverEndSpeed(const cav_msgs::Maneuver& mvr)
  {
    switch(mvr.type) {
        case cav_msgs::Maneuver::LANE_FOLLOWING:
            return mvr.lane_following_maneuver.end_speed;
        case cav_msgs::Maneuver::LANE_CHANGE:
            return mvr.lane_change_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT:
            return mvr.intersection_transit_straight_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN:
            return mvr.intersection_transit_left_turn_maneuver.end_speed;
        case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN:
            return mvr.intersection_transit_right_turn_maneuver.end_speed;
        case cav_msgs::Maneuver::STOP_AND_WAIT:
            return 0;
        default:
            ROS_ERROR_STREAM("Requested end speed from unsupported maneuver type");
            return 0;
    }
  }


class SCIStrategicPlugin
{
public:
  /**
   * \brief Struct representing a vehicle state for the purposes of planning
   */
  struct VehicleState
  {
    ros::Time stamp;      // Timestamp of this state data
    double downtrack;     // The downtrack of the vehicle along the route at time stamp
    double speed;         // The speed of the vehicle at time stamp
    lanelet::Id lane_id;  // The current lane id of the vehicle at time stamp
  };

  
  /**
   * \brief Constructor
   *
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   */
  SCIStrategicPlugin(carma_wm::WorldModelConstPtr wm, SCIStrategicPluginConfig& config);


  /**
   * \brief Service callback for arbitrator maneuver planning
   * \param req Plan maneuver request
   * \param[out] resp Plan maneuver response with a list of maneuver plan
   * \return If service call successed
   */
  bool planManeuverCb(cav_srvs::PlanManeuversRequest& req, cav_srvs::PlanManeuversResponse& resp);

  /**
   * \brief Returns the current plugin discovery message reflecting system status
   *
   * \return cav_msgs::Plugin The plugin discovery message
   */
  cav_msgs::Plugin getDiscoveryMsg() const;
  
  /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery and mobility operation msgs.
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();


  /**
   * \brief callback function for mobility operation
   * 
   * \param msg input mobility operation msg
   */
  void mobilityOperationCb(const cav_msgs::MobilityOperationConstPtr& msg);

  /**
   * \brief callback function for current pose
   * \param msg input pose stamed msg
   */
  void currentPoseCb(const geometry_msgs::PoseStampedConstPtr& msg);

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
  cav_msgs::Maneuver composeLaneFollowingManeuverMessage(int case_num, double start_dist, double end_dist, double start_speed,
                                                         double target_speed, ros::Time start_time, double time_to_stop,
                                                         std::vector<lanelet::Id> lane_ids);

  cav_msgs::Maneuver composeStopAndWaitManeuverMessage(double current_dist, double end_dist, double start_speed,
                                                      const lanelet::Id& starting_lane_id, const lanelet::Id& ending_lane_id, 
                                                      double stopping_accel, ros::Time start_time, ros::Time end_time) const;

  cav_msgs::Maneuver composeIntersectionTransitMessage(double start_dist, double end_dist, double start_speed, 
                                                      double target_speed, ros::Time start_time, ros::Time end_time, TurnDirection turn_direction,
                                                      const lanelet::Id& starting_lane_id, const lanelet::Id& ending_lane_id) const;

  /**
   * \brief Helper method to extract the initial vehicle state from the planning request method based on if the
   * prior_plan was set or not.
   *
   * \param req The maneuver planning request to extract the vehicle state from
   *
   * \return The extracted VehicleState
   */
  VehicleState extractInitialState(const cav_srvs::PlanManeuversRequest& req) const;

  /**
   * \brief Helper method which calls carma_wm::WorldModel::getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only,
   * bounds_inclusive) and throws and exception if the returned list of lanelets is empty. See the referenced method for additional
   * details on parameters.
   */
  std::vector<lanelet::ConstLanelet> getLaneletsBetweenWithException(double start_downtrack, double end_downtrack,
                                                                     bool shortest_path_only = false,
                                                                     bool bounds_inclusive = true) const;


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
  int determine_speed_profile_case(double stop_dist, double current_speed, double schedule_stop_time, double speed_limit);

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
  double calc_speed_before_decel(double stop_time, double stop_dist, double current_speed) const;

    /**
   * \brief parse strategy parameters
   * 
   * \param strategy_params input string of strategy params from mob op msg
   */
  void parseStrategyParams(const std::string& strategy_params);

  /**
   * \brief calculate the time vehicle will stop with optimal decelarion
   *
   * \param stop_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   *
   * \return the time vehicle will stop with optimal decelarion
   */
  double calcEstimatedStopTime(double stop_dist, double current_speed) const;

  /**
   * \brief Determine the desired speed profile parameters for Case 1
   *
   * \param speed_before_decel vehicle speed before it starts to decelerate
   *
   * \param current_speed current speed of vehicle
   * 
   * \param stop_time time duration to stop in s
   *
   * \param float_metadata_list metadata vector for storing speed profile parameters
   *
   */
  void caseOneSpeedProfile(double speed_before_decel, double current_speed, double stop_time, std::vector<double>* float_metadata_list) const;

  /**
   * \brief Determine the desired speed profile parameters for Case 2
   *
   * \param stop_dist distance to stop line
   *
   * \param speed_before_decel vehicle speed before it starts to decelerate
   *
   * \param current_speed current speed of vehicle
   * 
   * \param stop_time time duration to stop in s
   *
   * \param float_metadata_list metadata vector for storing speed profile parameters
   *
   */
  void caseTwoSpeedProfile(double stop_dist, double speed_before_decel, double current_speed, double stop_time,  double speed_limit, std::vector<double>* float_metadata_list) const;

  /**
   * \brief Determine the desired speed profile parameters for Case 3
   *
   * \param stop_dist distance to stop line
   *
   * \param current_speed current speed of vehicle
   * 
   * \param stop_time time duration to stop in s
   *
   * \return deceleration value for case three
   *
   */
  double caseThreeSpeedProfile(double stop_dist, double current_speed, double stop_time) const;

  /**
   * \brief Generates Mobility Operation messages
   *
   * \return mobility operation msg for status and intent
   */
  cav_msgs::MobilityOperation generateMobilityOperation();

  /**
   * \brief BSM callback function
   */
  void BSMCb(const cav_msgs::BSMConstPtr& msg);

  /**
   * \brief Determine the turn direction at intersection
   *
   * \param lanelets_list List of lanelets crossed around the intersection area
   *
   * \return turn direction in format of straight, left, right
   *
   */
  TurnDirection getTurnDirectionAtIntersection(std::vector<lanelet::ConstLanelet> lanelets_list);
  
  ////////// VARIABLES ///////////

  // CARMA Streets Variakes
  // timestamp for msg received from carma streets
  unsigned long long street_msg_timestamp_ = 0;
  // scheduled stop time
  unsigned long long scheduled_stop_time_ = 0;
  // scheduled enter time
  unsigned long long scheduled_enter_time_ = 0;
  // scheduled depart time
  unsigned long long scheduled_depart_time_ = 0;
  // scheduled latest depart position
  uint32_t scheduled_departure_position_ = std::numeric_limits<uint32_t>::max();
  // flag to show if the vehicle is allowed in intersection
  bool is_allowed_int_ = false;

  TurnDirection intersection_turn_direction_ = TurnDirection::Straight;
  bool vehicle_engaged_ = false;

  // approximate speed limit 
  double speed_limit_ = 100.0;

  // downtrack of host vehicle
  double current_downtrack_ = 0.0;

  bool approaching_stop_controlled_interction_ = false;

  ros::Publisher mobility_operation_pub;
  ros::Publisher plugin_discovery_pub;

  std::string bsm_id_ = "default_bsm_id";
  uint8_t bsm_msg_count_ = 0;
  uint16_t bsm_sec_mark_ = 0;

  private:
  //! World Model pointer
  carma_wm::WorldModelConstPtr wm_;

  //! Config containing configurable algorithm parameters
  SCIStrategicPluginConfig config_;

  //! Plugin discovery message
  cav_msgs::Plugin plugin_discovery_msg_;

  //! Cache variables for storing the current intersection state between state machine transitions
  boost::optional<double> intersection_speed_;
  boost::optional<double> intersection_end_downtrack_;

  // strategy for stop controlled intersection
  std::string stop_controlled_intersection_strategy_ = "Carma/stop_controlled_intersection";
  std::string previous_strategy_params_ = "";  
  

};
}  // namespace sci_strategic_plugin
