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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_v2x_msgs/msg/mobility_operation.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <bsm_helper/bsm_helper.h>
#include <carma_wm_ros2/Geometry.hpp>
#include <lanelet2_core/Forward.h>
#include <gtest/gtest_prod.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <carma_guidance_plugins/strategic_plugin.hpp>
#include "sci_strategic_plugin_config.hpp"

namespace sci_strategic_plugin
{
enum TurnDirection {
          Straight,
          Right,
          Left
};

class SCIStrategicPlugin : public carma_guidance_plugins::StrategicPlugin
{
public:
  /**
   * \brief Struct representing a vehicle state for the purposes of planning
   */
  struct VehicleState
  {
    rclcpp::Time stamp;      // Timestamp of this state data
    double downtrack;     // The downtrack of the vehicle along the route at time stamp
    double speed;         // The speed of the vehicle at time stamp
    lanelet::Id lane_id;  // The current lane id of the vehicle at time stamp
  };

  
  /**
     * \brief Default constructor for RouteFollowingPlugin class
     */
  explicit SCIStrategicPlugin(const rclcpp::NodeOptions &);

  /**
   * \brief Method to publish mobility operation msgs at a fixed rate of 10hz if approaching intersection.
   *
   */ 
  void publishMobilityOperation();

  /**
   * \brief Service callback for arbitrator maneuver planning
   * \param req Plan maneuver request
   * \param resp Plan maneuver response with a list of maneuver plan
   * \return If service call successed
   */
  void plan_maneuvers_callback(
    std::shared_ptr<rmw_request_id_t> srv_header, 
    carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp);

    /**
   * \brief Callback for dynamic parameter updates
   */
  rcl_interfaces::msg::SetParametersResult 
  parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

  /**
   * \brief callback function for mobility operation
   * 
   * \param msg input mobility operation msg
   */
  void mobilityOperationCb(carma_v2x_msgs::msg::MobilityOperation::UniquePtr msg);

  /**
   * \brief callback function for current pose
   * \param msg input pose stamed msg
   */
  void currentPoseCb(geometry_msgs::msg::PoseStamped::UniquePtr msg);

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
  carma_planning_msgs::msg::Maneuver composeLaneFollowingManeuverMessage(int case_num, double start_dist, double end_dist, double start_speed,
                                                         double target_speed, rclcpp::Time start_time, double time_to_stop,
                                                         std::vector<lanelet::Id> lane_ids);

  carma_planning_msgs::msg::Maneuver composeStopAndWaitManeuverMessage(double current_dist, double end_dist, double start_speed,
                                                      const lanelet::Id& starting_lane_id, const lanelet::Id& ending_lane_id, 
                                                      double stopping_accel, rclcpp::Time start_time, rclcpp::Time end_time) const;

  carma_planning_msgs::msg::Maneuver composeIntersectionTransitMessage(double start_dist, double end_dist, double start_speed, 
                                                      double target_speed, rclcpp::Time start_time, rclcpp::Time end_time, TurnDirection turn_direction,
                                                      const lanelet::Id& starting_lane_id, const lanelet::Id& ending_lane_id) const;

  /**
   * \brief Helper method to extract the initial vehicle state from the planning request method based on if the
   * prior_plan was set or not.
   *
   * \param req The maneuver planning request to extract the vehicle state from
   *
   * \return The extracted VehicleState
   */
  VehicleState extractInitialState(const carma_planning_msgs::srv::PlanManeuvers::Request& req) const;

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
  carma_v2x_msgs::msg::MobilityOperation generateMobilityOperation();

  /**
   * \brief BSM callback function
   */
  void BSMCb(carma_v2x_msgs::msg::BSM::UniquePtr msg);

  /**
   * \brief Callback for the Guidance State
   * \param msg Latest GuidanceState message
   */
  void guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg);

  /**
   * \brief Determine the turn direction at intersection
   *
   * \param lanelets_list List of lanelets crossed around the intersection area
   *
   * \return turn direction in format of straight, left, right
   *
   */
  TurnDirection getTurnDirectionAtIntersection(std::vector<lanelet::ConstLanelet> lanelets_list);
  
  ////////// OVERRIDES ///////////
  carma_ros2_utils::CallbackReturn on_configure_plugin();
  carma_ros2_utils::CallbackReturn on_activate_plugin();

  bool get_availability();
  std::string get_version_id();
        

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

  std::string bsm_id_ = "default_bsm_id";
  uint8_t bsm_msg_count_ = 0;
  uint16_t bsm_sec_mark_ = 0;

  private:

  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityOperation> mob_operation_sub_;
  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> current_pose_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> bsm_sub_;
  carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityOperation> mobility_operation_pub_;
  carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;

  bool guidance_engaged_ = false;

  // timer to publish mobility operation message
  rclcpp::TimerBase::SharedPtr mob_op_pub_timer_;

  //! World Model pointer
  carma_wm::WorldModelConstPtr wm_;

  //! Config containing configurable algorithm parameters
  SCIStrategicPluginConfig config_;

  //! Cache variables for storing the current intersection state between state machine transitions
  boost::optional<double> intersection_speed_;
  boost::optional<double> intersection_end_downtrack_;

  // strategy for stop controlled intersection
  std::string stop_controlled_intersection_strategy_ = "Carma/stop_controlled_intersection";
  std::string previous_strategy_params_ = "";  
  
  // Unit test helper functions
  carma_wm::WorldModelConstPtr get_wm() { return wm_; }
  void set_wm(carma_wm::WorldModelConstPtr new_wm) { wm_ = new_wm; }

  // Unit Test Accessors
  FRIEND_TEST(SCIStrategicPluginTest, findSpeedLimit);
  FRIEND_TEST(SCIStrategicPluginTest, moboperationcbtest);
  FRIEND_TEST(SCIStrategicPluginTest, parseStrategyParamstest);
  FRIEND_TEST(SCIStrategicPluginTest, calcEstimatedStopTimetest);
  FRIEND_TEST(SCIStrategicPluginTest, calc_speed_before_deceltest);
  FRIEND_TEST(SCIStrategicPluginTest, determine_speed_profile_casetest);
  FRIEND_TEST(SCIStrategicPluginTest, caseOneSpeedProfiletest);
  FRIEND_TEST(SCIStrategicPluginTest, caseTwoSpeedProfiletest);
  FRIEND_TEST(SCIStrategicPluginTest, caseThreeSpeedProfiletest);
  FRIEND_TEST(SCIStrategicPluginTest, testIntersectionturndirection);
  FRIEND_TEST(SCIStrategicPluginTest, DISABLED_maneuvercbtest);
  FRIEND_TEST(SCIStrategicPluginTest, maneuvercbtest);
};
}  // namespace sci_strategic_plugin
