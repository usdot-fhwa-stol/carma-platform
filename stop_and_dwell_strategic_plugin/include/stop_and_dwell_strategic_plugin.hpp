#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
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
#include <carma_wm/WMListener.hpp>
#include <carma_wm/WorldModel.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_wm/Geometry.hpp>
#include <lanelet2_core/Forward.h>
#include <gtest/gtest_prod.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <carma_guidance_plugins/strategic_plugin.hpp>
#include "stop_and_dwell_strategic_plugin_config.hpp"

namespace stop_and_dwell_strategic_plugin
{

class StopAndDwellStrategicPlugin : public carma_guidance_plugins::StrategicPlugin
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
  explicit StopAndDwellStrategicPlugin(const rclcpp::NodeOptions &);

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
   * \brief Callback for the Guidance State
   * \param msg Latest GuidanceState message
   */
  void guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg);
 
  ////////// OVERRIDES ///////////
  carma_ros2_utils::CallbackReturn on_configure_plugin();
  carma_ros2_utils::CallbackReturn on_activate_plugin();

  bool get_availability();
  std::string get_version_id();
        

  ////////// VARIABLES ///////////

  bool vehicle_engaged_ = false;

  // approximate speed limit 
  double speed_limit_ = 100.0;

  // downtrack of host vehicle
  double current_downtrack_ = 0.0;

  bool approaching_stop_controlled_interction_ = false;

  private:

  carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> current_pose_sub_;
  carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;

  bool guidance_engaged_ = false;

  //! World Model pointer
  carma_wm::WorldModelConstPtr wm_;

  //! Config containing configurable algorithm parameters
  StopAndDwellStrategicPluginConfig config_;

  // Unit test helper functions
  carma_wm::WorldModelConstPtr get_wm() { return wm_; }
  void set_wm(carma_wm::WorldModelConstPtr new_wm) { wm_ = new_wm; }

  // Unit Test Accessors
  FRIEND_TEST(StopAndDwellStrategicPluginTest, findSpeedLimit);
  FRIEND_TEST(StopAndDwellStrategicPluginTest, maneuvercbtest);
};
}  // namespace stop_and_dwell_strategic_plugin
