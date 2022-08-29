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
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_guidance_plugins/tactical_plugin.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/geometry.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>
#include "stop_controlled_intersection_config.hpp"
#include <unordered_set>
#include <autoware_msgs/msg/lane.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <basic_autonomy_ros2/basic_autonomy.hpp>
#include <basic_autonomy_ros2/helper_functions.hpp>
#include <basic_autonomy_ros2/log/log.hpp>
#include <gtest/gtest_prod.h>

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

namespace stop_controlled_intersection_tactical_plugin
{
using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

/**
 * \brief Class containing primary business logic for the Stop Controlled Intersection Tactical Plugin
 * 
 */

class StopControlledIntersectionTacticalPlugin : public carma_guidance_plugins::TacticalPlugin
{
public:

  // brief Constructor
  explicit StopControlledIntersectionTacticalPlugin(const rclcpp::NodeOptions &options);

 /**
   * \brief Converts a set of requested stop controlled intersection maneuvers to point speed limit pairs. 
   * 
   * \param maneuvers The list of maneuvers to convert
   * 
   * \param wm Pointer to intialized world model for semantic map access
   * * \param state The current state of the vehicle
   * 
   * \return List of centerline points paired with target speeds
   */
  std::vector<PointSpeedPair> maneuvers_to_points(const std::vector<carma_planning_msgs::msg::Maneuver>& maneuvers,
                                                             const carma_wm::WorldModelConstPtr& wm,
                                                             const carma_planning_msgs::msg::VehicleState& state);
  
   /**
   * \brief Creates a speed profile according to case one of the stop controlled intersection, where the vehicle accelerates and then decelerates to a stop. 
   * \param wm Pointer to intialized world model for semantic map access
   * 
   * \param maneuvers The maneuver to being planned for. Maneuver meta-dara parameters are used to create the trajectory profile.
   * 
   * \param route_geometry_points The geometry points along the route which are associated with a speed in this method.
   * 
   * \param starting_speed The current speed of the vehicle at the time of the trajectory planning request
   * 
   * \return List of centerline points paired with target speeds
   */
  std::vector<PointSpeedPair> create_case_one_speed_profile(const carma_wm::WorldModelConstPtr& wm, const carma_planning_msgs::msg::Maneuver& maneuver,
                                                            std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed, const carma_planning_msgs::msg::VehicleState& states);
  
     /**
   * \brief Creates a speed profile according to case two of the stop controlled intersection, 
   * where the vehicle first accelerates then cruises and finally decelerates to a stop. 
   * \param wm Pointer to intialized world model for semantic map access
   * 
   * \param maneuvers The maneuver to being planned for. Maneuver meta-dara parameters are used to create the trajectory profile.
   * 
   * \param route_geometry_points The geometry points along the route which are associated with a speed in this method.
   * 
   * \param starting_speed The current speed of the vehicle at the time of the trajectory planning request
   * 
   * \return List of centerline points paired with speed limits
   */
  std::vector<PointSpeedPair> create_case_two_speed_profile(const carma_wm::WorldModelConstPtr& wm, const carma_planning_msgs::msg::Maneuver& maneuver,
                                                          std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed);

       /**
   * \brief Creates a speed profile according to case three of the stop controlled intersection, 
   * where the vehicle continuously decelerates to a stop. 
   * \param wm Pointer to intialized world model for semantic map access
   * 
   * \param maneuvers The maneuver to being planned for. Maneuver meta-dara parameters are used to create the trajectory profile.
   * 
   * \param route_geometry_points The geometry points along the route which are associated with a speed in this method.
   * 
   * \param starting_speed The current speed of the vehicle at the time of the trajectory planning request
   * 
   * \return List of centerline points paired with speed limits
   */
  std::vector<PointSpeedPair> create_case_three_speed_profile(const carma_wm::WorldModelConstPtr& wm, const carma_planning_msgs::msg::Maneuver& maneuver,
                                                          std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed);

   /**
   * \brief Method converts a list of lanelet centerline points and current vehicle state into a usable list of trajectory points for trajectory planning
   * 
   * \param points The set of points that define the current lane the vehicle is in and are defined based on the request planning maneuvers. 
   *               These points must be in the same lane as the vehicle and must extend in front of it though it is fine if they also extend behind it. 
   * \param state The current state of the vehicle
   * \param state_time The abosolute time which the provided vehicle state corresponds to
   * 
   * \return A list of trajectory points to send to the carma planning stack
   */ 
  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const carma_planning_msgs::msg::VehicleState& state, const rclcpp::Time& state_time); 

  // overrides
  carma_ros2_utils::CallbackReturn on_configure_plugin() override;
  bool get_availability();
  std::string get_version_id();
  rcl_interfaces::msg::SetParametersResult 
  parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);
  void plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

  private:

  carma_wm::WorldModelConstPtr wm_;
  StopControlledIntersectionTacticalPluginConfig config_;

  std::string stop_controlled_intersection_strategy_ = "Carma/stop_controlled_intersection";

  double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with

  // Unit Test Accessors
  FRIEND_TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_one);
  FRIEND_TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_two);
  FRIEND_TEST(StopControlledIntersectionTacticalPlugin, TestSCIPlanning_case_three);
};


} // namespace stop_controlled_intersection_tactical_plugin