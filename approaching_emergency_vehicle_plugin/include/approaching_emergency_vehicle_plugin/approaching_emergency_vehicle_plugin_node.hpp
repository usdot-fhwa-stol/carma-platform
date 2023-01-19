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
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/msg/upcoming_lane_change_status.hpp>
#include <carma_planning_msgs/msg/route_state.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_v2x_msgs/msg/emergency_vehicle_response.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <lanelet2_extension/io/autoware_osm_parser.h>
#include <carma_wm_ros2/CARMAWorldModel.hpp>

#include <carma_guidance_plugins/strategic_plugin.hpp>
#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_plugin_config.hpp"
#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_transition_table.hpp"
#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_states.hpp"

namespace approaching_emergency_vehicle_plugin
{

  /**
   * \brief Convenience struct for storing relevant data for an Emergency Response Vehicle (ERV)
   */
  struct ErvInformation{
    std::string vehicle_id;            // The vehicle ID asssociated with an ERV 
    double current_speed;              // The current speed (m/s) of an ERV
    double current_latitude;           // The current latitude of an ERV
    double current_longitude;          // The current longitude of an ERV
    lanelet::BasicPoint2d current_position_in_map; // The current position of the ERV in the map frame
    lanelet::ConstLanelet intersecting_lanelet;    // The first intersecting lanelet between ERV's future route and CMV's future shortest path
    double seconds_until_passing;      // The estimated duration (seconds) until the ERV will pass the ego vehicle
                                       //     based on their current positions and current speeds
    bool in_rightmost_lane;            // Flag to indicate whether ERV is located in the rightmost lane
    rclcpp::Time latest_bsm_timestamp; // The timestamp of the latest BSM that triggered an update to this object
    rclcpp::Time latest_update_time;   // The timestamp (from this node's clock) associated with the last update of this object
  };

  /**
   * \brief Class that implements the Approaching Emergency Vehicle Plugin (ERV) strategic plugin. This
   * class is primarily responsible for handling ROS topic subscriptions, publications, and service callbacks
   * for processing incoming BSMs from ERV's, generating manuever plans that account for a nearby ERV, and 
   * publishing warning message(s) to an approaching ERV when the ego vehicle is in its path.
   */
  class ApproachingEmergencyVehiclePlugin : public carma_guidance_plugins::StrategicPlugin
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> incoming_bsm_sub_;

    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;

    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::RouteState> route_state_sub_;

    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::Plugin> plugin_discovery_pub_;

    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::EmergencyVehicleResponse> outgoing_emergency_vehicle_response_pub_;

    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::UpcomingLaneChangeStatus> upcoming_lane_change_status_pub_;

    /**
     * \brief Helper function to obtain an ERV's position in the map frame from its current latitude and longitude. 
     * \param current_latitude The current latitude of the ERV.
     * \param current_longitude The current longitude of the ERV.
     * \return An optional lanelet::BasicPoint2d object, which is empty if the projection of the ERV's current position into
     * the map frame was not successful.
     */
    boost::optional<lanelet::BasicPoint2d> getErvPositionInMap(const double& current_latitude, const double& current_longitude);

    /**
     * \brief Helper function to generate an ERV's route based on its current position and its future route destination points.
     * \param current_latitude The current latitude of the ERV.
     * \param current_longitude The current longitude of the ERV.
     * \param erv_destination_points The ERV's future destination points.
     * \return An optional lanelet::routing::Route object, which is empty if the generation of the ERV's route was not successful.
     */
    lanelet::Optional<lanelet::routing::Route> generateErvRoute(double current_latitude, double current_longitude, 
                                                                             std::vector<carma_v2x_msgs::msg::Position3D> erv_destination_points);

    /**
     * \brief Helper function to obtain the earliest lanelet that exists on both an ERV's future route and the ego vehicle's
     * future shortest path. 
     * \param erv_future_route The ERV's future route
     * \param ego_shortest_path The ego vehicle's future shortest path
     * \return An optional lanelet::ConstLanelet object, which is empty if no intersecting lanelet was found.
     */
    boost::optional<lanelet::ConstLanelet> getRouteIntersectingLanelet(const lanelet::routing::Route& erv_future_route, 
                        const lanelet::routing::LaneletPath& ego_shortest_path);

    /**
     * \brief Helper function to calculate the estimated seconds until an ERV will pass the ego vehicle. This is an estimate
     * since it assumes both the ERV and the ego vehicle will continue travelling at their current speed.
     * \param erv_future_route The ERV's future route
     * \param erv_position_in_map The ERV's current position in the map frame
     * \param erv_current_speed The ERV's current speed (m/s)
     * \param intersecting_lanelet The earliest lanelet that exists on both the ERV's future route and the ego vehicle's
     * future shortest path.
     * \return The estimated seconds until the ERV will pass the ego vehicle. If negative, the ERV is estimated to 
     * never pass the ego vehicle.
     */
    double getSecondsUntilPassing(lanelet::Optional<lanelet::routing::Route>& erv_future_route, const lanelet::BasicPoint2d& erv_position_in_map, 
                                  const double& erv_current_speed, lanelet::ConstLanelet& intersecting_lanelet);

    /**
     * \brief Through internal logic and calls to separate helper functions, this function processes a received BSM
     * to obtain all necessary information for an ERV to be tracked. This "necessary information" includes all fields
     * contained within an ErvInformation struct. 
     * \param msg The BSM message to be processed. It may or may not be from an active ERV.
     * \return An optional ErvInformation object, which is empty if BSM processing could not extract all necessary information
     * required for an ERV to be tracked.
     */
    boost::optional<ErvInformation> getErvInformationFromBsm(carma_v2x_msgs::msg::BSM::UniquePtr msg);

    /**
     * \brief This is a callback function for the erv_timeout_timer_ timer, and  is called to determine whether a 
     * timeout has occurred for the currently tracked ERV by comparing the duration since the latest ERV BSM was received 
     * with config_.timeout_duration. If a timeout has occurred, an ERV_UPDATE_TIMEOUT event will be triggered for this plugin's
     * transition_table_ object, and the has_tracked_erv_ boolean flag will be set to false.
     */
    void checkForErvTimeout();

    // Timer used to check whether a timeout has occurred with the currently-tracked ERV
    rclcpp::TimerBase::SharedPtr erv_timeout_timer_;

    // ApproachingEmergencyVehiclePlugin configuration
    Config config_;

    // State Machine Transition table
    ApproachingEmergencyVehicleTransitionTable transition_table_;

    // ErvInformation object, which includes information on the currently-tracked ERV
    ErvInformation tracked_erv_;

    // Boolean flag to indicate whether an ERV is being actively tracked by this plugin. When true, the
    //         tracked_erv_ object contains the latest received information pertaining to the ERV.
    // NOTE: An ERV is only actively tracked if it is considered to be approaching or passing the ego vehicle.
    bool has_tracked_erv_ = false;

    // Pointer for map projector
    boost::optional<std::string> map_projector_;

    // Latest route state
    carma_planning_msgs::msg::RouteState latest_route_state_;

    // Ego vehicle's current speed
    double current_speed_;

    // Logger name for this plugin
    std::string logger_name = "ApproachingEmergencyVehiclePlugin";

    // Separate world model object for storing the ERV's route and enabling downtrack calculations on its route
    std::shared_ptr<carma_wm::CARMAWorldModel> erv_world_model_;

    // Parameter for comparisons to 0
    double epsilon_ = 0.0001;

    // Unit Test Accessors
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testStateMachineTransitions);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testBSMProcessing);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testRouteConflict);

  public:
    /**
     * \brief ApproachingEmergencyVehiclePlugin constructor 
     */
    explicit ApproachingEmergencyVehiclePlugin(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Incoming BSM subscription callback, which determines whether a BSM should be processed, and extracts ERV information
     * from the BSM through a call to getErvInformationFromBsm(). This function is responsible for updating this plugin's tracked_erv_
     * object and setting has_tracked_erv_ to true when a new ERV becomes tracked.
     * \param msg The incoming BSM message.
     */
    void incomingBsmCallback(carma_v2x_msgs::msg::BSM::UniquePtr msg);

    /**
     * \brief Route State subscription callback, with is used to update this plugin's latest_route_state_ object. 
     * \param msg The latest Route State message.
     */
    void routeStateCallback(carma_planning_msgs::msg::RouteState::UniquePtr msg);

    /**
    * \brief Subscription callback for the georeference.
    * \param msg The latest georeference message.
    */
    void georeferenceCallback(const std_msgs::msg::String::UniquePtr msg);

    /**
      * \brief Subscription callback for the twist subscriber, which will store latest current velocity of the ego vehicle.
      * \param msg Latest twist message
      */
    void twistCallback(geometry_msgs::msg::TwistStamped::UniquePtr msg);

    ////
    // Overrides
    ////
    void plan_maneuvers_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp) override;

    bool get_availability() override;

    std::string get_version_id() override;
    
    /**
     * \brief This method is used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin();

    /**
     * \brief This method is used to create a timer and will be called on the activate transition.
     */ 
    carma_ros2_utils::CallbackReturn on_activate_plugin();

    // wm listener pointer and pointer to the actual wm object
    std::shared_ptr<carma_wm::WMListener> wml_;
    carma_wm::WorldModelConstPtr wm_;

  };

} // approaching_emergency_vehicle_plugin
