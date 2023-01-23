/*
 * Copyright (C) 2022-2023 LEIDOS.
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
#include <carma_v2x_msgs/msg/emergency_vehicle_ack.hpp>
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
                    ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_CHANGE ? (mvr).lane_change_maneuver.property :\
                        ((mvr).type == carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING ? (mvr).lane_following_maneuver.property :\
                            ((mvr).type == carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT ? (mvr).stop_and_wait_maneuver.property :\
                                throw std::invalid_argument("GET_MANEUVER_PROPERTY (property) called on maneuver with invalid type id " + std::to_string((mvr).type)))))))))

namespace approaching_emergency_vehicle_plugin
{

  /**
   * \brief Convenience struct for storing relevant data for an Emergency Response Vehicle (ERV).
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
    bool in_rightmost_lane = false;    // Flag to indicate whether ERV is located in the rightmost lane
    rclcpp::Time latest_bsm_timestamp; // The timestamp of the latest BSM that triggered an update to this object
    rclcpp::Time latest_update_time;   // The timestamp (from this node's clock) associated with the last update of this object
    bool has_triggered_warning_messages = false; // Flag to indicate whether ERV has triggered this plugin to broadcast EmergencyVehicleResponse warning messages
  };

  /**
   * \brief Convenience struct for storing the parameters of an upcoming lane change to ensure that the same parameters 
   * are used in separately generated maneuver plans.
   */
  struct UpcomingLaneChangeParameters{
    lanelet::ConstLanelet starting_lanelet; // The starting lanelet of the upcoming lane change
    lanelet::ConstLanelet ending_lanelet;   // The ending lanelet of the upcoming lane change
    double start_dist;                      // The starting downtrack of the upcoming lane change
    double end_dist;                        // The ending downtrack of the upcoming lane change
    double start_speed;                     // The start speed of the upcoming lane change
    double end_speed;                       // The end speed of the upcoming lane change
    std::string maneuver_id;                // The maneuver ID of the upcoming lane change
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

    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::EmergencyVehicleAck> incoming_emergency_vehicle_ack_sub_;

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
     * future shortest path. Accesses the ego vehicle's future shortest path using wm_ object.
     * \param erv_future_route The ERV's future route
     * \return An optional lanelet::ConstLanelet object, which is empty if no intersecting lanelet was found.
     */
    boost::optional<lanelet::ConstLanelet> getRouteIntersectingLanelet(const lanelet::routing::Route& erv_future_route);

    /**
     * \brief Helper function to calculate the estimated seconds until an ERV will pass the ego vehicle. This is an estimate
     * since it assumes both the ERV and the ego vehicle will continue travelling at their current speed.
     * \param erv_future_route The ERV's future route
     * \param erv_position_in_map The ERV's current position in the map frame
     * \param erv_current_speed The ERV's current speed (m/s)
     * \param intersecting_lanelet The earliest lanelet that exists on both the ERV's future route and the ego vehicle's
     * future shortest path.
     * \return An optional double, which includes the estimated seconds until the ERV will pass the ego vehicle. If empty, 
     * the ERV is behind the ego vehicle and travelling slower than the ego vehicle, or the ERV is in front of the ego vehicle
     * while not actively passing the ego vehicle.
     */
    boost::optional<double> getSecondsUntilPassing(lanelet::Optional<lanelet::routing::Route>& erv_future_route, const lanelet::BasicPoint2d& erv_position_in_map, 
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

    /**
     * \brief This is a callback function for the warning_broadcast_timer_, and is called to broadcast an EmergencyVehicleResponse
     * warning message to the currently tracked ERV when the ego vehicle is in the ERV's path, but is unable to change lanes because 
     * the ERV is estimated to pass the ego vehicle in under config_.do_not_move_over_threshold. It increases the num_warnings_broadcasted_ counter,
     * and resets it to 0 when it has reached config_.max_warning_broadcasts.
     */
    void broadcastWarningToErv();

    /**
     * \brief Helper function to convert a map x,y coordinates to a lanelet on the ego vehicle's route.
     * \param x_position A map x-coordinate.
     * \param y_position A map y-coordinate.
     * \return An optional lanelet::ConstLanelet object corresponding to the lanelet on the ego vehicle's route that the map x,y coordinates are positioned within.
     * If empty, the map x,y coordinates are not positioned within a lanelet on the ego vehicle's route.
     */
    boost::optional<lanelet::ConstLanelet> getLaneletOnEgoRouteFromMapPosition(const double& x_position, const double& y_position);

    /**
     * \brief Helper function to extract the speed limit from a provided speed limit.
     * \param lanelet A lanelet in the loaded vector map used by the CARMA System.
     * \return A double containing the speed limit for the provided lanelet.
     */
    double getLaneletSpeedLimit(const lanelet::ConstLanelet& lanelet);

    /**
     * \brief Helper function to obtain the duration of a provided maneuver.
     * \param maneuver The maneuver from which that duration is desired.
     * \param epsilon A double used for comparisons to zero.
     * \return The total duration of the provided maneuver.
     */
    rclcpp::Duration getManeuverDuration(const carma_planning_msgs::msg::Maneuver &maneuver, double epsilon) const;

    /**
     * \brief Function to compose a lane following maneuver message based on the provided maneuver parameters.
     * \param start_dist The starting downtrack for the maneuver.
     * \param end_dist The ending downtrack for the maneuver.
     * \param start_speed The start speed (m/s) for the maneuver.
     * \param target_speed The target speed (m/s) for the maneuver.
     * \param lanelet_id The ID of the lanelet that this maneuver is for.
     * \param start_time The estimated time that the ego vehicle will start this maneuver.
     * \return A lane following maneuver message composed based on the provided maneuver parameters.
     */
    carma_planning_msgs::msg::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, 
                                  double start_speed, double target_speed, int lanelet_id, rclcpp::Time& start_time) const;

    /**
     * \brief Function to compose a lane change maneuver message based on the provided maneuver parameters.
     * \param start_dist The starting downtrack for the maneuver.
     * \param end_dist The ending downtrack for the maneuver.
     * \param start_speed The start speed (m/s) for the maneuver.
     * \param target_speed The target speed (m/s) for the maneuver.
     * \param starting_lane_id The ID of the starting lanelet for this lane change maneuver.
     * \param ending_lane_id The ID of the ending lanelet for this lane change maneuver.
     * \param start_time The estimated time that the ego vehicle will start this maneuver.
     * \return A lane change maneuver message composed based on the provided maneuver parameters.
     */
    carma_planning_msgs::msg::Maneuver composeLaneChangeManeuverMessage(double start_dist, double end_dist, 
                                  double start_speed, double target_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, rclcpp::Time& start_time) const;

    /**
     * \brief Function to compose a stop and wait maneuver message based on the provided maneuver parameters.
     * \param start_dist The starting downtrack for the maneuver.
     * \param end_dist The ending downtrack for the maneuver.
     * \param start_speed The start speed (m/s) for the maneuver.
     * \param starting_lane_id The ID of the starting lanelet for this stop and wait maneuver.
     * \param ending_lane_id The ID of the ending lanelet for this stop and wait maneuver.
     * \param stopping_deceleration The stopping deceleration rate for the planned stop and wait maneuver.
     * \param start_time The estimated time that the ego vehicle will start this maneuver.
     * \return A lane change maneuver message composed based on the provided maneuver parameters.
     */
    carma_planning_msgs::msg::Maneuver composeStopAndWaitManeuverMessage(double start_dist, double end_dist, double start_speed, 
                                  lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double stopping_deceleration, rclcpp::Time& start_time) const;

    /**
     * \brief Function to add a stop and wait maneuver to the end of the maneuver plan contained in the provided 'resp' object. If there is signficant distance
     * between the provided downtrack_progress and stop_maneuver_beginning_downtrack, a lane follow maneuver will be added first to fill this distance. Otherwise,
     * the stop and wait maneuver will at the downtrack provided by downtrack_progress.
     * \param resp The service response for the Plan Maneuvers service, which will be updated by this function.
     * \param downtrack_progress The current downtrack that the first maneuver generated by this function will begin at.
     * \param stop_maneuver_beginning_downtrack The downtrack that the stop and wait maneuver must begin by.
     * \param end_of_route_downtrack The ending downtrack of the ego vehicle's route.
     * \param stopping_entry_speed The estimated speed (m/s) that the vehicle will begin the stop and wait maneuver at. If a lane follow is created before the stop and
     * wait maneuver, the lane follow maneuver will include a start_speed and target_speed equal to this speed.
     * \param current_lanelet_ending_downtrack The ending downtrack of the current lanelet that the first maneuver generated by this function will use.
     * \param stopping_deceleration The stopping deceleration rate for the planned stop and wait maneuver.
     * \param current_lanelet The current lanelet that the first maneuver generated by this function will use.
     * \param time_progress The time that the first maneuver generated by this function will begin at.
     * \return None; but function updates the provided 'resp', which is the service response for the Plan Maneuvers service. The maneuver plan in this response will
     * be updated to have a stop and wait maneuver added at its end.
     */
    void addStopAndWaitToEndOfPlan(carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
                          double downtrack_progress, double stop_maneuver_beginning_downtrack, double end_of_route_downtrack, 
                          double stopping_entry_speed, double stopping_deceleration, double current_lanelet_ending_downtrack,
                          lanelet::ConstLanelet current_lanelet, rclcpp::Time time_progress);

    /**
     * \brief Function to generate a maneuver plan when the ego vehicle must remain in its lane due to being in either the WAITING_FOR_APPROACHING_ERV
     * or SLOWING_DOWN_FOR_ERV states. If in the SLOWING_DOWN_FOR_ERV, the target speed of lane follow maneuvers in the generated plan will have a 
     * reduced value.
     * \param resp The service response for the Plan Maneuvers service, which will be updated by this function.
     * \param current_lanelet The current lanelet that the first maneuver generated by this function will use.
     * \param downtrack_progress The current downtrack that the first maneuver generated by this function will begin at.
     * \param current_lanelet_ending_downtrack The ending downtrack of the current lanelet that the first maneuver generated by this function will use.
     * \param speed_progress The current speed that the first maneuver generated by this function will begin at.
     * \param target_speed The target speed of the initial lane follow maneuver generated by this function.
     * \param time_progress The time that the first maneuver generated by this function will begin at.
     * \param is_slowing_down_for_erv Flag to indicate whether the target speed of the lane follow maneuver should be reduced due to an actively passing ERV.
     * \return None; but function updates the provided 'resp', which is the service response for the Plan Maneuvers service. The maneuver plan in this response will
     * be updated to contain all lane following maneuvers. A stop and wait maneuver will be included at the end if the maneuver plan reaches the end of the route.
     */
    void generateRemainInLaneManeuverPlan(carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp,
                                lanelet::ConstLanelet current_lanelet, double downtrack_progress, double current_lanelet_ending_downtrack,
                                double speed_progress, double target_speed, rclcpp::Time time_progress, bool is_slowing_down_for_erv);

    /**
     * \brief Function to generate a maneuver plan when the ego vehicle must change lanes due to being in the MOVING_OVER_FOR_APPROACHING_ERV state.
     * If both the ego vehicle and the ERV are in the rightmost lane, a left lane change will be contained in the plan; otherwise a right lane change will be planned.
     * \param resp The service response for the Plan Maneuvers service, which will be updated by this function.
     * \param current_lanelet The current lanelet that the first maneuver generated by this function will use.
     * \param downtrack_progress The current downtrack that the first maneuver generated by this function will begin at.
     * \param current_lanelet_ending_downtrack The ending downtrack of the current lanelet that the first maneuver generated by this function will use.
     * \param speed_progress The current speed that the first maneuver generated by this function will begin at.
     * \param target_speed The target speed of the initial lane follow maneuver generated by this function.
     * \param time_progress The time that the first maneuver generated by this function will begin at.
     * \return None; but function updates the provided 'resp', which is the service response for the Plan Maneuvers service. The maneuver plan in this response will
     * be updated to contain lane following maneuvers along with one lane change maneuver. A stop and wait maneuver will be included at the end if the maneuver plan 
     * reaches the end of the route.
     */
    void generateMoveOverManeuverPlan(carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp,
                                  lanelet::ConstLanelet current_lanelet, double downtrack_progress, double current_lanelet_ending_downtrack,
                                  double speed_progress, double target_speed, rclcpp::Time time_progress);

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

    // Timer used to check whether a timeout has occurred with the currently-tracked ERV
    rclcpp::TimerBase::SharedPtr erv_timeout_timer_;

    // Boolean flag to indicate that EmergencyVehicleResponse warning message(s) should be broadcasted to the currently-tracked ERV
    // Note: These are broadcasted when the ego vehicle is in the ERV's path but is unable to change lanes
    bool should_broadcast_warnings_ = false;

    // Counter for the number of EmergencyVehicleResponse warning messages broadcasted to the currently-tracked ERV
    int num_warnings_broadcasted_ = 0;

    // Timer used to trigger the broadcast of an EmergencyVehicleResponse warning message to the currently-tracked ERV
    rclcpp::TimerBase::SharedPtr warning_broadcast_timer_;

    // Object to store the parameters of an upcoming lane change maneuver so that the same parameters are used when the
    //        maneuver plan is regenerated
    UpcomingLaneChangeParameters upcoming_lc_params_;

    // Boolean flag to indicate that this plugin has planned an upcoming lane change, and those same lane change maneuver
    //        parameters should be used for the next generated maneuver plan as well
    bool has_planned_upcoming_lc_;

    // Pointer for map projector
    boost::optional<std::string> map_projector_;

    // Latest route state
    carma_planning_msgs::msg::RouteState latest_route_state_;

    // Ego vehicle's current speed
    double current_speed_;

    // Logger name for this plugin
    std::string logger_name = "ApproachingEmergencyVehiclePlugin";

    // The name of this strategic plugin
    std::string strategic_plugin_name_ = "approaching_emergency_vehicle_plugin";

    // Separate world model object for storing the ERV's route and enabling downtrack calculations on its route
    std::shared_ptr<carma_wm::CARMAWorldModel> erv_world_model_;

    // Parameter for comparisons to 0
    double epsilon_ = 0.0001;

    // Unit Test Accessors
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testStateMachineTransitions);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testBSMProcessing);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testRouteConflict);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenWaitingForApproachingErv);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenSlowingDownForErv);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testManeuverPlanWhenMovingOverForErv);
    FRIEND_TEST(Testapproaching_emergency_vehicle_plugin, testWarningBroadcast);

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
    * \brief Subscription callback for incoming EmergencyVehicleAck messages. If the message is from the currently tracked
    * ERV and is intended for the ego vehicle, then the plugin will reset num_warnings_broadcasted_ to zero and will reset
    * the should_broadcast_warnings_ flag to false.
    */
    void incomingEmergencyVehicleAckCallback(const carma_v2x_msgs::msg::EmergencyVehicleAck::UniquePtr msg);

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
