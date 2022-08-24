/*
 * Copyright (C) 2019-2022 LEIDOS.
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
#include <carma_ros2_utils/containers/containers.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/mobility_request.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/lane_change_status.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <basic_autonomy_ros2/basic_autonomy.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/msg/string.hpp>

#include <carma_guidance_plugins/tactical_plugin.hpp>
#include "cooperative_lanechange/cooperative_lanechange_config.hpp"

namespace cooperative_lanechange
{
  using PointSpeedPair = basic_autonomy::waypoint_generation::PointSpeedPair;

  /**
   * \brief Convenience struct for storing the original start_dist and starting_lane_id associated 
   * with a received lane change maneuver.
   */
  struct LaneChangeManeuverOriginalValues
  {
    std::string maneuver_id; // maneuver_id that this object corresponds to
    std::string original_starting_lane_id; // Original starting_lane_id associated with this lane change maneuver
    double original_start_dist; // Original start_dist associated with this lane change maneuver
    double original_longitudinal_vel_ms; // The vehicle velocity (in m/s) when the vehicle first began this lane change
    bool has_started = false; // Flag to indicate whether the vehicle's downtrack is beyond the original_start_dist of this lane change maneuver
  };

  /**
   * \class CooperativeLaneChangePlugin
   * \brief The class responsible for generating cooperative lanechange trajectories from received lane change maneuvers
   */
  class CooperativeLaneChangePlugin : public carma_guidance_plugins::TacticalPlugin
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> twist_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityResponse> incoming_mobility_response_sub_;
    carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> bsm_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::MobilityRequest> outgoing_mobility_request_pub_;
    carma_ros2_utils::PubPtr<carma_planning_msgs::msg::LaneChangeStatus> lanechange_status_pub_;

    // CooperativeLaneChangePlugin configuration
    Config config_;

    // World Model object
    carma_wm::WorldModelConstPtr wm_;

    // Map projection string, which defines the lat/lon -> map conversion
    std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

    // Trajectory frequency
    double traj_freq_ = 10;
    
    // Default recipient ID to be used for cooperative lane change Mobility Requests
    std::string DEFAULT_STRING_= "";

    // Time at which the cooperative lane change Mobility Request is first sent
    rclcpp::Time request_sent_time_;

    // Boolean that records whether request has already been sent
    bool request_sent_ = false;

    // Fraction of the received maneuver that has already been completed by the host vehicle
    double maneuver_fraction_completed_ = 0;

    // Boolean to check if CLC plugin's plan trajectory service has been called
    bool clc_called_ = false;

    // The plan ID associated with the cooperative lane change Mobility Request; initialized with a default value
    std::string clc_request_id_ = "default_request_id";

    // The latest BSM Core Data broadcasted by the host vehicle
    carma_v2x_msgs::msg::BSMCoreData bsm_core_;

    // Maps maneuver IDs to their corresponding LaneChangeManeuverOriginalValues object
    std::unordered_map<std::string, LaneChangeManeuverOriginalValues> original_lc_maneuver_values_;

    /**
     * \brief Callback for the pose subscriber, which will store latest pose locally
     * \param msg Latest pose message
     */
    void pose_cb(const geometry_msgs::msg::PoseStamped::UniquePtr msg);
            
    /**
     * \brief Callback for the twist subscriber, which will store latest twist locally
     * \param msg Latest twist message
     */
    void twist_cb(const geometry_msgs::msg::TwistStamped::UniquePtr msg);

    /**
     * \brief Callback for the BSM subscriber, which will store the latest BSM Core Data broadcasted by the host vehicle
     * \param msg Latest BSM message
     */
    void bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg);

    /**
     * \brief Method for extracting the BSM ID from a BSM Core Data object and converting it to a string
     * \param bsm_core The BSM Core Data object from which the BSM ID is converted to a string
     * \return The BSM ID (as a string) from the BSM Core Data object
     */
    std::string bsmIDtoString(carma_v2x_msgs::msg::BSMCoreData bsm_core);

  public:
    // Internal Variables used in unit testsis_lanechange_accepted_
    // Current vehicle forward speed
    double current_speed_;

    // Current vehicle pose in map
    geometry_msgs::msg::PoseStamped pose_msg_;

    // Boolean flag which is updated if lane change request is accepted
    bool is_lanechange_accepted_ = false;

    carma_planning_msgs::msg::VehicleState ending_state_before_buffer_;

    /**
     * \brief CooperativeLaneChangePlugin constructor 
     */
    explicit CooperativeLaneChangePlugin(const rclcpp::NodeOptions &);

    /**
     * \brief Callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    /**
     * \brief Creates a vector of Trajectory Points from maneuver information in trajectory request
     * 
     * \param req The service request
     * 
     * \return vector of unobstructed lane change trajectory points
     */ 
    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> plan_lanechange(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req);

    /**
     * \brief Calculates distance between subject vehicle and vehicle 2
     * 
     * \param veh2_lanelet_id Current lanelet id of vehicle 2
     * \param veh2_downtrack Downtrack of vehicle 2 in its current lanelet
     * \param ego_state vehicle state of the ego vehicle
     * 
     * \return the distance between subject vehicle and vehicle 2
     */ 
    double find_current_gap(long veh2_lanelet_id, double veh2_downtrack, carma_planning_msgs::msg::VehicleState& ego_state) const;

    /**
     * \brief Callback to subscribed mobility response topic
     * \param msg Latest mobility response message
     */
    void mobilityresponse_cb(const carma_v2x_msgs::msg::MobilityResponse::UniquePtr msg);

    /**
     * \brief Creates a mobility request message from planned trajectory and requested maneuver info
     * \param trajectory_plan A vector of lane change trajectory points
     * \param The mobility request message created from trajectory points, for publishing
     */
    carma_v2x_msgs::msg::MobilityRequest create_mobility_request(std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_plan, carma_planning_msgs::msg::Maneuver& maneuver);

    /**
     * \brief Converts Trajectory Plan to (Mobility) Trajectory
     * \param traj_points vector of Trajectory Plan points to be converted to Trajectory type message
     * \return The Trajectory type message in world frame
     */        
    carma_v2x_msgs::msg::Trajectory trajectory_plan_to_trajectory(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points) const;

    /**
     * \brief Converts Trajectory Point to ECEF frame using map projection
     * \param traj_points A Trajectory Plan point to be converted to Trajectory type message
     * \throw std::invalid_argument If the map_projector_ member variable has not been set
     * \return The trajectory point message transformed to ecef frame
     */
    carma_v2x_msgs::msg::LocationECEF trajectory_point_to_ecef(const carma_planning_msgs::msg::TrajectoryPlanPoint& traj_point) const;

    /**
     * \brief Adds the generated trajectory plan to the service response
     * \param req The plan trajectory service request
     * \param resp The plan trajectory service response
     * \param planned_trajectory_points The generated trajectory plan points, which are added to the service response
     */
    void add_trajectory_to_response(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
                                  carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp, 
                                  const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& planned_trajectory_points);

    /**
     * \brief Callback for map projection string to define lat/lon -> map conversion
     * \brief msg The proj string defining the projection.
     */ 
    void georeference_cb(const std_msgs::msg::String::UniquePtr msg);

    ////
    // Overrides
    ////
    void plan_trajectory_callback(
      std::shared_ptr<rmw_request_id_t>, 
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) override;

    bool get_availability() override;

    std::string get_version_id() override;
    
    /**
     * \brief This method should be used to load parameters and will be called on the configure state transition.
     */ 
    carma_ros2_utils::CallbackReturn on_configure_plugin();

    // Unit Tests
    FRIEND_TEST(CooperativeLaneChangePlugin, TestLaneChangefunctions);
    FRIEND_TEST(CooperativeLaneChangePlugin, Testcurrentgapcb);
    FRIEND_TEST(CooperativeLaneChangePlugin,TestNoPath_roadwayobject);
  };

} // cooperative_lanechange
