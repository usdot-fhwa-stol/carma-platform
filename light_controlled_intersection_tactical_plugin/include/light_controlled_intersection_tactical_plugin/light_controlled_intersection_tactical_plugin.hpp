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
#include <basic_autonomy/basic_autonomy.hpp>
#include <basic_autonomy/helper_functions.hpp>

#include "light_controlled_intersection_tactical_plugin/light_controlled_intersection_tactical_plugin_node.hpp"

/**
 * \brief Macro definition to enable easier access to fields shared across the maneuver types
 * \param mvr The maneuver object to invoke the accessors on
 * \param property The name of the field to access on the specific maneuver types.
 *                 Must be shared by all extant maneuver types
 * \return Expands to an expression (in the form of chained ternary operators) that evaluates to
 *         the desired field
 * NOTE: Here is the definition of the int_valued, float_valued, and string_valued meta data
 *       parameters that is implicitly agreed between LCI Strategic and Tactical plugins
 *       All lane_following_maneuver.properties:
 *        string_valued_meta_data[0]: light_controlled_intersection_strategy name
 *        float_valued_meta_data[0]: Trajectory Segment 1 starting acceleration a1_;
 *        float_valued_meta_data[1]: Trajectory Segment 1 starting velocity v1_;
 *        float_valued_meta_data[2]: Trajectory Segment 1 starting downtrack x1_;
 *        float_valued_meta_data[3]: Trajectory Segment 2 starting acceleration a2_;
 *        float_valued_meta_data[4]: Trajectory Segment 2 starting velocity v2_;
 *        float_valued_meta_data[5]: Trajectory Segment 2 starting downtrack x2_;
 *        float_valued_meta_data[6]: Trajectory Segment 3 starting acceleration a3_;
 *        float_valued_meta_data[7]: Trajectory Segment 3 starting velocity v3_;
 *        float_valued_meta_data[8]: Trajectory Segment 3 starting downtrack x3_;
 *        int_valued_meta_data[0]: Trajectory Smoothing Case Number
 *        int_valued_meta_data[1]: Is Trajectory Smoothing Algorithm Successful? (0: False, 1: True)
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
  using DebugPublisher = std::function<void(const carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds&)>;

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

    std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;

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
    carma_planning_msgs::msg::TrajectoryPlan last_trajectory_time_unbound_;
    carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> yield_client_;
    const std::string LCI_TACTICAL_LOGGER = "light_controlled_intersection_tactical_plugin";
    carma_planning_msgs::msg::VehicleState ending_state_before_buffer_; //state before applying extra points for curvature calculation that are removed later
    rclcpp::Time latest_traj_request_header_stamp_;
    double epsilon_ = 0.001; //Small constant to compare (double) 0.0 with

    // downtrack of host vehicle
    double current_downtrack_ = 0.0;

    double last_successful_ending_downtrack_;         // if algorithm was successful, this is traffic_light_downtrack
    double last_successful_scheduled_entry_time_;     // if algorithm was successful, this is also scheduled entry time (ET in TSMO UC2 Algo)

    std::string plugin_name_;
    DebugPublisher debug_publisher_;  // Publishes the debug message that includes many useful data used to generate the trajectory
    carma_debug_ros2_msgs::msg::TrajectoryCurvatureSpeeds debug_msg_;
    std::vector<double> last_speeds_time_unbound_;

    std::string light_controlled_intersection_strategy_ = "Carma/signalized_intersection"; // Strategy carma-streets is sending. Could be more verbose but needs to be changed on both ends

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
    * \brief Given a Lanelet, find its associated Speed Limit.
    *
    * \param llt Constant Lanelet object.
    * \param wm  World model pointer to query additional information.
    *
    * \throw std::invalid_argument if the speed limit could not be retrieved.
    *
    * \return Speed limit in meters per second.
    */
    double findSpeedLimit(const lanelet::ConstLanelet& llt, const carma_wm::WorldModelConstPtr &wm) const;

    /**
      * \brief Logs debug information about the previously planned trajectory.
      *
      * This helper function outputs detailed internal state and trajectory information
      * that can be used for debugging trajectory planning issues.
      */
    void logDebugInfoAboutPreviousTrajectory();

    /**
      * \brief Determines whether the last trajectory should be reused based on the planning case.
      *        Should use last case if 1) last traj is valid AND last case is the same as new case,
      *        OR 2) last traj is valid AND within certain evaluation zone before the intersection,
      *        the new case is not successful but last case is
      * \param new_case An enumerated type representing the new trajectory planning case.
      * \param is_new_case_successful A boolean flag indicating if the new trajectory
      *                               planning was successful.
      * \param current_time The current time stamp.
      *
      * \return True if the last trajectory should be used, false otherwise.
      */
    bool shouldUseLastTrajectory(TSCase new_case, bool is_new_case_successful,
                                  const rclcpp::Time& current_time);

    /**
      * \brief Smooths the trajectory as part of the trajectory planning process.
      *
      * This function takes a trajectory planning request, applies smoothing algorithms
      * to refine the trajectory, and populates the response with the smoothed trajectory.
      * NOTE: Function is called by planTrajectoryCB and doesn't use yield client
      * \param req  Shared pointer to the trajectory planning request message.
      * \param resp Shared pointer to the trajectory planning response message to be filled.
      */
    void planTrajectorySmoothing(
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

    /**
      * \brief Generates a new trajectory plan based on the provided maneuver plan and request.
      *        NOTE: This function plans for the entire maneuver input and doesn't time bound it.
      *
      * \param maneuver_plan A vector of maneuver messages defining the planned maneuvers.
      *                      The first maneuver is expected to have all the necessary TS parameters.
      * \param req           Shared pointer to the trajectory planning request message.
      * \param final_speeds  A vector that will be populated with the final speeds for each point
      *                      as it is useful for late operations or debugging
      *
      * \return A newly generated trajectory plan message.
      */
    carma_planning_msgs::msg::TrajectoryPlan generateNewTrajectory(
        const std::vector<carma_planning_msgs::msg::Maneuver>& maneuver_plan,
        const carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr& req,
        std::vector<double>& final_speeds);

    /**
      * \brief Checks if the last trajectory plan remains valid based on the current time.
      *
      * This function verifies that the previously generated trajectory still meets timing
      * and duration requirements. It can also enforce a minimum remaining duration if specified.
      *
      * \param current_time               The current time stamp.
      * \param min_remaining_time_seconds Optional parameter specifying the minimum required
      *                                   remaining time (in seconds) for the trajectory to be
      *                                   considered valid.
      *
      * \return True if the last trajectory is valid, false otherwise.
      */
    bool isLastTrajectoryValid(const rclcpp::Time& current_time,
      double min_remaining_time_seconds = 0.0) const;

    FRIEND_TEST(LCITacticalPluginTest, applyTrajectorySmoothingAlgorithm);
    FRIEND_TEST(LCITacticalPluginTest, applyOptimizedTargetSpeedProfile);
    FRIEND_TEST(LCITacticalPluginTest, planTrajectorySmoothing);
    FRIEND_TEST(LCITacticalPluginTest, createGeometryProfile);
    FRIEND_TEST(LCITacticalPluginTest, planTrajectoryCB);
    FRIEND_TEST(LCITacticalPluginTest, setConfig);

  public:

    /*!
    * \brief LightControlledIntersectionTacticalPlugin constructor
    */
    LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm,
      const Config& config,
      const DebugPublisher& debug_publisher,
      const std::string& plugin_name,
      std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh);

    /**
     * \brief Function to process the light controlled intersection tactical plugin service call for trajectory planning
     * \param req The service request
     * \param resp The service response
     */
    void planTrajectoryCB(
      carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
      carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

    /**
     * \brief set the yield service
     *
     * \param yield_srv input yield service
     */
    void set_yield_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> client);

    /**
     * \brief Setter function to set a new config for this object
     * \param config The new config to be used by this object
     */
    void setConfig(const Config& config);
  };

} // light_controlled_intersection_tactical_plugin
