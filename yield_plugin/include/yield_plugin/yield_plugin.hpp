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
#include <carma_v2x_msgs/msg/mobility_request.hpp>
#include <carma_v2x_msgs/msg/mobility_response.hpp>
#include <carma_v2x_msgs/msg/bsm.hpp>
#include <carma_planning_msgs/msg/lane_change_status.hpp>
#include <boost/shared_ptr.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <boost/geometry.hpp>
#include <carma_wm/WMListener.hpp>
#include <functional>
#include "yield_config.hpp"
#include <unordered_set>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_utils/quintic_coefficient_calculator.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <carma_v2x_msgs/msg/location_ecef.hpp>
#include <carma_v2x_msgs/msg/trajectory.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <carma_wm/Geometry.hpp>
#include <carma_wm/WorldModel.hpp>
#include <carma_wm/collision_detection.hpp>
#include <carma_wm/TrafficControl.hpp>


namespace yield_plugin
{
using MobilityResponseCB = std::function<void(const carma_v2x_msgs::msg::MobilityResponse&)>;
using LaneChangeStatusCB = std::function<void(const carma_planning_msgs::msg::LaneChangeStatus&)>;

/**
 * \brief Convenience class for pairing 2d points with speeds
 */
struct PointSpeedPair
{
  lanelet::BasicPoint2d point;
  double speed = 0;
};

/**
 * \brief Convenience class for saving collision results
 */
struct GetCollisionResult
{
  rclcpp::Time collision_time;
  lanelet::BasicPoint2d point1;
  lanelet::BasicPoint2d point2;
};

/**
 * \brief Class containing primary business logic for the In-Lane Cruising Plugin
 *
 */
class YieldPlugin
{
public:
  /**
   * \brief Constructor
   *
   * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
   * \param config The configuration to be used for this object
   * \param mobility_response_publisher Callback which will publish the mobility response
   * \param lc_status_publisher Callback which will publish the cooperative lane change status
   */
  YieldPlugin(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                       MobilityResponseCB mobility_response_publisher,
                       LaneChangeStatusCB lc_status_publisher);

  /**
   * \brief Service callback for trajectory planning
   * \param srv_header header
   * \param req The service request
   * \param resp The service response
   *
   */
  void plan_trajectory_callback(
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp);

  /**
   * \brief trajectory is modified to safely avoid obstacles on the road
   * \param original_tp original trajectory plan without object avoidance
   * \param current_speed_ current speed of the vehicle
   * \param
   * \return modified trajectory plan
   */
  carma_planning_msgs::msg::TrajectoryPlan update_traj_for_object(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects, double initial_velocity);

  /**
   * \brief calculate quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of polynomial for given input
   */
  double polynomial_calc(std::vector<double> coeff, double x) const;

  /**
   * \brief calculate derivative of quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of derivative polynomial for given input
   */
  double polynomial_calc_d(std::vector<double> coeff, double x) const;

  /**
   * \brief calculates the maximum speed in a set of tajectory points
   * \param trajectory_points trajectory points
   # \param timestamp_in_sec_to_search_until before which to look for max trajectory speed
   * \return maximum speed
   */
  double max_trajectory_speed(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points, double timestamp_in_sec_to_search_until) const;

  /**
   * \brief calculates distance between trajectory points in a plan
   * \param trajectory_plan input trajectory plan
   * \return vector of relative distances between trajectory points
   */
  std::vector<double> get_relative_downtracks(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan) const;

  /**
   * \brief callback for mobility request
   * \param msg mobility request message
   */
  void mobilityrequest_cb(const carma_v2x_msgs::msg::MobilityRequest::UniquePtr msg);

  /**
   * \brief callback for bsm message
   * \param msg mobility bsm message
   */
  void bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg);

  /**
   * \brief convert a carma trajectory from ecef frame to map frame
   * ecef trajectory consists of the point and a set of offsets with reference to the point
   * \param ecef_trajectory carma trajectory (ecef frame)
   * \return vector of 2d points in map frame
   */
  std::vector<lanelet::BasicPoint2d> convert_eceftrajectory_to_mappoints(const carma_v2x_msgs::msg::Trajectory& ecef_trajectory) const;

  /**
   * \brief convert a point in ecef frame (in cm) into map frame (in meters)
   * \param ecef_point carma point ecef frame in cm
   * \param map_in_earth translate frame
   * \return 2d point in map frame
   */
  lanelet::BasicPoint2d ecef_to_map_point(const carma_v2x_msgs::msg::LocationECEF& ecef_point) const;

  /**
   * \brief compose a mobility response message
   * \param resp_recipient_id vehicle id of the recipient of the message
   * \param req_plan_id plan id from the requested message
   * \param response accept/reject to the response based on conditions
   * \return filled mobility response
   */
  carma_v2x_msgs::msg::MobilityResponse compose_mobility_response(const std::string& resp_recipient_id, const std::string& req_plan_id, bool response) const;

  /**
   * \brief generate a Jerk Minimizing Trajectory(JMT) with the provided start and end conditions
   * \param original_tp original trajectory plan
   * \param intial_pos start position
   * \param goal_pos final position
   * \param initial_velocity start velocity
   * \param goal_velocity end velocity
   * \param planning_time time duration of the planning
   * \param original original_max_speed from original_tp up until the goal_pos portion
   * NOTE: the function would generate trajectory duration arbitrarily high if stopping motion is needed. This is to keep the original trajectory's
   * shape in terms of location so that the vehicle steers toward the direction of travel even when stopping.
   * \return updated JMT trajectory
   */
  carma_planning_msgs::msg::TrajectoryPlan generate_JMT_trajectory(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double initial_pos, double goal_pos,
    double initial_velocity, double goal_velocity, double planning_time, double original_max_speed);

  /**
   * \brief update trajectory for yielding to an incoming cooperative behavior
   * \param original_tp original trajectory plan
   * \param current_speed current speed of the vehicle
   * \return updated trajectory for cooperative behavior
   */
  carma_planning_msgs::msg::TrajectoryPlan update_traj_for_cooperative_behavior(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double current_speed);

  /**
   * \brief detect intersection point(s) of two trajectories
   * \param trajectory1 vector of 2d trajectory points
   * \param trajectory2 vector of 2d trajectory points
   * \return vector of pairs of 2d intersection points and index of the point in trajectory array
   */
  std::vector<std::pair<int, lanelet::BasicPoint2d>> detect_trajectories_intersection(std::vector<lanelet::BasicPoint2d> self_trajectory, std::vector<lanelet::BasicPoint2d> incoming_trajectory) const;


  /**
   * \brief set values for member variables related to cooperative behavior
   * \param req_trajectory requested trajectory
   * \param req_speed speed of requested cooperative behavior
   * \param req_planning_time planning time for the requested cooperative behavior
   * \param req_timestamp the mobility request time stamp
   */
  void set_incoming_request_info(std::vector <lanelet::BasicPoint2d> req_trajectory, double req_speed, double req_planning_time, double req_timestamp);

  /**
   * \brief Looks up the transform between map and earth frames, and sets the member variable
   */
  void lookup_ecef_to_map_transform();

  /**
   * \brief checks trajectory for minimum gap associated with it from the road
   * \param original_tp original trajectory plan
   * \return minumum required min_gap from the road, if none exists, return default minimum_safety_gap_in_meters
   */
  double check_traj_for_digital_min_gap(const carma_planning_msgs::msg::TrajectoryPlan& original_tp) const;

  /**
   * \brief Setter for map projection string to define lat/lon -> map conversion
   * \param georeference The proj string defining the projection.
   */
  void set_georeference_string(const std::string& georeference);

  /**
   * \brief Setter for external objects with predictions in the environment
   * \param object_list The object list.
   */
  void set_external_objects(const std::vector<carma_perception_msgs::msg::ExternalObject>& object_list);

  /**
   * \brief Return naive collision time and locations based on collision radius given two trajectories with one being obstacle's predicted steps
   * \param trajectory1 trajectory of the ego vehicle
   * \param trajectory2 trajectory of predicted steps
   * \param collision_radius a distance to check between two trajectory points at a same timestamp that is considered a collision
   * \param trajectory1_max_speed max speed of the trajectory1 to efficiently traverse through possible collision combination of the two trajectories
   * NOTE: Currently Traj2 is assumed to be a simple cv model to save computational performance
   * NOTE: Collisions are based on only collision radius at the same predicted time even if ego vehicle maybe past the obstacle. To filter these cases, see `is_object_behind_vehicle()`
   * \return data of time of collision if detected, otherwise, std::nullopt
   */
  std::optional<GetCollisionResult> get_collision(const carma_planning_msgs::msg::TrajectoryPlan& trajectory1, const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2, double collision_radius, double trajectory1_max_speed);

  /**
   * \brief Return collision time given two trajectories with one being external object with predicted steps
   * \param trajectory1 trajectory of the ego vehicle
   * \param trajectory2 trajectory of the obstacle
   * \param original_tp_max_speed max speed of the original_tp to efficiently traverse through possible collision combination of the two trajectories
   * NOTE: Currently curr_obstacle is assumed to be using a simple cv model to save computational performance
   * \return time_of_collision if collision detected, otherwise, std::nullopt
   */
  std::optional<rclcpp::Time> get_collision_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, const carma_perception_msgs::msg::ExternalObject& curr_obstacle, double original_tp_max_speed);

  /**
   * \brief Check if object location is behind the vehicle using estimates of the vehicle's length and route downtracks
   * \param object_id object id to use for the consecutive_clearance_count_for_obstacles_
   * \param collision_time predicted time of collision
   * \param vehicle_downtrack at the time of collision
   * \param object_downtrack at the time of collision
   * NOTE: Uses internal counter low pass filter to confirm the object is behind only if it counted continuously above
           config_.consecutive_clearance_count_for_passed_obstacles_threshold
   * \return return true if object is behind the vehicle
   */
  bool is_object_behind_vehicle(uint32_t object_id, const rclcpp::Time& collision_time, double vehicle_point, double object_downtrack);

  /**
   * \brief Return the earliest collision object and time of collision pair from the given trajectory and list of external objects with predicted states.
            Function first filters obstacles based on whether if their any of predicted state will be on the route. Only then, the logic compares trajectory and predicted states.
   * \param original_tp trajectory of the ego vehicle
   * \param external_objects list of external objects with predicted states
   * \return earliest collision object and its collision time if collision detected. std::nullopt if no collision is detected or if route is not available.
   */
  std::optional<std::pair<carma_perception_msgs::msg::ExternalObject, double>> get_earliest_collision_object_and_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects);

  /**
   * \brief Given the list of objects with predicted states, get all collision times concurrently using multi-threading
   * \param original_tp trajectory of the ego vehicle
   * \param external_objects list of external objects with predicted states
   * \param original_tp_max_speed max speed of the original_tp to efficiently traverse through possible collision combination of the two trajectories
   * \return mapping of objects' ids and their corresponding collision times (non-colliding objects are omitted)
   */
  std::unordered_map<uint32_t, rclcpp::Time> get_collision_times_concurrently(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects, double original_tp_max_speed);

  /**
   * \brief Given the object velocity in map frame with x,y components, this function returns the projected velocity along the trajectory at given time.
   * \param object_velocity_in_map_frame trajectory of the ego vehicle
   * \param original_tp trajectory of the ego vehicle
   * \param timestamp_in_sec_to_predict timestamp in seconds along the trajectory to return the projected velocity
   * NOTE: returns the last point's speed if the specified time is past the trajectory's planning time
   * \return get_predicted_velocity_at_time
   */
  double get_predicted_velocity_at_time(const geometry_msgs::msg::Twist& object_velocity_in_map_frame, const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double timestamp_in_sec_to_predict);


private:

  carma_wm::WorldModelConstPtr wm_;
  YieldPluginConfig config_;
  MobilityResponseCB mobility_response_publisher_;
  LaneChangeStatusCB lc_status_publisher_;
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;
  std::set<lanelet::Id> route_llt_ids_;
  lanelet::Id previous_llt_id_;
  std::vector<carma_perception_msgs::msg::ExternalObject> external_objects_;
  std::unordered_map<uint32_t, int> consecutive_clearance_count_for_obstacles_;
  // flag to show if it is possible for the vehicle to accept the cooperative request
  bool cooperative_request_acceptable_ = false;

  // incoming request trajectory information:
  std::vector <lanelet::BasicPoint2d> req_trajectory_points_;
  double req_target_speed_ = 0;
  double req_timestamp_ = 0;
  double req_target_plan_time_ = 0;
  int timesteps_since_last_req_ = 0;
  int clc_urgency_ = 0;
  std::optional<double> last_speed_ = std::nullopt;
  std::optional<rclcpp::Time> last_speed_time_ = std::nullopt;
  std::optional<rclcpp::Time> first_time_stopped_to_prevent_collision_ = std::nullopt;
  std::optional<
    carma_planning_msgs::msg::TrajectoryPlan> last_traj_plan_committed_to_stopping_ = std::nullopt;
  // time between ecef trajectory points
  double ecef_traj_timestep_ = 0.1;

  geometry_msgs::msg::Vector3 host_vehicle_size;
  double current_speed_;
  // BSM Message
  std::string host_bsm_id_;

  std::string georeference_{""};
  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

  std::string bsmIDtoString(carma_v2x_msgs::msg::BSMCoreData bsm_core)
  {
    std::string res = "";
    for (size_t i=0; i<bsm_core.id.size(); i++)
    {
      res+=std::to_string(bsm_core.id[i]);
    }
      return res;
  }

};
}  // namespace yield_plugin
