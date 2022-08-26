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
#include <carma_wm_ros2/WMListener.hpp>
#include <functional>
#include "yield_config.hpp"
#include <unordered_set>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_utils/quintic_coefficient_calculator.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_v2x_msgs/msg/location_ecef.hpp>
#include <carma_v2x_msgs/msg/trajectory.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <carma_wm_ros2/Geometry.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_wm_ros2/collision_detection.hpp>
#include <carma_wm_ros2/TrafficControl.hpp>


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
   * \return modified trajectory plan
   */
  carma_planning_msgs::msg::TrajectoryPlan update_traj_for_object(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double initial_velocity);

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
   * \return maximum speed
   */
  double max_trajectory_speed(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points) const;

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
   * \return updated JMT trajectory 
   */
  carma_planning_msgs::msg::TrajectoryPlan generate_JMT_trajectory(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double initial_pos, double goal_pos, double initial_velocity, double goal_velocity, double planning_time);
  
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
  void lookupECEFtoMapTransform();

  /**
   * \brief checks trajectory for minimum gap associated with it
   * \param original_tp original trajectory plan
   * \return minumum required
   */
  double check_traj_for_digital_min_gap(const carma_planning_msgs::msg::TrajectoryPlan& original_tp) const;

  /**
   * \brief Callback for map projection string to define lat/lon -> map conversion
   * \brief msg The proj string defining the projection.
   */ 
  void georeferenceCallback(const std_msgs::msg::String::UniquePtr msg);


private:

  carma_wm::WorldModelConstPtr wm_;
  YieldPluginConfig config_;
  MobilityResponseCB mobility_response_publisher_;
  LaneChangeStatusCB lc_status_publisher_;
  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;

  // flag to show if it is possible for the vehicle to accept the cooperative request
  bool cooperative_request_acceptable_ = false;

  // incoming request trajectory information:
  std::vector <lanelet::BasicPoint2d> req_trajectory_points_;
  double req_target_speed_ = 0;
  double req_timestamp_ = 0;
  double req_target_plan_time_ = 0;
  int timesteps_since_last_req_ = 0;
  int clc_urgency_ = 0;

  // time between ecef trajectory points
  double ecef_traj_timestep_ = 0.1;

  geometry_msgs::msg::Vector3 host_vehicle_size;
  double current_speed_;
  // BSM Message
  std::string host_bsm_id_;

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

  // TODO replace with Basic Autonomy moving average filter
  std::vector<double> moving_average_filter(const std::vector<double> input, int window_size, bool ignore_first_point=true)
  {
    if (window_size % 2 == 0) {
      throw std::invalid_argument("moving_average_filter window size must be odd");
    }

    std::vector<double> output;
    output.reserve(input.size());

    if (input.size() == 0) {
      return output;
    }

    int start_index = 0;
    if (ignore_first_point) {
      start_index = 1;
      output.push_back(input[0]);
    }

    for (int i = start_index; i<input.size(); i++) {
            
      double total = 0;
      int sample_min = std::max(0, i - window_size / 2);
      int sample_max = std::min((int) input.size() - 1 , i + window_size / 2);

      int count = sample_max - sample_min + 1;
      std::vector<double> sample;
      sample.reserve(count);
      for (int j = sample_min; j <= sample_max; j++) {
        total += input[j];
      }
      output.push_back(total / (double) count);

    }

    return output;
  }

};
};  // namespace yield_plugin
