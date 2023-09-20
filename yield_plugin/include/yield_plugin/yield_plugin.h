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

#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/MobilityRequest.h>
#include <cav_msgs/MobilityResponse.h>
#include <cav_msgs/BSM.h>
#include <cav_msgs/LaneChangeStatus.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include "yield_config.h"
#include <unordered_set>
#include <carma_wm/WorldModel.h>
#include <carma_wm/collision_detection.h>
#include <trajectory_utils/quintic_coefficient_calculator.h>
#include <geometry_msgs/PoseStamped.h>
#include <boost/property_tree/json_parser.hpp>
#include <carma_wm/TrafficControl.h>
#include <lanelet2_extension/projection/local_frame_projector.h>
#include <std_msgs/String.h>


namespace yield_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using MobilityResponseCB = std::function<void(const cav_msgs::MobilityResponse&)>;
using LaneChangeStatusCB = std::function<void(const cav_msgs::LaneChangeStatus&)>;

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
   * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
   * \param mobility_response_publisher Callback which will publish the mobility response
   * \param lc_status_publisher Callback which will publish the cooperative lane change status
   */ 
  YieldPlugin(carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher, 
                       MobilityResponseCB mobility_response_publisher,
                       LaneChangeStatusCB lc_status_publisher);

  /**
   * \brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
   * 
   * \return True if the node should continue running. False otherwise
   */ 
  bool onSpin();
  

  /**
   * \brief Service callback for trajectory planning
   * 
   * \param req The service request
   * \param resp The service response
   * 
   * \return True if success. False otherwise
   */ 
  bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

  /**
   * \brief trajectory is modified to safely avoid obstacles on the road
   * \param original_tp original trajectory plan without object avoidance
   * \param current_speed_ current speed of the vehicle
   * \return modified trajectory plan
   */
  cav_msgs::TrajectoryPlan update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, double initial_velocity);

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
  double max_trajectory_speed(const std::vector<cav_msgs::TrajectoryPlanPoint>& trajectory_points) const;

  /**
   * \brief calculates distance between trajectory points in a plan
   * \param trajectory_plan input trajectory plan 
   * \return vector of relative distances between trajectory points
   */                     
  std::vector<double> get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan) const;  

  /**
   * \brief callback for mobility request
   * \param msg mobility request message 
   */
  void mobilityrequest_cb(const cav_msgs::MobilityRequestConstPtr& msg);

  /**
   * \brief callback for bsm message
   * \param msg mobility bsm message 
   */
  void bsm_cb(const cav_msgs::BSMConstPtr& msg);

  /**
   * \brief convert a carma trajectory from ecef frame to map frame
   * ecef trajectory consists of the point and a set of offsets with reference to the point
   * \param ecef_trajectory carma trajectory (ecef frame)
   * \return vector of 2d points in map frame
   */
  std::vector<lanelet::BasicPoint2d> convert_eceftrajectory_to_mappoints(const cav_msgs::Trajectory& ecef_trajectory) const;
  
  /**
   * \brief convert a point in ecef frame (in cm) into map frame (in meters)
   * \param ecef_point carma point ecef frame in cm
   * \param map_in_earth translate frame 
   * \return 2d point in map frame
   */
  lanelet::BasicPoint2d ecef_to_map_point(const cav_msgs::LocationECEF& ecef_point) const;

  /**
   * \brief compose a mobility response message
   * \param resp_recipient_id vehicle id of the recipient of the message
   * \param req_plan_id plan id from the requested message
   * \param response accept/reject to the response based on conditions
   * \return filled mobility response
   */
  cav_msgs::MobilityResponse compose_mobility_response(const std::string& resp_recipient_id, const std::string& req_plan_id, bool response) const;
  
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
  cav_msgs::TrajectoryPlan generate_JMT_trajectory(const cav_msgs::TrajectoryPlan& original_tp, double initial_pos, double goal_pos, double initial_velocity, double goal_velocity, double planning_time);
  
  /**
   * \brief update trajectory for yielding to an incoming cooperative behavior
   * \param original_tp original trajectory plan
   * \param current_speed current speed of the vehicle
   * \return updated trajectory for cooperative behavior
   */
  cav_msgs::TrajectoryPlan update_traj_for_cooperative_behavior(const cav_msgs::TrajectoryPlan& original_tp, double current_speed);

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
  double check_traj_for_digital_min_gap(const cav_msgs::TrajectoryPlan& original_tp) const;

  /**
   * \brief Callback for map projection string to define lat/lon -> map conversion
   * \brief msg The proj string defining the projection.
   */ 
  void georeferenceCallback(const std_msgs::StringConstPtr& msg);

  /**
   * \brief TODO
   * \brief msg The proj string defining the projection.
   */ 
  bool detectCollision(const cav_msgs::TrajectoryPlan& trajectory1, const std::vector<cav_msgs::PredictedState>& trajectory2, double collisionRadius);
private:

  carma_wm::WorldModelConstPtr wm_;
  YieldPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
  MobilityResponseCB mobility_response_publisher_;
  LaneChangeStatusCB lc_status_publisher_;
  std::set<lanelet::Id> route_llt_ids_;

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


  cav_msgs::Plugin plugin_discovery_msg_;
  geometry_msgs::Vector3 host_vehicle_size;
  double current_speed_;
  // BSM Message
  std::string host_bsm_id_;

  std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

  std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core)
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
