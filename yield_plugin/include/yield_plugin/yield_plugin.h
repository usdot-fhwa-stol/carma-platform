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
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <tf2_ros/transform_listener.h>
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


namespace yield_plugin
{
using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;
using MobilityResponseCB = std::function<void(const cav_msgs::MobilityResponse&)>;

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
   */ 
  YieldPlugin(carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                       PublishPluginDiscoveryCB plugin_discovery_publisher, MobilityResponseCB mobility_response_publisher);

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
  cav_msgs::TrajectoryPlan update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, double current_speed_);

  /**
   * \brief calculate quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of polynomial for given input
   */
  double polynomial_calc(std::vector<double> coeff, double x);

  /**
   * \brief calculate derivative of quintic polynomial equation for a given x
   * \param coeff vector including polynomial coefficiencrs
   * \param x input variable to the polynomial
   * \return value of derivative polynomial for given input
   */
  double polynomial_calc_d(std::vector<double> coeff, double x);

  /**
   * \brief calculates the maximum speed in a set of tajectory points
   * \param trajectory_points trajectory points
   * \return maximum speed
   */
  double max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points) ;

  /**
   * \brief calculates distance between trajectory points in a plan
   * \param trajectory_plan input trajectory plan 
   * \return vector of relative distances between trajectory points
   */                     
  std::vector<double> get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan);  

    /////Descriotuibn
  void mobilityrequest_cb(const cav_msgs::MobilityRequestConstPtr& msg);
  void bsm_cb(const cav_msgs::BSMConstPtr& msg);
  std::vector<lanelet::BasicPoint2d> convert_eceftrajectory_to_mappoints(const cav_msgs::Trajectory& ecef_trajectory, const geometry_msgs::TransformStamped& tf);
  cav_msgs::MobilityResponse compose_mobility_response(std::string resp_recipient_id, std::string req_plan_id);
  cav_msgs::TrajectoryPlan generate_JMT_trajectory(const cav_msgs::TrajectoryPlan& original_tp, double initial_pos, double goal_pos, double initial_velocity, double goal_velocity, double planning_time);
  cav_msgs::TrajectoryPlan update_traj_for_cooperative_behavior(const cav_msgs::TrajectoryPlan& original_tp, double current_speed);
  std::vector<lanelet::BasicPoint2d> detect_trajectories_intersection(std::vector<lanelet::BasicPoint2d> trajectory1, std::vector<lanelet::BasicPoint2d> trajectory2);
  void set_incoming_request_info(std::vector <lanelet::BasicPoint2d> req_trajectory, double req_speed, double req_planning_time);
  
  

private:

  carma_wm::WorldModelConstPtr wm_;
  YieldPluginConfig config_;
  PublishPluginDiscoveryCB plugin_discovery_publisher_;
  MobilityResponseCB mobility_response_publisher_;

  // incoming request trajectory information:
  std::vector <lanelet::BasicPoint2d> req_trajectory_points_;
  double req_target_speed_;
  double req_target_plan_time_;


  cav_msgs::Plugin plugin_discovery_msg_;
  geometry_msgs::Vector3 host_vehicle_size;
  double current_speed_;
  // BSM Message
  std::string host_bsm_id_;
  std::string host_id_;

  // TF listenser
  tf2_ros::Buffer tf2_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;

  bool received_cooperative_request_ = false;

  std::string bsmIDtoString(cav_msgs::BSMCoreData bsm_core)
  {
    std::string res = "";
    for (size_t i=0; i<bsm_core.id.size(); i++)
    {
      res+=std::to_string(bsm_core.id[i]);
    }
      return res;
  }

};
};  // namespace yield_plugin
