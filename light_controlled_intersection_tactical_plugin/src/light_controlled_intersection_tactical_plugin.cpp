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

#include <ros/ros.h>
#include <string>
#include <algorithm>
#include <memory>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <sstream>
#include <carma_utils/containers/containers.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <unordered_set>
#include <vector>
#include <cav_msgs/Trajectory.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <carma_wm/CARMAWorldModel.h>
#include <carma_utils/containers/containers.h>
#include <carma_wm/Geometry.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <light_controlled_intersection_plugin.h>

using oss = std::ostringstream;

namespace light_controlled_intersection_transit_plugin
{
LightControlledIntersectionTacticalPlugin::LightControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm,const LightControlledIntersectionTacticalPluginConfig& config,
                                    const PublishPluginDiscoveryCB& plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
  {
    plugin_discovery_msg_.name = "LightControlledIntersectionTacticalPlugin";
    plugin_discovery_msg_.versionId = "v1.0";
    plugin_discovery_msg_.available = true;
    plugin_discovery_msg_.activated = false;
    plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
    plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
  }

bool LightControlledIntersectionTacticalPlugin::onSpin()
{
    plugin_discovery_publisher_(plugin_discovery_msg_);
    return true;
}

bool LightControlledIntersectionTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
{
    ROS_DEBUG_STREAM("Starting light controlled intersection trajectory planning");
    
    if(req.maneuver_index_to_plan >= req.maneuver_plan.maneuvers.size())
    {
    throw std::invalid_argument(
        "Light Control Intersection Plugin asked to plan invalid maneuver index: " + std::to_string(req.maneuver_index_to_plan) + 
        " for plan of size: " + std::to_string(req.maneuver_plan.maneuvers.size()));
    }
    std::vector<cav_msgs::Maneuver> maneuver_plan;
    for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++){
        
        if((req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_FOLLOWING || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT
        || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || req.maneuver_plan.maneuvers[i].type ==cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN) 
        && GET_MANEUVER_PROPERTY(req.maneuver_plan.maneuvers[i], parameters.string_valued_meta_data.front()) == light_controlled_intersection_strategy_)
        {
            maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
            resp.related_maneuvers.push_back(req.maneuver_plan.maneuvers[i].type);
        }
        else
        {
            break;
        }
    }

    lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
    ROS_DEBUG_STREAM("Planning state x:"<<req.vehicle_state.X_pos_global <<" , y: " << req.vehicle_state.Y_pos_global);

    double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
    ROS_DEBUG_STREAM("Current_downtrack"<< current_downtrack);

    std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points( maneuver_plan, wm_, req.vehicle_state);

    //Trajectory Plan
    cav_msgs::TrajectoryPlan trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = req.header.stamp;
    trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

    //Add compose trajectory from centerline
    trajectory.trajectory_points = compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state, req.header.stamp);
    trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;

    resp.trajectory_plan = trajectory;
    
    resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

    return true;
}

double LightControlledIntersectionTacticalPlugin::calcEstimatedEntryTimeLeft(double entry_dist, double current_speed, double speed_limit) const
{
  double t_entry = 0;
  t_entry = 2*entry_dist/(current_speed + speed_limit);
  return t_entry;
}

int LightControlledIntersectionTacticalPlugin::determineSpeedProfileCase(double entry_dist, double current_speed, double schedule_entry_time, double speed_limit)
{
  int case_num = 0;
  double estimated_entry_time = calcEstimatedEntryTimeLeft(entry_dist, current_speed, speed_limit );
  double speed_before_decel = calcSpeedBeforeDecel(estimated_entry_time, entry_dist, current_speed, speed_limit);
  double speed_before_accel = calcSpeedBeforeAccel(estimated_entry_time, entry_dist, current_speed, speed_limit);
  
  ROS_DEBUG_STREAM("estimated_entry_time: " << estimated_entry_time << ", and schedule_entry_time: " << schedule_entry_time);
  if (estimated_entry_time < schedule_entry_time)
  {
    ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << ", and speed_limit: " << speed_limit);

    if (speed_before_accel < speed_limit)
    {
      case_num = 4;
    }
    else
    {
      case_num = 3;
    }
  }
  else
  {
    ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << ", and speed_limit: " << speed_limit);

    if (speed_before_decel <= speed_limit)
    {
      case_num = 2;
    }
    else
    {
      case_num = 1;
    }
  }
  
  return case_num;
}


std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                            const carma_wm::WorldModelConstPtr& wm, const cav_msgs::VehicleState& state)
{
    std::vector<PointSpeedPair> points_and_target_speeds;
    std::unordered_set<lanelet::Id> visited_lanelets;

    lanelet::BasicPoint2d veh_pos(state.X_pos_global, state.Y_pos_global);
    double max_starting_downtrack = wm_->routeTrackPos(veh_pos).downtrack; //The vehicle position
    double starting_speed = state.longitudinal_vel;

    bool first = true;    
    double starting_downtrack;

    if (maneuvers.size() > 1)
    {
        ROS_WARN_STREAM("maneuvers_to_points received more than 1 maneuver, and only last one will be planned");
    }
    
    for (const auto& maneuver : maneuvers)
    {
        if(maneuver.type != cav_msgs::Maneuver::LANE_FOLLOWING ){ //detect special case for light controlled. 
            throw std::invalid_argument("Light Controlled Intersection Tactical Plugin does not support this maneuver type");
        }
        // assume they are all correct
        if(first)
        {
            starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist); 
            if (starting_downtrack > max_starting_downtrack)
            {
                starting_downtrack = max_starting_downtrack;
            }
            first = false;
        }

        // Identify the lanelets which will be crossed by approach maneuvers lane follow maneuver
        std::vector<lanelet::ConstLanelet> crossed_lanelets =
            getLaneletsBetweenWithException(starting_downtrack, GET_MANEUVER_PROPERTY(maneuver,end_dist), true, true);

        // approaching stop line speed
        speed_limit_ = findSpeedLimit(crossed_lanelets.back());

        // check error
        if(GET_MANEUVER_PROPERTY(maneuver,parameters.float_valued_meta_data).empty()){
            throw std::invalid_argument("No time_to_schedule_entry is provided in float_valued_meta_data");
        }

        // lane following to intersection
        double time_to_schedule_entry = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
        int case_num = determineSpeedProfileCase(GET_MANEUVER_PROPERTY(maneuver,end_dist) - starting_downtrack,starting_speed , time_to_schedule_entry, speed_limit_);

        // currently assumed to receive just 1 maneuver
        if(case_num == 1){
            points_and_target_speeds = create_case_one_speed_profile();
        }
        else if(case_num == 2){
            points_and_target_speeds = create_case_two_speed_profile();
        }
        else if(case_num == 3)
        {
            points_and_target_speeds = create_case_three_speed_profile();
        }
        else if(case_num == 4)
        {
            points_and_target_speeds = create_case_four_speed_profile();
        }
        else{
            throw std::invalid_argument("The light controlled intersection tactical plugin doesn't handle the case number requested");
        }
    }

    return points_and_target_speeds;
}

double LightControlledIntersectionTacticalPlugin::findSpeedLimit(const lanelet::ConstLanelet& llt) const
{
  lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
  if (traffic_rules)
  {
    return (*traffic_rules)->speedLimit(llt).speedLimit.value();
  }
  else
  {
    throw std::invalid_argument("Valid traffic rules object could not be built");
  }
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeDecel(double entry_time, double entry_dist, double current_speed, double speed_limit) const
{
  double speed_before_decel = 0;

  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  double acc_dec_ratio = desired_acceleration/desired_deceleration;
  double required_speed = entry_dist / entry_time;

  double sqr_term = sqrt(pow(1 - (acc_dec_ratio), 2) * pow(required_speed, 2) - (1 -acc_dec_ratio) *
                        (acc_dec_ratio * speed_limit * (speed_limit - 2 * required_speed) + current_speed * (2* required_speed - current_speed)));

  speed_before_decel = required_speed + sqr_term/(1 - acc_dec_ratio);

  return speed_before_decel;
}

double LightControlledIntersectionTacticalPlugin::calcSpeedBeforeAccel(double entry_time, double entry_dist, double current_speed, double speed_limit) const
{
  double speed_before_accel = 0;

  double desired_acceleration = config_.vehicle_accel_limit * config_.vehicle_accel_limit_multiplier;
  double desired_deceleration = -1 * config_.vehicle_decel_limit * config_.vehicle_decel_limit_multiplier;
  double acc_dec_ratio = desired_acceleration/desired_deceleration;
  double required_speed = entry_dist / entry_time;

  double sqr_term = sqrt(pow((acc_dec_ratio - 1), 2) * pow(required_speed, 2) - (acc_dec_ratio - 1) *
                        (speed_limit * (speed_limit - 2 * required_speed) + acc_dec_ratio * current_speed * (2* required_speed - current_speed)));

  speed_before_accel = required_speed + sqr_term/(acc_dec_ratio - 1);

  return speed_before_accel;
}

std::vector<lanelet::ConstLanelet> LightControlledIntersectionTacticalPlugin::getLaneletsBetweenWithException(double start_downtrack,
                                                                                      double end_downtrack,
                                                                                      bool shortest_path_only,
                                                                                      bool bounds_inclusive) const
{
  std::vector<lanelet::ConstLanelet> crossed_lanelets =
      wm_->getLaneletsBetween(start_downtrack, end_downtrack, shortest_path_only, bounds_inclusive);

  if (crossed_lanelets.empty())
  {
    throw std::invalid_argument("getLaneletsBetweenWithException called but inputs do not cross any lanelets going "
                                "from: " +
                                std::to_string(start_downtrack) + " to: " + std::to_string(end_downtrack));
  }

  return crossed_lanelets;
}

std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::create_case_one_speed_profile(){
    std::vector<PointSpeedPair> points_and_target_speeds;
    return points_and_target_speeds;
}
std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::create_case_two_speed_profile(){
    std::vector<PointSpeedPair> points_and_target_speeds;
    return points_and_target_speeds;
}
std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::create_case_three_speed_profile(){
    std::vector<PointSpeedPair> points_and_target_speeds;
    return points_and_target_speeds;
}
std::vector<PointSpeedPair> LightControlledIntersectionTacticalPlugin::create_case_four_speed_profile(){
    std::vector<PointSpeedPair> points_and_target_speeds;
    return points_and_target_speeds;
}

std::vector<cav_msgs::TrajectoryPlanPoint> LightControlledIntersectionTacticalPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time){
    
    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory;
    
    return trajectory;
}


}