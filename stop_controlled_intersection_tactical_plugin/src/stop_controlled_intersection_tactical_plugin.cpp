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
#include <cav_msgs/StopAndWaitManeuver.h>
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
#include <stop_controlled_intersection_plugin.h>

using oss = std::ostringstream;

namespace stop_controlled_intersection_transit_plugin
{
StopControlledIntersectionTacticalPlugin::StopControlledIntersectionTacticalPlugin(carma_wm::WorldModelConstPtr wm,const StopControlledIntersectionTacticalPluginConfig& config,
                                    const PublishPluginDiscoveryCB& plugin_discovery_publisher)
  : wm_(wm), config_(config), plugin_discovery_publisher_(plugin_discovery_publisher)
  {
    plugin_discovery_msg_.name = "StopControlledIntersectionTacticalPlugin";
    plugin_discovery_msg_.version_id = "v1.0";
    plugin_discovery_msg_.available = true;
    plugin_discovery_msg_.activated = true;
    plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
    plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
  }

bool StopControlledIntersectionTacticalPlugin::onSpin()
{
    plugin_discovery_publisher_(plugin_discovery_msg_);
    return true;
}

bool StopControlledIntersectionTacticalPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
{
    ROS_DEBUG_STREAM("Starting stop controlled intersection trajectory planning");
    
    if(req.maneuver_index_to_plan >= req.maneuver_plan.maneuvers.size())
    {
    throw std::invalid_argument(
        "Stop Control Intersection Plugin asked to plan invalid maneuver index: " + std::to_string(req.maneuver_index_to_plan) + 
        " for plan of size: " + std::to_string(req.maneuver_plan.maneuvers.size()));
    }
    std::vector<cav_msgs::Maneuver> maneuver_plan;
    for(size_t i = req.maneuver_index_to_plan; i < req.maneuver_plan.maneuvers.size(); i++){
        
        if((req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::LANE_FOLLOWING || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT
        || req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN || req.maneuver_plan.maneuvers[i].type ==cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN) 
        && GET_MANEUVER_PROPERTY(req.maneuver_plan.maneuvers[i], parameters.string_valued_meta_data.front()) == stop_controlled_intersection_strategy_)
        {
            maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
            resp.related_maneuvers.push_back(req.maneuver_plan.maneuvers[i].type);
        }
        else
        {
            break;
        }
    }

    lanelet::BasicPoint2d veh_pos(req.vehicle_state.x_pos_global, req.vehicle_state.y_pos_global);
    ROS_DEBUG_STREAM("Planning state x:"<<req.vehicle_state.x_pos_global <<" , y: " << req.vehicle_state.y_pos_global);

    double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
    ROS_DEBUG_STREAM("Current_downtrack"<< current_downtrack);

    std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points( maneuver_plan, wm_, req.vehicle_state);
    ROS_DEBUG_STREAM("Maneuver to points size:"<< points_and_target_speeds.size());
    // ROS_DEBUG_STREAM("Printing points: ");
    // TODO: add print logic
    cav_msgs::TrajectoryPlan trajectory;
    trajectory.header.frame_id = "map";
    trajectory.header.stamp = req.header.stamp;
    trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

    //Add compose trajectory from centerline
    trajectory.trajectory_points = compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state, req.header.stamp);
    trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;

    // Set the planning plugin field name
    for (auto& p : trajectory.trajectory_points) {
        p.planner_plugin_name = plugin_discovery_msg_.name;
        // p.controller_plugin_name = "PurePursuit";
    }

    resp.trajectory_plan = trajectory;
    
    resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

    return true;
}

std::vector<PointSpeedPair> StopControlledIntersectionTacticalPlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                            const carma_wm::WorldModelConstPtr& wm, const cav_msgs::VehicleState& state)
{
    std::vector<PointSpeedPair> points_and_target_speeds;
    std::unordered_set<lanelet::Id> visited_lanelets;

    lanelet::BasicPoint2d veh_pos(state.x_pos_global, state.y_pos_global);
    double max_starting_downtrack = wm_->routeTrackPos(veh_pos).downtrack; //The vehicle position
    double starting_speed = state.longitudinal_vel;

    bool first = true;    
    double starting_downtrack;
    for (const auto& maneuver : maneuvers)
    {
        if(maneuver.type != cav_msgs::Maneuver::LANE_FOLLOWING && maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT && maneuver.type != cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN
        && maneuver.type !=cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN ){
            throw std::invalid_argument("Stop Controlled Intersection Tactical Plugin does not support this maneuver type");
        }
    
        if(first)
        {
            starting_downtrack = GET_MANEUVER_PROPERTY(maneuver, start_dist);
            if (starting_downtrack > max_starting_downtrack)
            {
                starting_downtrack = max_starting_downtrack;
            }
            first = false;
        }

        // Sample the lanelet centerline at fixed increments.
        // std::min call here is a guard against starting_downtrack being within 1m of the maneuver end_dist
        // in this case the sampleRoutePoints method will return a single point allowing execution to continue
        std::vector<lanelet::BasicPoint2d> route_points = wm->sampleRoutePoints(
            std::min(starting_downtrack + config_.centerline_sampling_spacing, GET_MANEUVER_PROPERTY(maneuver,end_dist)),
            GET_MANEUVER_PROPERTY(maneuver, end_dist), config_.centerline_sampling_spacing);
        
        route_points.insert(route_points.begin(), veh_pos);
        ROS_DEBUG_STREAM("Route geometery points size: "<<route_points.size());
        //get case num from maneuver parameters
        if(GET_MANEUVER_PROPERTY(maneuver,parameters.int_valued_meta_data).empty()){
            throw std::invalid_argument("No case number specified for stop controlled intersection maneuver");
        }
        
        int case_num = GET_MANEUVER_PROPERTY(maneuver,parameters.int_valued_meta_data[0]);
        if(case_num == 1){
            points_and_target_speeds = create_case_one_speed_profile(wm, maneuver, route_points, starting_speed, state);
        }
        else if(case_num == 2){
            points_and_target_speeds = create_case_two_speed_profile(wm, maneuver, route_points, starting_speed);
        }
        else if(case_num == 3)
        {
            points_and_target_speeds = create_case_three_speed_profile(wm, maneuver, route_points, starting_speed);
        }
        else{
            throw std::invalid_argument("The stop controlled intersection tactical plugin doesn't handle the case number requested");
        }
        
        
    }

    return points_and_target_speeds;
}

std::vector<PointSpeedPair> StopControlledIntersectionTacticalPlugin::create_case_one_speed_profile(const carma_wm::WorldModelConstPtr& wm,
const cav_msgs::Maneuver& maneuver, std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed, const cav_msgs::VehicleState& state){

    ROS_DEBUG_STREAM("Planning for Case One");
    //Derive meta data values from maneuver message - Using order in sci_strategic_plugin
    double a_acc = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
    double a_dec = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]); //a_dec is a -ve value
    double t_acc = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[2]);
    double t_dec = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[3]);
    double speed_before_decel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[4]);
    ROS_DEBUG_STREAM("a_acc received: "<< a_acc);
    ROS_DEBUG_STREAM("a_dec received: "<< a_dec);
    ROS_DEBUG_STREAM("t_acc received: "<< t_acc);
    ROS_DEBUG_STREAM("t_dec received: "<< t_dec);
    ROS_DEBUG_STREAM("speed before decel received: "<< speed_before_decel);
    
    //Derive start and end dist from maneuver
    double start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
    double end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);

    ROS_DEBUG_STREAM("Maneuver starting downtrack: "<< start_dist);
    ROS_DEBUG_STREAM("Maneuver ending downtrack: "<< end_dist);
    //Checking state against start_dist and adjust profile
    lanelet::BasicPoint2d state_point(state.x_pos_global, state.y_pos_global);
    double route_starting_downtrack = wm->routeTrackPos(state_point).downtrack;  //Starting downtrack based on geometry points
    double dist_acc;        //Distance for which acceleration lasts

    if(route_starting_downtrack < start_dist){
        //Update parameters
        //Keeping the deceleration part the same
        ROS_DEBUG_STREAM("Starting distance is less than maneuver start, updating parameters");
        double dist_decel = pow(speed_before_decel, 2)/(2*std::abs(a_dec));

        dist_acc = end_dist - dist_decel;
        a_acc = (pow(speed_before_decel, 2) - pow(starting_speed,2))/(2*dist_acc);
        ROS_DEBUG_STREAM("Updated a_acc: "<< a_acc);
    }
    else{
        //Use parameters from maneuver message
        dist_acc = (pow(speed_before_decel, 2) - pow(starting_speed, 2))/(2*a_acc);
    }

    std::vector<PointSpeedPair> points_and_target_speeds;
    PointSpeedPair first_point;
    first_point.point = state_point;
    first_point.speed = starting_speed;
    points_and_target_speeds.push_back(first_point);

    lanelet::BasicPoint2d prev_point = state_point;
    double total_dist_covered = 0;                  //Starting dist for maneuver treated as 0.0

    for(size_t i = 1; i < route_geometry_points.size(); i++){
        lanelet::BasicPoint2d current_point = route_geometry_points[i];
        double delta_d = lanelet::geometry::distance2d(prev_point, current_point);
        total_dist_covered += delta_d;      
        //Find speed at dist covered
        double speed_i; 
        if(total_dist_covered <= dist_acc){
            //Acceleration part
            speed_i = sqrt(pow(starting_speed,2) + 2*a_acc*total_dist_covered);
        }
        else{
            //Deceleration part
            speed_i = sqrt(std::max(pow(speed_before_decel,2) + 2*a_dec*(total_dist_covered - dist_acc),0.0)); //std::max to ensure negative value is not sqrt
            if(speed_i < epsilon_){
                speed_i = 0.0;
            }
        }

        PointSpeedPair p;
        if(speed_i < epsilon_){
            p.point = prev_point;
            p.speed = 0.0;
        }
        else{
            p.point = route_geometry_points[i];
            p.speed = speed_i;
            prev_point = current_point; //Advance prev point if speed changes
        }
        points_and_target_speeds.push_back(p);

    }

    return points_and_target_speeds;

}

std::vector<PointSpeedPair> StopControlledIntersectionTacticalPlugin::create_case_two_speed_profile(const carma_wm::WorldModelConstPtr& wm,
const cav_msgs::Maneuver& maneuver, std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed){
    ROS_DEBUG_STREAM("Planning for Case Two");
    //Derive meta data values from maneuver message - Using order in sci_strategic_plugin
    double a_acc = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);
    double a_dec = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[1]); //a_dec is a -ve value
    double t_acc = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[2]);
    double t_dec = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[3]);
    double t_cruise = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[4]);
    double speed_before_decel = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[5]);

    //Derive start and end dist from maneuver
    double start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
    double end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
    ROS_DEBUG_STREAM("Maneuver starting downtrack: "<< start_dist);
    ROS_DEBUG_STREAM("Maneuver ending downtrack: "<< end_dist);

    //Checking route geometry start against start_dist and adjust profile
    double route_starting_downtrack = wm->routeTrackPos(route_geometry_points[0]).downtrack;  //Starting downtrack based on geometry points
    double dist_acc;        //Distance over which acceleration happens
    double dist_cruise;     //Distance over which cruising happens
    double dist_decel;      //Distance over which deceleration happens

    if(route_starting_downtrack < start_dist){
        //update parameters
        //Keeping acceleration and deceleration part same as planned in strategic plugin
        ROS_DEBUG_STREAM("Starting distance is less than maneuver start, updating parameters");
        dist_acc = starting_speed*t_acc + 0.5 * a_acc * pow(t_acc,2);
        dist_decel = speed_before_decel*t_dec + 0.5 * a_dec * pow(t_dec,2);
        dist_cruise = end_dist - route_starting_downtrack - (dist_acc + dist_decel);
    }
    else{   
        //Use maneuver parameters to create speed profile
        dist_acc = starting_speed*t_acc + 0.5 * a_acc * pow(t_acc,2);
        dist_cruise = speed_before_decel*t_cruise;
        dist_decel = speed_before_decel*t_dec + 0.5 * a_dec * pow(t_dec,2);
    }

    //Check calculated total dist against maneuver limits
    double total_distance_needed = dist_acc + dist_cruise + dist_decel;
    if(total_distance_needed - (end_dist - start_dist) > epsilon_ ){
        //Requested maneuver needs to be modified to meet start and end dist req
        //Sacrifice on cruising and then acceleration if needed
        ROS_DEBUG_STREAM("Updating maneuver to meet start and end dist req.");
        double delta_total_dist = total_distance_needed - (end_dist - start_dist);
        dist_cruise -= delta_total_dist;
        if(dist_cruise < 0){
            dist_acc += dist_cruise;
            dist_cruise = 0;
        }
        //Not considering dist_acc < 0 after this.
    }

    std::vector<PointSpeedPair> points_and_target_speeds;
    PointSpeedPair first_point;
    first_point.point = route_geometry_points[0];
    first_point.speed = starting_speed;
    points_and_target_speeds.push_back(first_point);

    lanelet::BasicPoint2d prev_point = route_geometry_points.front();
    double total_dist_planned = 0;                  //Starting dist for maneuver treated as 0.0
    double prev_speed = starting_speed;
    for(auto route_point : route_geometry_points){
        lanelet::BasicPoint2d current_point = route_point;
        double delta_d = lanelet::geometry::distance2d(prev_point, current_point);
        total_dist_planned += delta_d;  

        //Find speed at dist covered
        double speed_i;
        if(total_dist_planned < dist_acc){
            //Acceleration part
            speed_i = sqrt(pow(starting_speed,2) + 2*a_acc*total_dist_planned);
        }
        else if(dist_cruise > 0 && total_dist_planned >= dist_acc && total_dist_planned <= (dist_acc + dist_cruise)){
            //Cruising part
            speed_i = prev_speed;
        }
        else{
            //Deceleration part
            speed_i = sqrt(std::max(pow(speed_before_decel,2) + 2*a_dec*(total_dist_planned - dist_acc - dist_cruise),0.0));//std::max to ensure negative value is not sqrt
        }
        
        PointSpeedPair p;
        if(speed_i < epsilon_){
            p.point = prev_point;
            p.speed = 0.0;
        }
        else{
            p.point = route_point;
            p.speed = std::min(speed_i,speed_before_decel);
            prev_point = current_point; //Advance prev point if speed changes
        }
        points_and_target_speeds.push_back(p);

        prev_speed = speed_i;
    }

    return points_and_target_speeds;

}

std::vector<PointSpeedPair> StopControlledIntersectionTacticalPlugin::create_case_three_speed_profile(const carma_wm::WorldModelConstPtr& wm,
const cav_msgs::Maneuver& maneuver, std::vector<lanelet::BasicPoint2d>& route_geometry_points, double starting_speed){
    ROS_DEBUG_STREAM("Planning for Case three");
    //Derive meta data values from maneuver message - Using order in sci_strategic_plugin
    double a_dec = GET_MANEUVER_PROPERTY(maneuver, parameters.float_valued_meta_data[0]);

    //Derive start and end dist from maneuver
    double start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
    double end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);
    ROS_DEBUG_STREAM("Maneuver starting downtrack: "<< start_dist);
    ROS_DEBUG_STREAM("Maneuver ending downtrack: "<< end_dist);

    //Checking route geometry start against start_dist and adjust profile
    double route_starting_downtrack = wm->routeTrackPos(route_geometry_points[0]).downtrack;  //Starting downtrack based on geometry points

    if(route_starting_downtrack < start_dist){
        //update parameter
        ROS_DEBUG_STREAM("Starting distance is less than maneuver start, updating parameters");
        a_dec = pow(starting_speed, 2)/(2*(end_dist - route_starting_downtrack));
    }

    std::vector<PointSpeedPair> points_and_target_speeds;
    PointSpeedPair first_point;
    first_point.point = route_geometry_points[0];
    first_point.speed = starting_speed;
    points_and_target_speeds.push_back(first_point);

    lanelet::BasicPoint2d prev_point = route_geometry_points[0];
    double total_dist_covered = 0;          //Starting dist for maneuver treated as 0.0

    for(size_t i = 0;i < route_geometry_points.size(); i++){
        lanelet::BasicPoint2d current_point = route_geometry_points[i];
        double delta_d = lanelet::geometry::distance2d(prev_point, current_point);
        total_dist_covered +=delta_d;
        //Find speed at dist covered
        double speed_i = sqrt(std::max(pow(starting_speed,2) + 2 * a_dec * total_dist_covered, 0.0)); //std::max to ensure negative value is not sqrt
        
        PointSpeedPair p;
        
        if(speed_i < epsilon_){
            p.point = prev_point;
            p.speed = 0.0;
        }
        else{
            p.point = route_geometry_points[i];
            p.speed = speed_i;
            prev_point = current_point; //Advance prev point if speed changes
        }
        
        points_and_target_speeds.push_back(p);

        
    }

    return points_and_target_speeds;
}

std::vector<cav_msgs::TrajectoryPlanPoint> StopControlledIntersectionTacticalPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time){
    
    std::vector<cav_msgs::TrajectoryPlanPoint> trajectory;
    ROS_DEBUG_STREAM("VehicleState: "
                        << " x: " << state.x_pos_global << " y: " << state.y_pos_global << " yaw: " << state.orientation
                        << " speed: " << state.longitudinal_vel);
    
    int nearest_pt_index = basic_autonomy::waypoint_generation::get_nearest_point_index(points, state);
    ROS_DEBUG_STREAM("Nearest pt index: "<<nearest_pt_index);
    std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); //Points in front of current vehicle position
    ROS_DEBUG_STREAM("Future points size: "<<future_points.size());
    auto time_bound_points = basic_autonomy::waypoint_generation::constrain_to_time_boundary(future_points, config_.trajectory_time_length);
    ROS_DEBUG_STREAM("Got time bound points with size:" << time_bound_points.size());

    //Attach past points
    std::vector<PointSpeedPair> back_and_future = attach_past_points(points, time_bound_points, nearest_pt_index, config_.back_distance);
    ROS_DEBUG_STREAM("Got back_and_future points with size: "<<back_and_future.size());

    std::vector<double> speed_limits;
    std::vector<lanelet::BasicPoint2d> curve_points;
    split_point_speed_pairs(time_bound_points, &curve_points, &speed_limits);

    std::unique_ptr<basic_autonomy::smoothing::SplineI> fit_curve = basic_autonomy::waypoint_generation::compute_fit(curve_points); //Compute splines based on curve points
    if(!fit_curve)
    {
        throw std::invalid_argument("Could not fit a spline curve along the trajectory!");
    }

    ROS_DEBUG("Got fit");
    ROS_DEBUG_STREAM("Speed_limits.size(): "<<speed_limits.size());

    std::vector<lanelet::BasicPoint2d> all_sampling_points;
    all_sampling_points.reserve(1 + curve_points.size() * 2);

    std::vector<double> distributed_speed_limits;
    distributed_speed_limits.reserve(1+ curve_points.size() * 2);

    //Compute total length of the trajectory to get correct number of points
    // we expect using curve resample step size
    std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

    auto total_step_along_curve = static_cast<int>(downtracks_raw.back() / config_.curve_resample_step_size);

    int current_speed_index = 0;
    size_t total_point_size = curve_points.size();

    double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
    double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
    std::vector<double> better_curvature;
    better_curvature.reserve(1 + curve_points.size() * 2);

    for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
    {
        lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
        all_sampling_points.push_back(p);
        double c = basic_autonomy::waypoint_generation::compute_curvature_at((*fit_curve), scaled_steps_along_curve);
        better_curvature.push_back(c);

        if((double) steps_along_curve > step_threshold_for_next_speed)
        {
            step_threshold_for_next_speed += (double)total_step_along_curve / (double)total_point_size;
            current_speed_index++;
        }
        distributed_speed_limits.push_back(speed_limits[current_speed_index]);  //Identify speed limits for resampled points
        scaled_steps_along_curve += 1.0 / total_step_along_curve;               //adding steps_along_curve_step_size
    }

    ROS_DEBUG_STREAM("Got sampled points with size:" << all_sampling_points.size());

    std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(all_sampling_points);

    std::vector<double> curvatures = basic_autonomy::smoothing::moving_average_filter(better_curvature, config_.curvature_moving_average_window_size, false);
    std::vector<double> ideal_speeds =
        trajectory_utils::constrained_speeds_for_curvatures(curvatures, config_.lateral_accel_limit);

    std::vector<double> constrained_speed_limits = basic_autonomy::waypoint_generation::apply_speed_limits(ideal_speeds, distributed_speed_limits); //Speed min(ideal, calculated)
    ROS_DEBUG("Processed all points in computed fit");
    std::vector<double> final_actual_speeds = constrained_speed_limits;

    if (all_sampling_points.empty())
    {
        ROS_WARN_STREAM("No trajectory points could be generated");
        return {};
    }

    //Drop Past points
    nearest_pt_index = basic_autonomy::waypoint_generation::get_nearest_index_by_downtrack(all_sampling_points, wm_, state);
    std::vector<lanelet::BasicPoint2d> future_basic_points(all_sampling_points.begin() + nearest_pt_index + 1,
                                                            all_sampling_points.end());
    std::vector<double> future_speeds(final_actual_speeds.begin() + nearest_pt_index + 1, 
                                    final_actual_speeds.end());                                                            
    std::vector<double> future_yaw(final_yaw_values.begin() + nearest_pt_index + 1,
                                    final_yaw_values.end());

    // Add current vehicle point to front of the trajectory
    lanelet::BasicPoint2d cur_veh_point(state.x_pos_global, state.y_pos_global);

    future_basic_points.insert(future_basic_points.begin(),
                                cur_veh_point); // Add current vehicle position to front of sample points
    future_speeds.insert(future_speeds.begin(), state.longitudinal_vel);
    future_yaw.insert(future_yaw.begin(), state.orientation);

    // Compute points to local downtracks
    std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(future_basic_points);

    final_actual_speeds = basic_autonomy::smoothing::moving_average_filter(future_speeds, config_.speed_moving_average_window_size);

    // Convert speeds to times
    std::vector<double> times;

    //Force last point speed to 0.0 if close to end
    if(lanelet::geometry::distance2d(future_basic_points.back(), points.back().point) < epsilon_){
        final_actual_speeds.back() = 0.0;
    }

    trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

    // Build trajectory points
    std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
        basic_autonomy::waypoint_generation::trajectory_from_points_times_orientations(future_basic_points, times, future_yaw, state_time, "default");

    return traj_points;
}


}