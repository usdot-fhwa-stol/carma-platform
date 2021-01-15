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
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include "unobstructed_lanechange.h"
#include <unordered_set>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <carma_utils/containers/containers.h>
#include "smoothing/filters.h"

namespace unobstructed_lanechange
{           
    UnobstructedLaneChangePlugin::UnobstructedLaneChangePlugin(){}

    void UnobstructedLaneChangePlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/UnobstructedLaneChangePlugin/plan_trajectory", &UnobstructedLaneChangePlugin::plan_trajectory_cb, this);
                
        ubobstructed_lanechange_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "UnobstructedLaneChangePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            ubobstructed_lanechange_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();

    }

    void UnobstructedLaneChangePlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::setSpinRate(10.0);
        ros::CARMANodeHandle::spin();

    }

    bool UnobstructedLaneChangePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){

        cav_msgs::Maneuver maneuver_msg = req.maneuver_plan.maneuvers[0];
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
        target_speed_= maneuver_msg.lane_change_maneuver.end_speed;

        auto points_and_target_speeds = maneuvers_to_points(req.maneuver_plan.maneuvers, current_downtrack, wm_);
        
        //Ignoring downsampling based on assumption than lane change will happen over short distance
        //auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds.size(), 8);

        cav_msgs::TrajectoryPlan trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        trajectory.trajectory_points = compose_trajectory_from_centerline(points_and_target_speeds, req.vehicle_state);

        trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, 2.2352);

        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_CHANGE);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;

    }

    std::vector<PointSpeedPair> UnobstructedLaneChangePlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
    double max_starting_downtrack, const carma_wm::WorldModelConstPtr& wm){
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        for(const auto& maneuver : maneuvers)
        {
            if(maneuver.type != cav_msgs::Maneuver::LANE_CHANGE)
            {
                throw std::invalid_argument("Unobstructed Lane Change does  not support this maneuver type");
            }
            cav_msgs::LaneChangeManeuver lane_change_maneuver = maneuver.lane_change_maneuver;
            
            double starting_downtrack = lane_change_maneuver.start_dist;
            if(first){
                if(starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
                first = false;
            }
            // //Get lane change route
           lanelet::BasicLineString2d route_geometry = create_route_geom(starting_downtrack,stoi(lane_change_maneuver.starting_lane_id),lane_change_maneuver.end_dist, stoi(lane_change_maneuver.ending_lane_id), wm);

            first = true;
            double delta_v = (lane_change_maneuver.end_speed - start_speed_)/route_geometry.size();
            double prev_v = start_speed_;
            for(auto p :route_geometry)
            {
                if(first && points_and_target_speeds.size() !=0){
                    first = false;
                    continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                pair.speed = prev_v+ delta_v;
                points_and_target_speeds.push_back(pair);
                prev_v = pair.speed;
                //std::cout<<"Point "<<" x:"<< pair.point.x()<<" y:"<<pair.point.y()<<" Speed:"<< pair.speed<<std::endl;

            }
        }

        return points_and_target_speeds;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> UnobstructedLaneChangePlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {
        int nearest_pt_index = getNearestPointIndex(points, state);

        ROS_DEBUG_STREAM("NearestPtIndex: " << nearest_pt_index);
        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position

        auto time_bound_points = constrain_to_time_boundary(future_points, 6.0);

        std::vector<double> final_yaw_values;
        std::vector<double> final_actual_speeds;
        std::vector<lanelet::BasicPoint2d> all_sampling_points;

        std::vector<double> speed_limits;
        std::vector<lanelet::BasicPoint2d> curve_points;
        splitPointSpeedPairs(time_bound_points, &curve_points, &speed_limits);
        
        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(curve_points); // Compute splines based on curve points
        if (!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }


        std::vector<double> distributed_speed_limits;
        distributed_speed_limits.reserve(curve_points.size());

        // compute total length of the trajectory to get correct number of points 
        // we expect using curve_resample_step_size
        std::vector<double> downtracks_raw = carma_wm::geometry::compute_arc_lengths(curve_points);

        int total_step_along_curve= downtracks_raw.size();
        int current_speed_index = 0;
        size_t total_point_size = curve_points.size();

        double step_threshold_for_next_speed = (double)total_step_along_curve / (double)total_point_size;
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory

        //Geometry already fit to spline for lane change
        for (size_t steps_along_curve = 0; steps_along_curve < total_step_along_curve; steps_along_curve++) // Resample curve at tighter resolution
        {
            
            if ((double)steps_along_curve > step_threshold_for_next_speed)
            {
            step_threshold_for_next_speed += (double)total_step_along_curve / (double) total_point_size;
            current_speed_index ++;
            }
            distributed_speed_limits.push_back(speed_limits[current_speed_index]); // Identify speed limits for resampled points
            
        }


        std::vector<double> yaw_values = carma_wm::geometry::compute_tangent_orientations(curve_points);

        std::vector<double> curvatures = carma_wm::geometry::local_circular_arc_curvatures(
            curve_points, 1);  

        curvatures = smoothing::moving_average_filter(curvatures, 5);


        std::vector<double> ideal_speeds =
            trajectory_utils::constrained_speeds_for_curvatures(curvatures, 1.5);

        std::vector<double> actual_speeds = apply_speed_limits(ideal_speeds, distributed_speed_limits);

        // Drop last point
        final_yaw_values.insert(final_yaw_values.end(), yaw_values.begin(), yaw_values.end());
        all_sampling_points.insert(all_sampling_points.end(), curve_points.begin(), curve_points.end() );
        final_actual_speeds.insert(final_actual_speeds.end(), actual_speeds.begin(), actual_speeds.end());

        if (all_sampling_points.size() == 0)
        {
            ROS_WARN_STREAM("No trajectory points could be generated");
            return {};
        }

        // Find Lookahead Distance based on Velocity
        double lookahead_distance = get_adaptive_lookahead(state.longitudinal_vel);


        // Apply lookahead speeds
        final_actual_speeds = get_lookahead_speed(all_sampling_points, final_actual_speeds, lookahead_distance);
        
        // Add current vehicle point to front of the trajectory
        lanelet::BasicPoint2d cur_veh_point(state.X_pos_global, state.Y_pos_global);

        all_sampling_points.insert(all_sampling_points.begin(),
                                    cur_veh_point);  // Add current vehicle position to front of sample points

        final_actual_speeds.insert(final_actual_speeds.begin(), std::max(state.longitudinal_vel, 2.2352));

        final_yaw_values.insert(final_yaw_values.begin(), state.orientation);

        
        // Compute points to local downtracks
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(all_sampling_points);

        
        // Apply accel limits
        final_actual_speeds = trajectory_utils::apply_accel_limits_by_distance(downtracks, final_actual_speeds,
                                                                                1.5, 1.5);

        
        final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, 5);

        for (auto& s : final_actual_speeds)  // Limit minimum speed. TODO how to handle stopping?
        {
            s = std::max(s, 2.2352);
        }

        // Convert speeds to times
        std::vector<double> times;
        trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);

        
        // Build trajectory points
        // TODO When more plugins are implemented that might share trajectory planning the start time will need to be based
        // off the last point in the plan if an earlier plan was provided
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
            trajectory_from_points_times_orientations(all_sampling_points, times, final_yaw_values, ros::Time::now());

        return traj_points;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> UnobstructedLaneChangePlugin::trajectory_from_points_times_orientations(
    const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& times, const std::vector<double>& yaws,
    ros::Time startTime)
    {
        if (points.size() != times.size() || points.size() != yaws.size())
        {
            throw std::invalid_argument("All input vectors must have the same size");
        }

        std::vector<cav_msgs::TrajectoryPlanPoint> traj;
        traj.reserve(points.size());

        for (int i = 0; i < points.size(); i++)
        {
            cav_msgs::TrajectoryPlanPoint tpp;
            ros::Duration relative_time(times[i]);
            tpp.target_time = startTime + relative_time;
            tpp.x = points[i].x();
            tpp.y = points[i].y();
            tpp.yaw = yaws[i];

            tpp.controller_plugin_name = "default";
            tpp.planner_plugin_name = plugin_discovery_msg_.name;

            traj.push_back(tpp);
        }

        return traj;
    }

    double UnobstructedLaneChangePlugin::get_adaptive_lookahead(double velocity)
    {
        // lookahead:
        // v<10kph:  5m
        // 10kph<v<50kph:  0.5*v
        // v>50kph:  25m

        double lookahead = 5.0;

        if (velocity < 2.8)
        {
            lookahead = 5.0;
        } 
        else if (velocity >= 2.8 && velocity < 13.9)
        {
            lookahead = 2.0 * velocity;
        } 
        else lookahead = 25.0;

        return lookahead;

    }

    std::vector<double> UnobstructedLaneChangePlugin::get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead)
    {
  
        if (lookahead < 5.0)
        {
            throw std::invalid_argument("Invalid lookahead value");
        }

        if (speeds.size() < 1)
        {
            throw std::invalid_argument("Invalid speeds vector");
        }

        if (speeds.size() != points.size())
        {
            throw std::invalid_argument("Speeds and Points lists not same size");
        }

        std::vector<double> out;
        out.reserve(speeds.size());

        for (int i = 0; i < points.size(); i++)
        {
            int idx = i;
            double min_dist = std::numeric_limits<double>::max();
            for (int j=i+1; j < points.size(); j++){
            double dist = lanelet::geometry::distance2d(points[i],points[j]);
            if (abs(lookahead - dist) <= min_dist){
                idx = j;
                min_dist = abs(lookahead - dist);
            }
            }
            out.push_back(speeds[idx]);
        }
        
        return out;
        }

    std::vector<double> UnobstructedLaneChangePlugin::apply_speed_limits(const std::vector<double> speeds,
                                                             const std::vector<double> speed_limits)
    {
        if (speeds.size() != speed_limits.size())
        {
            throw std::invalid_argument("Speeds and speed limit lists not same size");
        }
        std::vector<double> out;
        for (size_t i = 0; i < speeds.size(); i++)
        {
            out.push_back(std::min(speeds[i], speed_limits[i]));
        }

        return out;
    }

    std::unique_ptr<smoothing::SplineI>
    UnobstructedLaneChangePlugin::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
    {
        if (basic_points.size() < 3)
        {
            ROS_WARN_STREAM("Insufficient Spline Points");
            return nullptr;
        }
        std::unique_ptr<smoothing::SplineI> spl = std::make_unique<smoothing::BSpline>();
        spl->setPoints(basic_points);
        return spl;
    }

    std::vector<DiscreteCurve> UnobstructedLaneChangePlugin::compute_sub_curves(const std::vector<PointSpeedPair>& map_points)
    {
        if (map_points.size() < 2)
        {
            throw std::invalid_argument("Not enough points");
        }

        std::vector<DiscreteCurve> curves;
        DiscreteCurve curve;
        curve.frame = compute_heading_frame(map_points[0].point, map_points[1].point);
        Eigen::Isometry2d map_in_curve = curve.frame.inverse();

        for (size_t i = 0; i < map_points.size() - 1; i++)
        {
            lanelet::BasicPoint2d p1 = map_in_curve * map_points[i].point;
            lanelet::BasicPoint2d p2 = map_in_curve * map_points[i + 1].point;  // TODO Optimization to cache this value

            PointSpeedPair initial_pair;
            initial_pair.point = p1;
            initial_pair.speed = map_points[i].speed;
            curve.points.push_back(initial_pair);

            bool x_dir = (p2.x() - p1.x()) > 0;
            if (!x_dir)  // If x starts going backwards we need a new curve
            {
            // New Curve
            curves.push_back(curve);

            curve = DiscreteCurve();
            curve.frame = compute_heading_frame(map_points[i].point, map_points[i + 1].point);
            map_in_curve = curve.frame.inverse();

            PointSpeedPair pair;
            pair.point = map_in_curve * map_points[i].point;
            pair.speed = map_points[i].speed;

            curve.points.push_back(pair);  // Include first point in curve
                }
            }

        curves.push_back(curve);

        return curves;
    }

    Eigen::Isometry2d UnobstructedLaneChangePlugin::compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
    {
    Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

    return carma_wm::geometry::build2dEigenTransform(p1, yaw);
    }

    std::vector<PointSpeedPair> UnobstructedLaneChangePlugin::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,double time_span)
    {
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        splitPointSpeedPairs(points, &basic_points, &speeds);

        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

        size_t time_boundary_exclusive_index =
        trajectory_utils::time_boundary_index(downtracks, speeds, 6.0);

        if (time_boundary_exclusive_index == 0)
        {
            throw std::invalid_argument("No points to fit in timespan"); 
        }

        std::vector<PointSpeedPair> time_bound_points;
        time_bound_points.reserve(time_boundary_exclusive_index);

        if (time_boundary_exclusive_index == points.size())
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.end());  // All points fit within time boundary
        }
        else
        {
            time_bound_points.insert(time_bound_points.end(), points.begin(),
                                    points.begin() + time_boundary_exclusive_index - 1);  // Limit points by time boundary
        }

        return time_bound_points;

    }
    void UnobstructedLaneChangePlugin::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
                                                std::vector<lanelet::BasicPoint2d>* basic_points,
                                                std::vector<double>* speeds)
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

    int UnobstructedLaneChangePlugin::getNearestPointIndex(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        ROS_DEBUG_STREAM("veh_point: " << veh_point.x() << ", " << veh_point.y());
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            ROS_DEBUG_STREAM("distance: " << distance);
            ROS_DEBUG_STREAM("p: " << p.point.x() << ", " << p.point.y());
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;
    }

    lanelet::BasicLineString2d UnobstructedLaneChangePlugin::create_route_geom(double starting_downtrack, int start_lane_id, double ending_downtrack, int end_lane_id, const carma_wm::WorldModelConstPtr& wm)
    {
        //Get geometry for maneuver
        if (starting_downtrack >= ending_downtrack)
        {
            throw std::invalid_argument("Start distance is greater than or equal to end distance");
        }

        auto shortest_path = wm_->getRoute()->shortestPath();

        //Use start and end lanelet index on maneuver-route to get lanelets in path
        int start_lanelet_index = findLaneletIndexFromPath(start_lane_id,shortest_path);
        int end_lanelet_index = findLaneletIndexFromPath(end_lane_id, shortest_path);
    
        std::vector<lanelet::ConstLanelet> lanelets_in_path(shortest_path.begin() + start_lanelet_index, shortest_path.begin() + end_lanelet_index);

    
        //Concatenate centerline path for forward lanelets, create spline for lane change=
        lanelet::Id prev_lanelet_id= lanelets_in_path[0].id();

        //find index on path
        int index = findLaneletIndexFromPath(prev_lanelet_id, shortest_path);
        lanelet::Id following_lanelet = wm_->getRoute()->followingRelations(shortest_path[index])[0].lanelet.id();
        lanelet::BasicLineString2d centerline_points = wm_->getRoute()->followingRelations(shortest_path[index])[0].lanelet.centerline2d().basicLineString();
        lanelet::BasicLineString2d new_points;


        for(size_t i=1;i<lanelets_in_path.size();i++){
            index = findLaneletIndexFromPath(lanelets_in_path[i].id(), shortest_path);
            if(lanelets_in_path[i].id() !=following_lanelet){
                //CHANGING LANES
                //generate new points
                lanelet::BasicLineString2d out;
                lanelet::BasicLineString2d  a = wm_->getRoute()->followingRelations(shortest_path[index])[0].lanelet.centerline2d().basicLineString();
                lanelet::BasicPoint2d start = centerline_points.front();
                lanelet::BasicPoint2d end = a.back();
                //Use Spline lib to create lane change route
                new_points = create_lanechange_route(start, end);
            }
            else{
                //GOING STRAIGHT
                new_points = wm_->getRoute()->followingRelations(shortest_path[index])[0].lanelet.centerline2d().basicLineString();
                //concatenate
                //carma_wm::geometry::concatenate_line_strings(centerline_points, new_points);

            }

            //Start from .begin() for first iteration to account for relation between lanelet 0 and lanelet 1
            if(i == 1){     
                centerline_points = new_points;
            }
            else{
                centerline_points.insert(centerline_points.end(), new_points.begin(), new_points.end());
            }

            prev_lanelet_id = lanelets_in_path[i].id();
            index = findLaneletIndexFromPath(prev_lanelet_id, shortest_path);
            following_lanelet = wm_->getRoute()->followingRelations(shortest_path[index])[0].lanelet.id();
        }

        return centerline_points;
        
    }

    lanelet::BasicLineString2d  UnobstructedLaneChangePlugin::create_lanechange_route(lanelet::BasicPoint2d start, lanelet::BasicPoint2d end)
    {    
        lanelet::BasicLineString2d lc_route;

        lanelet::BasicPoint2d mid;
        mid.x() = (start.x() + end.x())/2;
        mid.y() = (start.y() + end.y())/2;
        std::vector<lanelet::BasicPoint2d> points = {start, mid, end};

        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(points);
        if(!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }

        //lc_route.push_back(start);
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
        for(int i =0;i<num_points;i++){
            lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
            lc_route.push_back(p);
            scaled_steps_along_curve += 1.0/num_points;
        }
        lc_route.push_back(end);


        return lc_route;

    }

    bool UnobstructedLaneChangePlugin::identifyLaneChange(lanelet::routing::LaneletRelations relations, int target_id)
    {        
        for(auto& relation : relations)
        {
            if(relation.lanelet.id() == target_id && relation.relationType == lanelet::routing::RelationType::Left || relation.relationType == lanelet::routing::RelationType::Right )
            {
                return true;
            }
        }
        return false;

    }

    int UnobstructedLaneChangePlugin::findLaneletIndexFromPath(int target_id, lanelet::routing::LaneletPath& path)
    {
        for(size_t i = 0; i < path.size(); ++i)
        {
            if(path[i].id() == target_id)
            {
                return i;
            }
        }
        return -1;
    }
}