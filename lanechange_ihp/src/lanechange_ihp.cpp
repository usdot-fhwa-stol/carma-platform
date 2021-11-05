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
#include "lanechange_ihp.h"
#include <unordered_set>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.h>
#include <trajectory_utils/conversions/conversions.h>
#include <carma_utils/containers/containers.h>
#include "smoothing/filters.h"
#include<limits>
#include<cmath>

namespace lanechange_ihp
{                   
    int i_count=0;
    void LaneChangeIHPPlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        
        trajectory_srv_ = nh_->advertiseService("plugins/LaneChangeIHPPlugin/plan_trajectory", &LaneChangeIHPPlugin::plan_trajectory_cb, this);
        yield_client_ = nh_->serviceClient<cav_srvs::PlanTrajectory>("plugins/YieldPlugin/plan_trajectory");

        rerout_client_ = nh_->serviceClient<cav_srvs::ReRoute>("reroute_service");
                
        lanechange_ihp_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "LaneChangeIHPPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose", 1, &LaneChangeIHPPlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &LaneChangeIHPPlugin::twist_cd, this);

        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<std::string>("control_plugin_name", control_plugin_name_, "NULL");
        pnh_->param<double>("minimum_speed", minimum_speed_, 2.2352);
        pnh_->param<double>("max_accel", max_accel_, 1.5);
        pnh_->param<double>("minimum_lookahead_distance", minimum_lookahead_distance_, 5.0);
        pnh_->param<double>("maximum_lookahead_distance", maximum_lookahead_distance_, 25.0);
        pnh_->param<double>("minimum_lookahead_speed", minimum_lookahead_speed_, 2.8);
        pnh_->param<double>("maximum_lookahead_speed", maximum_lookahead_speed_, 13.9);
        pnh_->param<double>("lateral_accel_limit", lateral_accel_limit_, 1.5);
        pnh_->param<double>("speed_moving_average_window_size", speed_moving_average_window_size_, 5);
        pnh_->param<double>("curvature_moving_average_window_size", curvature_moving_average_window_size_, 9);
        pnh_->param<double>("curvature_calc_lookahead_count", curvature_calc_lookahead_count_, 1);
        pnh_->param<int>("downsample_ratio", downsample_ratio_, 8);
        pnh_->param<bool>("enable_object_avoidance_lc", enable_object_avoidance_lc_, false);
        pnh_->param<double>("acceptable_time_difference_", acceptable_time_difference_, 1.0);
        pnh_->param<double>("min_timestep",min_timestep_);
        // update ros_dur_ value
        time_dur_ = ros::Duration(acceptable_time_difference_);

        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { lanechange_ihp_plugin_discovery_pub_.publish(plugin_discovery_msg_); });

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();

    }

    void LaneChangeIHPPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());

        i_count++;

        if (i_count>3000)
        {
            i_count=10;
        }

        cav_srvs::ReRoute service_message;

        if (i_count==1999)
        {
            std::cout<<"start to call xin's service 1"<<std::endl;
            service_message.request.choice=1;
            rerout_client_.call(service_message);
        }else if (i_count==1000)
        {
            std::cout<<"start to call xin's service 0"<<std::endl;
            service_message.request.choice=0;
            rerout_client_.call(service_message);
        }

    }
    void LaneChangeIHPPlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }

    void LaneChangeIHPPlugin::run()
    {
    	initialize();
        ros::CARMANodeHandle::spin();

    }

    bool LaneChangeIHPPlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        //PlanTrajectoryRequest 里面有五个内容： 
        //                std_msgs/Header header; 
        //                cav_msgs/VehicleState vehicle_state; 
        //                cav_msgs/ManeuverPlan maneuver_plan;
        //                cav_msgs/TrajectoryPlan initial_trajectory_plan;
        //                uint16 maneuver_index_to_plan
        //PlanTrajectoryResponse 里面有三个内容： 
        //                cav_msgs/TrajectoryPlan trajectory_plan; 
        //                uint8[] related_maneuvers;
        //                uint8[] maneuver_status

        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global, req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        // Only plan the trajectory for the requested LANE_CHANGE maneuver
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        if(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan].type != cav_msgs::Maneuver::LANE_CHANGE)
        {
            throw std::invalid_argument ("IHP Lane Change Plugin doesn't support this maneuver type");
        }
        maneuver_plan.push_back(req.maneuver_plan.maneuvers[req.maneuver_index_to_plan]);
        resp.related_maneuvers.push_back(req.maneuver_index_to_plan);

        auto points_and_target_speeds = maneuvers_to_points(maneuver_plan, current_downtrack, wm_,req.vehicle_state);
        
        auto downsampled_points = carma_utils::containers::downsample_vector(points_and_target_speeds, downsample_ratio_);

        cav_msgs::TrajectoryPlan original_trajectory;
        original_trajectory.header.frame_id = "map";
        original_trajectory.header.stamp = ros::Time::now();
        original_trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        int starting_lanelet_id = stoi(maneuver_plan.front().lane_change_maneuver.starting_lane_id);
        double final_max_speed = maneuver_plan.front().lane_change_maneuver.end_speed;
        original_trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points, req.vehicle_state, req.header.stamp, starting_lanelet_id, final_max_speed);

        original_trajectory.initial_longitudinal_velocity = std::max(req.vehicle_state.longitudinal_vel, minimum_speed_);
        resp.trajectory_plan = original_trajectory;
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;
    }

    std::vector<PointSpeedPair> LaneChangeIHPPlugin::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
    double max_starting_downtrack, const carma_wm::WorldModelConstPtr& wm,const cav_msgs::VehicleState& state){
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        for(const auto& maneuver : maneuvers)
        {
            if(maneuver.type != cav_msgs::Maneuver::LANE_CHANGE)
            {
                throw std::invalid_argument("IHP Lane Change does  not support this maneuver type");
            }
            cav_msgs::LaneChangeManeuver lane_change_maneuver = maneuver.lane_change_maneuver;
            
            double starting_downtrack = lane_change_maneuver.start_dist;
            if(first){
                //如果当前maneuver plan里面的开始位置已经超过当前车辆当前位置，那么将maneuver plan里面的当前位置修改为车辆当前位置
                if(starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
                first = false;
            }
            // //Get lane change route
            double ending_downtrack = lane_change_maneuver.end_dist;
                //Get geometry for maneuver
            if (starting_downtrack >= ending_downtrack)
            {
                throw std::invalid_argument("Start distance is greater than or equal to end distance");
            }

            //get route geometry between starting and ending downtracks
            //利用lanelet 的中心线生成waypoints
            lanelet::BasicLineString2d route_geometry = create_route_geom(starting_downtrack, stoi(lane_change_maneuver.starting_lane_id), ending_downtrack,stoi(lane_change_maneuver.ending_lane_id), wm);

            int nearest_pt_index = getNearestRouteIndex(route_geometry,state);
            int ending_pt_index = get_ending_point_index(route_geometry, ending_downtrack);
            //只取那一段需要的轨迹点
            lanelet::BasicLineString2d future_route_geometry(route_geometry.begin() + nearest_pt_index, route_geometry.begin() + ending_pt_index);
            first = true;

            //将速度加到waypoints上
            for(auto p :future_route_geometry)
            {
                if(first && points_and_target_speeds.size() !=0){
                    first = false;
                    continue; // Skip the first point if we have already added points from a previous maneuver to avoid duplicates
                }
                PointSpeedPair pair;
                pair.point = p;
                if(state.longitudinal_vel  < minimum_speed_){
                    pair.speed = lane_change_maneuver.end_speed;
                }
                else{
                    pair.speed = state.longitudinal_vel;
                }
                points_and_target_speeds.push_back(pair);

            }
        }

        return points_and_target_speeds;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> LaneChangeIHPPlugin::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state, const ros::Time& state_time, int starting_lanelet_id, double max_speed)
    {
        ROS_DEBUG_STREAM("Input points size in: compose_trajectory_from_centerline" << points.size());
        int nearest_pt_index = getNearestPointIndex(points, state);
        ROS_DEBUG_STREAM("nearest_pt_index: " << nearest_pt_index);


        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position
        ROS_DEBUG_STREAM("future_points points size in: " << future_points.size());
        
        //Compute yaw values from original trajectory
        std::vector<lanelet::BasicPoint2d> future_geom_points;
        std::vector<double> final_actual_speeds;
        splitPointSpeedPairs(future_points, &future_geom_points, &final_actual_speeds);
        std::vector<double> final_yaw_values = carma_wm::geometry::compute_tangent_orientations(future_geom_points);

    
        // Compute points to local downtracks
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(future_geom_points);//根据两点距离计算累计的downtrack距离
        final_actual_speeds = smoothing::moving_average_filter(final_actual_speeds, speed_moving_average_window_size_);

        // Convert speeds to times
        std::vector<double> times;
        trajectory_utils::conversions::speed_to_time(downtracks, final_actual_speeds, &times);//利用平均速度模型根据速度计算每个downtrack对应的时间

        // Build trajectory points
        std::vector<cav_msgs::TrajectoryPlanPoint> traj_points =
            trajectory_from_points_times_orientations(future_geom_points, times, final_yaw_values, state_time);//将waypoints, 时间，以及yaw 的值转化为标准的std::vector<cav_msgs::TrajectoryPlanPoint>
        
        ROS_DEBUG_STREAM("PRINTING FINAL SPEEDS");
        for (auto speed : final_actual_speeds)
        {
            ROS_DEBUG_STREAM("Final Speed: " << speed);
        }
        //std::vector<cav_msgs::TrajectoryPlanPoint> traj;
        return traj_points;

    }

    std::vector<cav_msgs::TrajectoryPlanPoint> LaneChangeIHPPlugin::trajectory_from_points_times_orientations(
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

    double LaneChangeIHPPlugin::compute_curvature_at(const lanechange_ihp::smoothing::SplineI& fit_curve, double step_along_the_curve) const
    {
        lanelet::BasicPoint2d f_prime_pt = fit_curve.first_deriv(step_along_the_curve);
        lanelet::BasicPoint2d f_prime_prime_pt = fit_curve.second_deriv(step_along_the_curve);
        // Convert to 3d vector to do 3d vector operations like cross.
        Eigen::Vector3d f_prime = {f_prime_pt.x(), f_prime_pt.y(), 0};
        Eigen::Vector3d f_prime_prime = {f_prime_prime_pt.x(), f_prime_prime_pt.y(), 0};
        return (f_prime.cross(f_prime_prime)).norm()/(pow(f_prime.norm(),3));
    }

    double LaneChangeIHPPlugin::get_adaptive_lookahead(double velocity)
    {
        // lookahead:
        // v<10kph:  5m
        // 10kph<v<50kph:  0.5*v
        // v>50kph:  25m

        double lookahead = minimum_lookahead_distance_;

        if (velocity < minimum_lookahead_speed_)
        {
            lookahead = minimum_lookahead_distance_;
        } 
        else if (velocity >= minimum_lookahead_speed_ && velocity < maximum_lookahead_speed_)
        {
            lookahead = 2.0 * velocity;
        } 
        else lookahead = maximum_lookahead_distance_;

        return lookahead;

    }

    std::vector<double> LaneChangeIHPPlugin::get_lookahead_speed(const std::vector<lanelet::BasicPoint2d>& points, const std::vector<double>& speeds, const double& lookahead)
    {
  
        if (lookahead < minimum_lookahead_distance_)
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

    std::vector<double> LaneChangeIHPPlugin::apply_speed_limits(const std::vector<double> speeds,
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
    LaneChangeIHPPlugin::compute_fit(const std::vector<lanelet::BasicPoint2d>& basic_points)
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


    Eigen::Isometry2d LaneChangeIHPPlugin::compute_heading_frame(const lanelet::BasicPoint2d& p1,
                                                              const lanelet::BasicPoint2d& p2)
    {
    Eigen::Rotation2Dd yaw(atan2(p2.y() - p1.y(), p2.x() - p1.x()));

    return carma_wm::geometry::build2dEigenTransform(p1, yaw);
    }

    std::vector<PointSpeedPair> LaneChangeIHPPlugin::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,double time_span)
    {
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        splitPointSpeedPairs(points, &basic_points, &speeds);

        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

        size_t time_boundary_exclusive_index =
        trajectory_utils::time_boundary_index(downtracks, speeds, trajectory_time_length_);

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
    void LaneChangeIHPPlugin::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points,
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

    int LaneChangeIHPPlugin::getNearestRouteIndex(lanelet::BasicLineString2d& points, const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
                double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p,veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }
        return best_index;

    }
    int LaneChangeIHPPlugin::get_ending_point_index(lanelet::BasicLineString2d& points, double ending_downtrack){
        int best_index = points.size()-1;
        for(int i=points.size()-1;i>=0;i--){
            double downtrack = wm_->routeTrackPos(points[i]).downtrack;
            if(downtrack <= ending_downtrack){
                best_index = i;
                break;
            }
        }
        return best_index;
    }

    int LaneChangeIHPPlugin::getNearestPointIndex(const std::vector<PointSpeedPair>& points,
                                               const cav_msgs::VehicleState& state)
    {
        lanelet::BasicPoint2d veh_point(state.X_pos_global, state.Y_pos_global);
        double min_distance = std::numeric_limits<double>::max();
        int i = 0;
        int best_index = 0;
        for (const auto& p : points)
        {
            double distance = lanelet::geometry::distance2d(p.point, veh_point);
            if (distance < min_distance)
            {
            best_index = i;
            min_distance = distance;
            }
            i++;
        }

        return best_index;
    }

    lanelet::BasicLineString2d  LaneChangeIHPPlugin::create_lanechange_path(PointIndexPair start, lanelet::ConstLanelet& start_lanelet, PointIndexPair end, lanelet::ConstLanelet& end_lanelet)
    {
        std::vector<lanelet::BasicPoint2d> centerline_points={};
        lanelet::BasicLineString2d centerline_start_lane = start_lanelet.centerline2d().basicLineString();
        lanelet::BasicLineString2d centerline_end_lane = end_lanelet.centerline2d().basicLineString();


        lanelet::BasicPoint2d startlanelet_end=centerline_start_lane.back();
        lanelet::BasicPoint2d endlanelet_begin=centerline_end_lane.front();

        double startlanelet_end_downtrack = wm_->routeTrackPos(startlanelet_end).downtrack;
        double endlanelet_begin_downtrack = wm_->routeTrackPos(endlanelet_begin).downtrack;
        double endlanelet_end_downtrack = wm_->routeTrackPos(centerline_end_lane.back()).downtrack;

        double start_point_index=start.index;
        double end_point_index=end.index;
        lanelet::BasicPoint2d start_lane_pt  = start.point;
        lanelet::BasicPoint2d end_lane_pt = end.point;


        int total_points = end_point_index-start_point_index;
        if (abs(startlanelet_end_downtrack-endlanelet_end_downtrack)<1)
        {
            double dist = sqrt(pow((end_lane_pt.x() - start_lane_pt.x()),2) + pow((end_lane_pt.y() - start_lane_pt.y()),2));
            
            if (total_points>=1)
            {
                double delta_step = 1.0/total_points;
                for(int i=1;i<total_points;i++){
                    lanelet::BasicPoint2d current_position;
                    start_lane_pt = centerline_start_lane[start_point_index+i];
                    end_lane_pt = centerline_end_lane[start_point_index+i];
                    double delta = delta_step*i;
                    current_position.x() = end_lane_pt.x()*delta + (1-delta)* start_lane_pt.x();
                    current_position.y() = end_lane_pt.y()*delta + (1-delta)* start_lane_pt.y();

                    centerline_points.push_back(current_position);
                }
            }else{

                throw std::invalid_argument("Not enough points to plan"); 
            }
        }

        std::unique_ptr<smoothing::SplineI> fit_curve = compute_fit(centerline_points);
        if(!fit_curve)
        {
            throw std::invalid_argument("Could not fit a spline curve along the given trajectory!");
        }

        lanelet::BasicLineString2d lc_route;
        //lc_route.push_back(start);
        double scaled_steps_along_curve = 0.0; // from 0 (start) to 1 (end) for the whole trajectory
        for(int i =0;i<centerline_points.size();i++){
            lanelet::BasicPoint2d p = (*fit_curve)(scaled_steps_along_curve);
            lc_route.push_back(p);
            scaled_steps_along_curve += 1.0/total_points;
        }
        lc_route.push_back(end.point);

        return lc_route;
    }


    PointIndexPair LaneChangeIHPPlugin::find_closest_point_index_pair(lanelet::ConstLanelet lanelet_input,double dowtrack)
    {
        lanelet::BasicLineString2d points =lanelet_input.centerline2d().basicLineString();
        std::vector<double> downtrack_dis_lanelet;
        for (const auto& p : points)
        {
            // std::cout<<"p in lanelets x =  "<<p.x()<<", p in lanelets y =  "<<p.y()<<std::endl;
            double downtrack_p = wm_->routeTrackPos(p).downtrack;
            downtrack_dis_lanelet.push_back(abs(downtrack_p-dowtrack));
        }
        int minPosition = min_element(downtrack_dis_lanelet.begin(),downtrack_dis_lanelet.end()) - downtrack_dis_lanelet.begin();
        lanelet::BasicPoint2d closest_point = points[minPosition];

        PointIndexPair pair;
        pair.point=closest_point;
        pair.index=minPosition;
        return pair;
    }

    lanelet::BasicLineString2d LaneChangeIHPPlugin::create_route_geom( double starting_downtrack, int starting_lane_id, double ending_downtrack, int ending_lane_id, const carma_wm::WorldModelConstPtr& wm)
    {
        //Create route geometry for lane change maneuver
        //Starting downtrack maybe in the previous lanelet, if it is, have a lane follow till new lanelet starts
        std::vector<lanelet::ConstLanelet> lanelets_in_route = wm->getLaneletsBetween(starting_downtrack, ending_downtrack, true, false);

        lanelet::BasicLineString2d centerline_points = {};

        lanelet::ConstLanelet start_lanelet,end_lanelet;
        for(int i=0;i<lanelets_in_route.size();i++){
            if(lanelets_in_route[i].id() == starting_lane_id){
                start_lanelet=lanelets_in_route[i];
            }else if (lanelets_in_route[i].id() == ending_lane_id) 
            {
                end_lanelet=lanelets_in_route[i];
            }
        }

        PointIndexPair start_pair=find_closest_point_index_pair(start_lanelet,starting_downtrack);
        PointIndexPair end_pair=find_closest_point_index_pair(end_lanelet,ending_downtrack);


        lanelet::BasicLineString2d new_points = create_lanechange_path(start_pair,start_lanelet, end_pair, end_lanelet);
        centerline_points.insert(centerline_points.end(),new_points.begin(),new_points.end() );
    
        return centerline_points;
        
    }

    bool LaneChangeIHPPlugin::validate_yield_plan(const cav_msgs::TrajectoryPlan& yield_plan, const std::string& original_plan_id)
    {
        if (yield_plan.trajectory_points.size()>= 2 && yield_plan.trajectory_id == original_plan_id)
        {
            ros::Duration time_difference = yield_plan.trajectory_points[0].target_time - ros::Time::now();
            
            if (time_difference <= time_dur_)
            {
                return true;
            }
            else
            {
                ROS_DEBUG_STREAM("Old Yield Trajectory");
            }
        }
        else
        {
            ROS_DEBUG_STREAM("Invalid Yield Trajectory"); 
        }
        return false;
    }

}