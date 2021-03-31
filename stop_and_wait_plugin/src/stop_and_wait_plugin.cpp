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
#include "stop_and_wait_plugin.h"
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



using oss = std::ostringstream;

namespace stop_and_wait_plugin
{
    void StopandWait::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh2_.reset(new ros::CARMANodeHandle("/"));

        trajectory_srv_ = nh_->advertiseService("plan_trajectory",&StopandWait::plan_trajectory_cb, this);
        
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery",1);
        jerk_pub_ = nh_->advertise<std_msgs::Float64>("jerk",1);
        plugin_discovery_msg_.name = "StopandWaitPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
        
        pose_sub_ = nh_->subscribe("current_pose",1, &StopandWait::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &StopandWait::twist_cb, this);

        wml_.reset(new carma_wm::WMListener());
        wm_ = wml_->getWorldModel();
        
        pnh_->param<double>("minimal_trajectory_duration", minimal_trajectory_duration_);
        pnh_->param<double>("max_jerk_limit", max_jerk_limit_);
        pnh_->param<double>("min_timestep",min_timestep_);
        pnh_->param<double>("min_jerk", min_jerk_limit_);
        pnh_->param<double>("/guidance/destination_downtrack_range",destination_downtrack_range_);

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool
        {
            plugin_discovery_pub_.publish(plugin_discovery_msg_);
            std_msgs::Float64 jerk_msg;
            jerk_msg.data = jerk_;
            jerk_pub_.publish(jerk_msg);
            return true;
        });
    }

    void StopandWait::run()
    {
        initialize();
        double spin_rate = pnh_->param<double>("spin_rate_hz",10.0);
        ros::CARMANodeHandle::setSpinRate(spin_rate);
        ros::CARMANodeHandle::spin();
    }

    void StopandWait::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
    }
    
    void StopandWait::twist_cb(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
    }
    
    bool StopandWait::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp)
    {
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global,req.vehicle_state.Y_pos_global);
        ROS_DEBUG_STREAM("curr state x:"<< pose_msg_.pose.position.x << ", y: " << pose_msg_.pose.position.y);
        ROS_DEBUG_STREAM("planning state x:"<<  req.vehicle_state.X_pos_global << ", y: " << req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;
        ROS_DEBUG_STREAM("Starting stop&wait planning");
        ROS_DEBUG_STREAM("Current_downtrack"<<current_downtrack);
        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(const auto maneuver : req.maneuver_plan.maneuvers)
        {
            if(maneuver.type == cav_msgs::Maneuver::STOP_AND_WAIT)
            {
                maneuver_plan.push_back(maneuver);
            }
        }

        if(current_downtrack < maneuver_plan[0].stop_and_wait_maneuver.start_dist){
            //Do nothing
            return true;
        }

        auto curr_state = req.vehicle_state;

        std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points(maneuver_plan, current_downtrack, wm_, curr_state);

        auto downsampled_points = 
            carma_utils::containers::downsample_vector(points_and_target_speeds,downsample_ratio_);
        ROS_DEBUG_STREAM("downsampled points size:"<<downsampled_points.size());
        //Trajectory plan
        cav_msgs::TrajectoryPlan  trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
      
        trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points,curr_state);
        ROS_DEBUG_STREAM("Trajectory points size:"<<trajectory.trajectory_points.size());
        trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;
        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::STOP_AND_WAIT);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;
    }

    std::vector<PointSpeedPair> StopandWait::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm, const cav_msgs::VehicleState& state)
    {
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::unordered_set<lanelet::Id> visited_lanelets;

        bool first = true;
        for(const auto& maneuver : maneuvers)
        {
            if(maneuver.type != cav_msgs::Maneuver::STOP_AND_WAIT)
            {
                throw std::invalid_argument ("Stop and Wait Maneuver Plugin doesn't support this maneuver type");
            }
            cav_msgs::StopAndWaitManeuver stop_and_wait_maneuver= maneuver.stop_and_wait_maneuver;

            double starting_downtrack = stop_and_wait_maneuver.start_dist;  //starting downtrack recorded in message
            if(first)   //check for first maneuver in vector 
            {
                if(starting_downtrack > max_starting_downtrack)
                {
                    starting_downtrack = max_starting_downtrack;
                }
                first = false;

            }
           
            double ending_downtrack = stop_and_wait_maneuver.end_dist; 
            double start_speed = state.longitudinal_vel;    //Get static value of current speed at start of planning
            //maneuver_time_ = ros::Duration(stop_and_wait_maneuver.end_time - stop_and_wait_maneuver.start_time).toSec();
            
            maneuver_time_ = (3*(ending_downtrack - starting_downtrack))/(2*start_speed);

            double delta_time, curr_time;
            if(start_speed < epsilon_ )  //If at end_dist return zero speed trajectory
            {
                ///guidance/route/destination_downtrack_range
                auto shortest_path = wm_->getRoute()->shortestPath();

                delta_time = min_timestep_;
                curr_time = 0.0;
                //wait
                lanelet::BasicPoint2d curr_pose (state.X_pos_global,state.Y_pos_global);
                while(curr_time < minimal_trajectory_duration_)
                {
                    PointSpeedPair pair;
                    pair.point = curr_pose;
                    pair.speed = 0.0;
                    points_and_target_speeds.push_back(pair);
                    curr_time += delta_time;

                    points_and_target_speeds.push_back(pair);
                }
            }
            else
            {
                double jerk_req = (2*start_speed)/pow(maneuver_time_,2);

                if(jerk_req > max_jerk_limit_)
                {
                    //unsafe to stop at the required jerk - reset to max_jerk and go beyond the maneuver end_dist
                    jerk_ = max_jerk_limit_;  
                    double travel_dist_new = start_speed * maneuver_time_ - (0.167 * jerk_ * pow(maneuver_time_,3));
                    ending_downtrack = travel_dist_new + starting_downtrack;

                    auto shortest_path = wm_->getRoute()->shortestPath();
                    if(ending_downtrack > wm_->getRouteEndTrackPos().downtrack)
                    {
                        ROS_ERROR("Ending distance is beyond known route");
                        throw std::invalid_argument("Ending distance is beyond known route"); 
                    }
                }
                else {
                    //find distance to the end
                    lanelet::BasicPoint2d curr_pose (state.X_pos_global,state.Y_pos_global);
                    double current_downtrack = wm_->routeTrackPos(curr_pose).downtrack;
                    //stay approximately at crawl speed until within destination downtrack range (defined in route)
                    if(start_speed <= min_crawl_speed_ && current_downtrack < ending_downtrack - destination_downtrack_range_){
                        jerk_ = 0.0;
                    }
                    else{
                        jerk_ = jerk_req;
                    }
                }
                //get all the lanelets in between starting and ending downtrack on shortest path
                auto lanelets = wm_->getLaneletsBetween(starting_downtrack, ending_downtrack, true);
                //record all the lanelets to be added to path
                std::vector<lanelet::ConstLanelet> lanelets_to_add;
                for (auto& l : lanelets)
                {
                    if(visited_lanelets.find(l.id()) == visited_lanelets.end())
                    {
                        lanelets_to_add.push_back(l);
                        visited_lanelets.insert(l.id());
                    }
                }

                lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);
                int nearest_pt_index = getNearestRouteIndex(route_geometry,state);
                auto temp_state = state;
                temp_state.X_pos_global = wm_->getRoute()->getEndPoint().basicPoint2d().x();
                temp_state.Y_pos_global =  wm_->getRoute()->getEndPoint().basicPoint2d().y();
                int nearest_end_pt_index = getNearestRouteIndex(route_geometry,temp_state);
                lanelet::BasicLineString2d future_route_geometry(route_geometry.begin() + nearest_pt_index, route_geometry.begin()+ nearest_end_pt_index);
                
                int points_count = future_route_geometry.size();
                delta_time = maneuver_time_/(points_count-1);

                first = true;
                curr_time = 0.0;

                for(auto p : future_route_geometry)
                {
                    if (first && points_and_target_speeds.empty())
                    {
                        first = false;

                        continue; //Skip the first point to avoid duplicates from previous maneuver
                    }
                    PointSpeedPair pair;
                    pair.point = p;
                    pair.speed = start_speed - (0.5 * jerk_ * pow(curr_time,2));
                    if(pair.speed < 0.01){
                        pair.speed = 0.0;
                    }

                    if(p == future_route_geometry.back()) 
                    {
                        pair.speed = 0.0;    //force speed to 0 at last point
                    }
                    curr_time +=delta_time;

                    points_and_target_speeds.push_back(pair);
                }
                //Change timestep to 0.1s
                PointSpeedPair last_point = points_and_target_speeds.back();
                if(delta_time < min_timestep_){
                    int downsample_ratio = min_timestep_/delta_time;
                    points_and_target_speeds = carma_utils::containers::downsample_vector(points_and_target_speeds,downsample_ratio);
                }
                points_and_target_speeds.push_back(last_point);            }
            //If planned time is less than min trajectory duration add zero speed points
            while(curr_time < minimal_trajectory_duration_)
            {
                PointSpeedPair pair;
                pair.point = points_and_target_speeds.back().point;
                pair.speed = 0.0;
                curr_time += delta_time;
                points_and_target_speeds.push_back(pair);
            }
            
        }
        
        return points_and_target_speeds;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> StopandWait::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {
        int nearest_pt_index = getNearestPointIndex(points,state);
        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index, points.end()); // Points in front of current vehicle position
        //Get yaw - geometrically
        std::vector<double> yaw_values;
        for(size_t i=0 ;i < future_points.size()-1 ;i++)
        {
            double yaw = atan((future_points[i+1].point.y() - future_points[i].point.y())/ (future_points[i+1].point.x() - future_points[i].point.x()));
            yaw_values.push_back(yaw);
        }
        yaw_values.push_back(0.0); //No rotation from last point

        //get target time from speed
        std::vector<double> target_times;
        std::vector<lanelet::BasicPoint2d> trajectory_locations;
        std::vector<double> trajectory_speeds;
        //split point speed pair
        splitPointSpeedPairs(future_points,&trajectory_locations,&trajectory_speeds);
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(trajectory_locations);

        //get trajectory time from distance and speed
        speed_to_time(downtracks, trajectory_speeds,target_times, jerk_);
        std::vector <cav_msgs::TrajectoryPlanPoint> traj;
        ros::Time start_time = ros::Time::now();
        cav_msgs::TrajectoryPlanPoint traj_prev;
        traj_prev.x= future_points[0].point.x();
        traj_prev.y = future_points[0].point.y();
        traj_prev.yaw = yaw_values[0];
        traj_prev.target_time = start_time;

        for (size_t i=0; i < future_points.size(); i++)
        {
            cav_msgs::TrajectoryPlanPoint traj_point;
            if(trajectory_speeds[i] > 0.0){
                traj_point.x = future_points[i].point.x();
                traj_point.y = future_points[i].point.y();
                traj_point.yaw = yaw_values[i];
                traj_point.target_time = start_time + ros::Duration(target_times[i]);
            }
            else    //speed_to_time doesn't work for 0.0 speed
            {
                traj_point.x = future_points[i].point.x();
                traj_point.y = future_points[i].point.y();
                traj_point.yaw=traj_prev.yaw;
                traj_point.target_time = traj_prev.target_time + ros::Duration(min_timestep_);
                
            }
            traj_point.controller_plugin_name = "Pure Pursuit Jerk";
            traj_point.planner_plugin_name =plugin_discovery_msg_.name;
            traj.push_back(traj_point);
            traj_prev = traj_point;
        }
        //If trajectory only contains one 0.0 mph point, add another iteration(valid trajectory needs to have atleast 2 points)
        if(traj.size() == 1 && trajectory_speeds[0] == 0){
            cav_msgs::TrajectoryPlanPoint traj_point;
            traj_point = traj_prev;
            traj_point.target_time = traj_prev.target_time + ros::Duration(min_timestep_);
            traj.push_back(traj_point);
        }
        
        return traj;
    }

    void StopandWait::speed_to_time(const std::vector<double>& downtrack, const std::vector<double>& speeds,std::vector<double>& times, double jerk) const
    {
        if(downtrack.size() !=speeds.size())
        {
            throw std::invalid_argument("Input vector sizes do not match");
        }
        if (downtrack.empty())
        {
            throw std::invalid_argument("Input vectors are empty");
        }

        times.reserve(downtrack.size());  

        //Uses equation 
        //d_t = sqrt(2(d_v)/j)
        double prev_speed = speeds[0];
        double prev_time = 0.0;
        double prev_pos = downtrack[0];
        times.push_back(prev_time);
        for(int i=1; i <downtrack.size();i++)
        {
            double cur_speed = speeds[i];
            double delta_v = std::abs(cur_speed - prev_speed);
            double dt = sqrt(2*delta_v/jerk);
            double inst_acc = jerk * dt;
            if(jerk < min_jerk_limit_)    //Below minimum jerk slow down is ignored, treat as constant velocity
            {
                double cur_pos = downtrack[i];
                double delta_x = cur_pos - prev_pos;
                dt = delta_x/cur_speed;
            }
            double cur_time = dt + prev_time;
            times.push_back(cur_time);

            prev_speed = cur_speed;
            prev_time = cur_time;
            prev_pos = downtrack[i];
        }
    }


    int StopandWait::getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state) const
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


    int StopandWait::getNearestRouteIndex(lanelet::BasicLineString2d& points, const cav_msgs::VehicleState& state)
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

    void StopandWait::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
                        std::vector<double>* speeds) const
    {
        basic_points->reserve(points.size());
        speeds->reserve(points.size());

        for (const auto& p : points)
        {
            basic_points->push_back(p.point);
            speeds->push_back(p.speed);
        }
    }

}