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



using oss = std::ostringstream;

namespace stop_and_wait_plugin
{
    void StopandWait::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        trajectory_srv_ = nh_->advertiseService("plugins/StopandWaitPlugin/plan_trajectory",&StopandWait::plan_trajectory_cb, this);
        
        plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery",1);
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

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool
        {
            plugin_discovery_pub_.publish(plugin_discovery_msg_);
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

    StopandWait::StopandWait(carma_wm::WorldModelConstPtr wm, PublishPluginDiscoveryCB plugin_discovery_publisher)
    : wm_(wm) , plugin_discovery_publisher_(plugin_discovery_publisher)
    {
        plugin_discovery_msg_.name = "StopandWaitPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::TACTICAL;
        plugin_discovery_msg_.capability = "tactical_plan/plan_trajectory";
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
        if(current_speed_== 0.0){
            getWaitTrajectory(resp);
            return true;
        }
        lanelet::BasicPoint2d veh_pos(req.vehicle_state.X_pos_global,req.vehicle_state.Y_pos_global);
        double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

        std::vector<cav_msgs::Maneuver> maneuver_plan;
        for(int i=0;i<req.maneuver_plan.maneuvers.size();i++)
        {
            if(req.maneuver_plan.maneuvers[i].type == cav_msgs::Maneuver::STOP_AND_WAIT)
            {
                maneuver_plan.push_back(req.maneuver_plan.maneuvers[i]);
            }
        }
        ROS_INFO_STREAM("Incoming message, manuever size:"<< maneuver_plan.size());
        ROS_INFO_STREAM("Maneuver type:"<<int(maneuver_plan[0].type));

        std::vector<PointSpeedPair> points_and_target_speeds = maneuvers_to_points(maneuver_plan, current_downtrack, wm_);
        ROS_INFO_STREAM("Stop and Wait, maneuvers to points completed, points size:"<<points_and_target_speeds.size());
        auto downsampled_points = 
            carma_utils::containers::downsample_vector(points_and_target_speeds,8);

        //Trajectory plan
        cav_msgs::TrajectoryPlan  trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

        trajectory.trajectory_points = compose_trajectory_from_centerline(downsampled_points,req.vehicle_state);
        ROS_INFO_STREAM("Stop and Wait, compose traj completed, points size:"<<trajectory.trajectory_points.size());
        trajectory.initial_longitudinal_velocity = req.vehicle_state.longitudinal_vel;
        ROS_INFO_STREAM("Stop and Wait, Inital longitudinal velocity"<<trajectory.initial_longitudinal_velocity);
        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::STOP_AND_WAIT);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);


        return true;
    }

    void StopandWait::getWaitTrajectory(cav_srvs::PlanTrajectoryResponse& resp)
    {
        cav_msgs::TrajectoryPlan  trajectory;
        trajectory.header.frame_id = "map";
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        std::vector<PointSpeedPair> points_and_target_speeds;
        std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points;
        double curr_time = ros::Time::now().toSec();
        double target_time = curr_time;
        lanelet::BasicPoint2d curr_pose (pose_msg_.pose.position.x,pose_msg_.pose.position.y);
        while((target_time - curr_time) <= trajectory_time_length_)
        {
            cav_msgs::TrajectoryPlanPoint trajectory_point;
            trajectory_point.x=curr_pose.x();
            trajectory_point.x=curr_pose.y();
            trajectory_point.yaw= 0.0;
            trajectory_point.target_time = ros::Time(target_time);
            trajectory_point.lane_id;
            trajectory_point.controller_plugin_name = "default";
            trajectory_point.planner_plugin_name =plugin_discovery_msg_.name;
            trajectory_points.push_back(trajectory_point);
            target_time += 0.1;

        }
        trajectory.trajectory_points = trajectory_points;
        trajectory.initial_longitudinal_velocity = 0.0;
        resp.trajectory_plan = trajectory;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::STOP_AND_WAIT);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
        ROS_INFO_STREAM("Wait maneuver in progress");
        double time_diff = (trajectory_points.back().target_time - trajectory_points.front().target_time).toSec();
        ROS_INFO_STREAM("Duration:"<<time_diff);
    }

    std::vector<PointSpeedPair> StopandWait::maneuvers_to_points(const std::vector<cav_msgs::Maneuver>& maneuvers,
                                                                      double max_starting_downtrack,
                                                                      const carma_wm::WorldModelConstPtr& wm)
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
            
            maneuver_time_ = ros::Duration(stop_and_wait_maneuver.end_time - stop_and_wait_maneuver.start_time).toSec();
            
            double delta_time, curr_time;
            if(starting_downtrack == ending_downtrack || current_speed_ == 0)
            {
                delta_time = maneuver_time_/1000;
                curr_time = 0;
                //wait
                lanelet::BasicPoint2d curr_pose (pose_msg_.pose.position.x,pose_msg_.pose.position.y);
                for(int i=0;i<1000;i++)
                {
                    PointSpeedPair pair;
                    pair.point = curr_pose;
                    pair.speed = 0.0;
                    points_and_target_speeds.push_back(pair);
                    curr_time += delta_time;
                }
            }
            else
            {
                double jerk_req, jerk;
                jerk_req = (2*current_speed_)/pow(maneuver_time_,2);
                if(jerk_req > max_jerk_limit_)
                {
                    //unsafe to stop at the required jerk - reset to max_jerk and go beyond the maneuver end_dist
                    jerk = max_jerk_limit_;  
                    maneuver_time_ = pow(2*stop_and_wait_maneuver.start_speed, 0.5);
                    double travel_dist_new = stop_and_wait_maneuver.start_speed * maneuver_time_ - (0.167 * jerk * pow(maneuver_time_,3));
                    ending_downtrack = travel_dist_new + starting_downtrack;
                }
                else jerk = jerk_req;

                //get all the lanelets in between starting and ending downtrack on shortest path
                auto lanelets = wm_->getLaneletsBetween(starting_downtrack, ending_downtrack, true);
                //record all the lanelets to be added to path
                std::vector<lanelet::ConstLanelet> lanelets_to_add;
                for (auto l : lanelets)
                {
                    ROS_DEBUG_STREAM("Lanelet ID: " << l.id());
                    if(visited_lanelets.find(l.id()) == visited_lanelets.end())
                    {
                        lanelets_to_add.push_back(l);
                        visited_lanelets.insert(l.id());
                    }
                }

                lanelet::BasicLineString2d route_geometry = carma_wm::geometry::concatenate_lanelets(lanelets_to_add);
                
                int points_count = route_geometry.size();
                delta_time = maneuver_time_/(points_count-1);

                first = true;
                curr_time = 0.0;

                for(auto p : route_geometry)
                {
                    if (first && points_and_target_speeds.size() != 0)
                    {
                        first = false;

                        continue; //Skip the first point to avoid duplicates from previous maneuver
                    }
                    PointSpeedPair pair;
                    pair.point = p;
                    pair.speed = stop_and_wait_maneuver.start_speed - (0.5 * jerk_req * pow(curr_time,2));
                    ROS_INFO_STREAM("Point speed :"<<pair.speed);
                    if(p == route_geometry.back()) pair.speed = 0.0;    //force speed to 0 at last point
                    curr_time +=delta_time;

                    points_and_target_speeds.push_back(pair);
                }
            }
            ROS_DEBUG_STREAM("Maneuver time:"<<maneuver_time_);
            if(maneuver_time_ < 6.0)
            {
                while(curr_time < minimal_trajectory_duration_)
                {
                    PointSpeedPair pair;
                    pair.point = points_and_target_speeds.back().point;
                    pair.speed = 0.0;
                    curr_time += delta_time;
                }
            }
        }
        
        return points_and_target_speeds;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> StopandWait::compose_trajectory_from_centerline(
    const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
    {        
        int nearest_pt_index = getNearestPointIndex(points,state);
        std::vector<PointSpeedPair> future_points(points.begin() + nearest_pt_index + 1, points.end()); // Points in front of current vehicle position
        auto time_bound_points = constrain_to_time_boundary(future_points, trajectory_time_length_);
        //Get yaw - geometrically
        std::vector<float> yaw_values;
        for(size_t i=0 ;i < time_bound_points.size()-1 ;i++)
        {
            float yaw = atan((time_bound_points[i+1].point.y() - time_bound_points[i].point.y())/ (time_bound_points[i+1].point.x() - time_bound_points[i].point.x()));
            yaw_values.push_back(yaw);
        }
        yaw_values.push_back(0.0); //No rotation from last point

        //get target time from speed
        std::vector<double> target_times;
        std::vector<lanelet::BasicPoint2d> trajectory_locations;
        std::vector<double> trajectory_speeds;
        //split point speed pair
        splitPointSpeedPairs(points,&trajectory_locations,&trajectory_speeds);
        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(trajectory_locations);

        trajectory_utils::conversions::speed_to_time(downtracks, trajectory_speeds, &target_times);

        std::vector <cav_msgs::TrajectoryPlanPoint> traj;
        ros::Time start_time = ros::Time::now();
        cav_msgs::TrajectoryPlanPoint traj_prev;
        traj_prev.x= time_bound_points[0].point.x();
        traj_prev.y = time_bound_points[0].point.y();
        traj_prev.yaw = yaw_values[0];
        traj_prev.target_time = start_time;
        double delta_time = maneuver_time_/points.size();
        for (size_t i=0; i < time_bound_points.size(); i++)
        {
            cav_msgs::TrajectoryPlanPoint traj_point;
            if(trajectory_speeds[i] > 0){
                traj_point.x = time_bound_points[i].point.x();
                traj_point.y = time_bound_points[i].point.y();
                traj_point.yaw = yaw_values[i];
                ROS_INFO_STREAM("target traj time:"<<(start_time + ros::Duration(target_times[i])).toSec() <<" Points size:"<<points.size()<<" Current Iteration:"<<i);
                traj_point.target_time = start_time + ros::Duration(target_times[i]);
            }
            else    //speed_to_time doesn't work for 0.0 speed
            {
                traj_point.x=traj_prev.x;
                traj_point.y=traj_prev.y;
                traj_point.yaw=traj_prev.yaw;
                //increasing time till end of maneuver
                ROS_INFO_STREAM("In else condition target traj time:"<<(start_time + ros::Duration(target_times[i])).toSec() <<" Points size:"<<points.size()<<" Current Iteration:"<<i);
                traj_point.target_time = traj_prev.target_time + ros::Duration(delta_time*i);
                
            }
            traj_point.controller_plugin_name = "default";
            traj_point.planner_plugin_name =plugin_discovery_msg_.name;
            traj.push_back(traj_point);
            traj_prev = traj_point;
        }
        
        return traj;
    }

    int StopandWait::getNearestPointIndex(const std::vector<PointSpeedPair>& points, const cav_msgs::VehicleState& state)
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

    void StopandWait::splitPointSpeedPairs(const std::vector<PointSpeedPair>& points, std::vector<lanelet::BasicPoint2d>* basic_points,
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


    std::vector<PointSpeedPair> StopandWait::constrain_to_time_boundary(const std::vector<PointSpeedPair>& points,
                                                                            double time_span)
    {
        std::vector<lanelet::BasicPoint2d> basic_points;
        std::vector<double> speeds;
        splitPointSpeedPairs(points, &basic_points, &speeds);

        std::vector<double> downtracks = carma_wm::geometry::compute_arc_lengths(basic_points);

        size_t time_boundary_exclusive_index =
        trajectory_utils::time_boundary_index(downtracks, speeds, time_span);

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
}