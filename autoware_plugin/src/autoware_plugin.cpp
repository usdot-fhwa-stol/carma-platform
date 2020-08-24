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
#include "autoware_plugin.h"


namespace autoware_plugin
{
    AutowarePlugin::AutowarePlugin() :
                    current_speed_(0.0),
                    trajectory_time_length_(6.0),
                    trajectory_point_spacing_(0.1) {}

    void AutowarePlugin::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));


        // get vehicle size
        double x,y,z;
        nh_->getParam("vehicle_length", x);
        nh_->getParam("vehicle_width", y);
        nh_->getParam("vehicle_height", z);

        host_vehicle_size.x = x;
        host_vehicle_size.y = y;
        host_vehicle_size.z = z;

        
        maneuver_srv_ = nh_->advertiseService("plugins/AutowarePlugin/plan_maneuvers", &AutowarePlugin::plan_maneuver_cb, this);
        trajectory_srv_ = nh_->advertiseService("plugins/AutowarePlugin/plan_trajectory", &AutowarePlugin::plan_trajectory_cb, this);
                
        autoware_plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "AutowarePlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = false;
        plugin_discovery_msg_.type = cav_msgs::Plugin::STRATEGIC;
        plugin_discovery_msg_.capability = "strategic_plan/plan_maneuvers";

        waypoints_sub_ = nh_->subscribe("final_waypoints", 1, &AutowarePlugin::waypoints_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1, &AutowarePlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &AutowarePlugin::twist_cd, this);
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<double>("trajectory_point_spacing", trajectory_point_spacing_, 0.1);
        pnh_->param<double>("tpmin", tpmin, 0.0);
        pnh_->param<double>("maximum_deceleration_value", maximum_deceleration_value, 0.0);
        pnh_->param<double>("min_downtrack", min_downtrack, 0.0);
        pnh_->param<double>("x_gap", x_gap, 0.0);

        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            autoware_plugin_discovery_pub_.publish(plugin_discovery_msg_);
            return true;
        });
    }


    void AutowarePlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }



    bool AutowarePlugin::plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest &req, cav_srvs::PlanTrajectoryResponse &resp){
        resp.trajectory_plan = trajectory_msg;
        resp.related_maneuvers.push_back(cav_msgs::Maneuver::LANE_FOLLOWING);
        resp.maneuver_status.push_back(cav_srvs::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);

        return true;
    }

    bool AutowarePlugin::plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp){        

        cav_msgs::Maneuver maneuver_msg;

        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;  


        maneuver_msg.lane_following_maneuver.start_dist = 0.0;
        maneuver_msg.lane_following_maneuver.start_speed = ((waypoints_list.size() > 0) ? waypoints_list[0].twist.twist.linear.x : 0.0);
        maneuver_msg.lane_following_maneuver.start_time = ros::Time::now();


        maneuver_msg.lane_following_maneuver.end_dist = 20.0;
        maneuver_msg.lane_following_maneuver.end_speed = ((waypoints_list.size() > 0) ? waypoints_list[waypoints_list.size() - 1].twist.twist.linear.x : 0.0);
        maneuver_msg.lane_following_maneuver.end_time = ros::Time::now()+ros::Duration(mvr_length);
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "AutowarePlugin";

        resp.new_plan.maneuvers.push_back(maneuver_msg);
       
        return true;
    }

    void AutowarePlugin::waypoints_cb(const autoware_msgs::LaneConstPtr& msg)
    {
        if(msg->waypoints.size() == 0)
        {
            ROS_WARN_STREAM("Received an empty trajectory!");
            return;
        }

        cav_msgs::TrajectoryPlan trajectory;
        trajectory.header.frame_id = msg->header.frame_id;
        trajectory.header.stamp = ros::Time::now();
        trajectory.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());
        trajectory.trajectory_points = compose_trajectory_from_waypoints(msg->waypoints);
        waypoints_list = msg->waypoints;

        trajectory_msg = update_traj_for_object(trajectory);
    }

    void AutowarePlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    void AutowarePlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
        velocity = msg->twist;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> AutowarePlugin::compose_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints)
    {
        std::vector<autoware_msgs::Waypoint> partial_waypoints = get_waypoints_in_time_boundary(waypoints, trajectory_time_length_);
        std::vector<cav_msgs::TrajectoryPlanPoint> tmp_trajectory = create_uneven_trajectory_from_waypoints(partial_waypoints);
        std::vector<cav_msgs::TrajectoryPlanPoint> final_trajectory = post_process_traj_points(tmp_trajectory);
        return final_trajectory;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> AutowarePlugin::create_uneven_trajectory_from_waypoints(std::vector<autoware_msgs::Waypoint> waypoints)
    {
        std::vector<cav_msgs::TrajectoryPlanPoint> uneven_traj;
        // TODO land id is not populated because we are not using it in Autoware
        // Adding current vehicle location as the first trajectory point if it is not on the first waypoint
        if(fabs(pose_msg_->pose.position.x - waypoints[0].pose.pose.position.x) > 0.1 || fabs(pose_msg_->pose.position.y - waypoints[0].pose.pose.position.y) > 0.1)
        {
            cav_msgs::TrajectoryPlanPoint starting_point;
            starting_point.target_time = 0.0;
            starting_point.x = pose_msg_->pose.position.x;
            starting_point.y = pose_msg_->pose.position.y;
            uneven_traj.push_back(starting_point);
        }

        double previous_wp_v = waypoints[0].twist.twist.linear.x;
        double previous_wp_x = pose_msg_->pose.position.x;
        double previous_wp_y = pose_msg_->pose.position.y;
        double previous_wp_t = 0.0;
        for(int i = 0; i < waypoints.size(); ++i)
        {
            if(i != 0)
            {
                previous_wp_v = waypoints[i - 1].twist.twist.linear.x;
                previous_wp_x = uneven_traj.back().x;
                previous_wp_y = uneven_traj.back().y;
                previous_wp_t = uneven_traj.back().target_time;
            }
            if(i == 0 && uneven_traj.size() == 0)
            {
                cav_msgs::TrajectoryPlanPoint starting_point;
                starting_point.target_time = 0.0;
                starting_point.x = waypoints[i].pose.pose.position.x;
                starting_point.y = waypoints[i].pose.pose.position.y;
                uneven_traj.push_back(starting_point);
                continue;
            }
            cav_msgs::TrajectoryPlanPoint traj_point;
            // assume the vehicle is starting from stationary state because it is the same assumption made by pure pursuit wrapper node
            double average_speed = previous_wp_v;
            double delta_d = sqrt(pow(waypoints[i].pose.pose.position.x - previous_wp_x, 2) + pow(waypoints[i].pose.pose.position.y - previous_wp_y, 2));
            traj_point.target_time = (delta_d / average_speed) * 1e9 + previous_wp_t;
            traj_point.x = waypoints[i].pose.pose.position.x;
            traj_point.y = waypoints[i].pose.pose.position.y;
            uneven_traj.push_back(traj_point);
        }

        return uneven_traj;
    }

    std::vector<autoware_msgs::Waypoint> AutowarePlugin::get_waypoints_in_time_boundary(std::vector<autoware_msgs::Waypoint> waypoints, double time_span)
    {
        std::vector<autoware_msgs::Waypoint> sublist;
        double total_time = 0.0;
        for(int i = 0; i < waypoints.size(); ++i)
        {
            sublist.push_back(waypoints[i]);
            if(i == 0)
            {
                continue;
            }
            double delta_x_square = pow(waypoints[i].pose.pose.position.x - waypoints[i - 1].pose.pose.position.x, 2);
            double delta_y_square = pow(waypoints[i].pose.pose.position.y - waypoints[i - 1].pose.pose.position.y, 2);
            // Here we ignore z attribute because it is not used by Autoware
            
            double delta_d = sqrt(delta_x_square + delta_y_square);
            double average_v = 0.5 * (waypoints[i].twist.twist.linear.x + waypoints[i - 1].twist.twist.linear.x);
            double delta_t = delta_d / average_v;
            total_time += delta_t;
            if(total_time >= time_span)
            {
                break;
            }
        }
        return sublist;
    }

    std::vector<cav_msgs::TrajectoryPlanPoint> AutowarePlugin::post_process_traj_points(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory)
    {
        uint64_t current_nsec = ros::Time::now().toNSec();
        for(int i = 0; i < trajectory.size(); ++i)
        {
            trajectory[i].controller_plugin_name = "mpc_follower";
            trajectory[i].planner_plugin_name = "autoware";
            trajectory[i].target_time += current_nsec;
        }

        return trajectory;
    }

    cav_msgs::TrajectoryPlan AutowarePlugin::update_traj_for_object(cav_msgs::TrajectoryPlan& original_tp) 
    {
        
        cav_msgs::TrajectoryPlan update_tpp_vector;

        std::cout << "here 244" << std::endl;

        // // TODO get roadway object
        std::vector<cav_msgs::RoadwayObstacle> rwol = wm_->getRoadwayObjects();

        std::vector<cav_msgs::RoadwayObstacle> rwol_collision = carma_wm::collision_detection::WorldCollisionDetection(rwol, original_tp, host_vehicle_size, velocity, 10.0);

        // correct the input types
        if(rwol_collision.size() > 0) {

            // use trajectory utiles to update trajectory plan points

            carma_wm::TrackPos track_pose = wm_->routeTrackPos(lanelet::BasicPoint2d(original_tp.trajectory_points[0].x, original_tp.trajectory_points[0].y));
 
            // lead vehicle trjactory
            double x_lead = rwol_collision[0].down_track;

            // roadway object position
            double gap_time = (x_lead - x_gap)/current_speed_;

            double vt = rwol_collision[0].object.velocity.twist.linear.x;

            double xt = rwol_collision[0].down_track - vt * gap_time;
            double x0 = track_pose.downtrack;

            double a0 = 0;
            double at = 0;

            double collision_time = (x_lead - x0)/(vt - current_speed_); // comming from carma_wm collision detection

            double t0 = 0;

            double tp = 0;
            double delta_v_max = rwol_collision[0].object.velocity.twist.linear.x - max_trajectory_speed(original_tp.trajectory_points);
            double t_ref = original_tp.trajectory_points[0].target_time - original_tp.trajectory_points[original_tp.trajectory_points.size() - 1].target_time;
            double t_ph = 4 * delta_v_max / maximum_deceleration_value;

            if(t_ph > tpmin && t_ref < t_ph){
                tp = t_ph;
            }
            else if(t_ph < tpmin){
                tp = tpmin;
            }
            else {
                tp = t_ref;
            }

            std::vector<double> values = quintic_coefficient_calculator::quintic_coefficient_calculator(x0, 
                                                                                                        xt, 
                                                                                                        current_speed_, 
                                                                                                        vt, 
                                                                                                        a0, 
                                                                                                        at, 
                                                                                                        t0, 
                                                                                                        tp);

            std::vector<cav_msgs::TrajectoryPlanPoint> new_trajectory_points;

            new_trajectory_points.push_back(original_tp.trajectory_points[0]);

            auto shortest_path = wm_->getRoute()->shortestPath();
            std::vector<lanelet::ConstLanelet> tmp;
            lanelet::ConstLanelet start_lanelet;

            for(int i = 1; i < original_tp.trajectory_points.size() - 1; i++ )
            {            
                for (lanelet::ConstLanelet l : shortest_path) {

                    if (l.id()== std::stoi(original_tp.trajectory_points[i].lane_id)) {
                        start_lanelet = l;
                    }
                }

                cav_msgs::TrajectoryPlanPoint new_tpp;
                new_tpp.target_time = i * tp / original_tp.trajectory_points.size();

                double down_track_ = polynomial_calc(values,new_tpp.target_time);

                new_tpp.lane_id = original_tp.trajectory_points[i].lane_id;
                new_tpp.controller_plugin_name = original_tp.trajectory_points[i].controller_plugin_name;
                new_tpp.planner_plugin_name = original_tp.trajectory_points[i].planner_plugin_name;

                for (auto centerline_point:start_lanelet.centerline2d()) {
                    double dt = wm_->routeTrackPos(centerline_point).downtrack;
                    if (dt - down_track_ <= min_downtrack){
                        new_tpp.x = centerline_point.x();
                        new_tpp.y = centerline_point.y();
                        break;
                    }
                }

                double x_original = original_tp.trajectory_points[i].x - original_tp.trajectory_points[i - 1].x;
                double t_original = original_tp.trajectory_points[i].target_time - original_tp.trajectory_points[i - 1].target_time;

                double x_new = new_trajectory_points[i].x - new_trajectory_points[i - 1].x;
                double t_new = new_trajectory_points[i].target_time - new_trajectory_points[i - 1].target_time;

                double v_new = x_new/t_new;
                double v_original = x_original/t_original;

                if(v_new > v_original){
                    carma_wm::TrackPos track_pose = wm_->routeTrackPos(lanelet::BasicPoint2d(original_tp.trajectory_points[i].x, original_tp.trajectory_points[i].y));
                    values = quintic_coefficient_calculator::quintic_coefficient_calculator(track_pose.downtrack, 
                                                                                            xt, 
                                                                                            current_speed_, 
                                                                                            vt, 
                                                                                            a0, 
                                                                                            at, 
                                                                                            t0, 
                                                                                            tp);
                }

                new_tpp.x = polynomial_calc(values,new_tpp.target_time);

                new_trajectory_points.push_back(new_tpp);
            }

            update_tpp_vector.header = original_tp.header;

            update_tpp_vector.trajectory_id = original_tp.trajectory_id;

            update_tpp_vector.trajectory_points = new_trajectory_points;

            return update_tpp_vector;
        }

        return original_tp;
    } 

    double AutowarePlugin::polynomial_calc(std::vector<double> coeff, double x)
    {
        double result = 0;

        for (size_t i = 0; i < coeff.size(); i++) {

            __uint64_t value = coeff[i] * pow(x, (__uint64_t)(coeff.size() - 1 - i));
            
            result = result + value;
        }
    
        return result;
    }

    // TODO
    double AutowarePlugin::max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points) 
    {
        double max_speed = 0;

        for(int i = 0; i < trajectory_points.size() - 2; i++ )
        {
            double x = trajectory_points[i + 1].x - trajectory_points[i].x;
            double t = trajectory_points[i + 1].target_time - trajectory_points[i].target_time;
            double v = x/t;

            if(v > max_speed){
                max_speed = v;
            }

        }

        return max_speed;
    }
}
