/*
 * Copyright (C) 2019 LEIDOS.
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
        trajectory_pub_ = nh_->advertise<cav_msgs::TrajectoryPlan>("plan_trajectory", 1);
        waypoints_sub_ = nh_->subscribe("final_waypoints", 1, &AutowarePlugin::waypoints_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1, &AutowarePlugin::pose_cb, this);
        twist_sub_ = nh_->subscribe("current_velocity", 1, &AutowarePlugin::twist_cd, this);
        pnh_->param<double>("trajectory_time_length", trajectory_time_length_, 6.0);
        pnh_->param<double>("trajectory_point_spacing", trajectory_point_spacing_, 0.1);
    }

    void AutowarePlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
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
        trajectory_pub_.publish(trajectory);
    }

    void AutowarePlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = msg;
    }

    void AutowarePlugin::twist_cd(const geometry_msgs::TwistStampedConstPtr& msg)
    {
        current_speed_ = msg->twist.linear.x;
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
            //double delta_z_square = waypoints[i].pose.pose.position.z - waypoints[i - 1].pose.pose.position.z;
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
            trajectory[i].controller_plugin_name = "pure_pursuit";
            trajectory[i].planner_plugin_name = "autoware";
            trajectory[i].target_time += current_nsec;
        }

        return trajectory;
    }
}
