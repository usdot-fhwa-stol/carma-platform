/*
 * Copyright (C) 2020 LEIDOS.
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
#include "trajectory_visualizer.h"

namespace trajectory_visualizer {

    TrajectoryVisualizer::TrajectoryVisualizer() {}
    void TrajectoryVisualizer::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void TrajectoryVisualizer::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
         // init publishers
        traj_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("trajectory_visualizer", 1, true);

        // init subscribers
        traj_sub_ = nh_->subscribe("plan_trajectory", 1, &TrajectoryVisualizer::callbackPlanTrajectory, this);
    }

    void TrajectoryVisualizer::callbackPlanTrajectory(const cav_msgs::TrajectoryPlan& msg)
    {
        if (msg.trajectory_points.size() == 0)
        {
            ROS_WARN_STREAM("No trajectory point in plan_trajectory! Returning");
        }
        visualization_msgs::MarkerArray tmp_marker_array;
        // display by markers the velocity between each trajectory point/target time.
        visualization_msgs::Marker marker;
        marker.header = msg.header;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.z = 0.4;
        marker.frame_locked = true;

        //some for
        for (auto i = 1; i < msg.trajectory_points.size(); i++)
        {
            
            ros::Duration dt = msg.trajectory_points[i].target_time - msg.trajectory_points[i - 1].target_time;
            
            double dx = msg.trajectory_points[i].x - msg.trajectory_points[i-1].x;
            double dy = msg.trajectory_points[i].y - msg.trajectory_points[i-1].y;
            double dist = sqrt(dx * dx + dy * dy);

            double speed = dist/ dt.toSec();

            // map color to the scale of the speed
            // red being the highest, green being the lowest (0ms)
            double max_speed = 
            
            velocity.id = i;
            geometry_msgs::Point relative_p;
            relative_p.y = -0.65;
            velocity.pose.position = calcAbsoluteCoordinate(relative_p, lane_waypoint.waypoints[i].pose.pose);
            velocity.pose.position.z += 0.2;

            // double to string
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(1) << mps2kmph(lane_waypoint.waypoints[i].twist.twist.linear.x);
            velocity.text = oss.str();

            tmp_marker_array.markers.push_back(velocity);
        }

        traj_marker_pub_.publish(tmp_marker_array);
    }
}

