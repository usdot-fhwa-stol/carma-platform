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
#include "lane_select.h"

namespace lane_select {

    LaneSelect::LaneSelect() {}
    void LaneSelect::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void LaneSelect::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
         // init publishers
        lane_pub_ = nh_->advertise<autoware_msgs::Lane>("base_waypoints", 1);
        lane_select_viz_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("lane_select_marker", 1);

        // init subscribers
        lane_array_sub_ = nh_->subscribe("traffic_waypoints_array", 1, &LaneSelect::callbackFromLaneArray, this);
    }

    void LaneSelect::callbackFromLaneArray(const autoware_msgs::LaneArrayConstPtr &msg)
    {
        if (msg->lanes.size() == 0)
        {
            ROS_WARN_STREAM("lane_select: Lane Array received has no lane inside! Returning");
            return;
        }
        current_lane_ = msg->lanes[0]; //currently hardcoded to expect only one lane

        if (current_lane_.waypoints.empty())
        {
            ROS_WARN_STREAM("lane_select: First lane in the Lane Array has no waypoints inside! Returning");
            return;
        }
        publishVisualizer();
        lane_pub_.publish(current_lane_);
    }

    void LaneSelect::publishVisualizer()
    {
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(createCurrentLaneMarker());
        lane_select_viz_pub_.publish(marker_array);
    }

    visualization_msgs::Marker LaneSelect::createCurrentLaneMarker()
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time();
        marker.ns = "current_lane_marker";

        if (current_lane_.waypoints.empty())
        {
            marker.action = visualization_msgs::Marker::DELETE;
            return marker;
        }

        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;

        std_msgs::ColorRGBA color_current;
        color_current.b = 1.0;
        color_current.g = 0.7;
        color_current.a = 1.0;
        marker.color = color_current;

        for (const auto &em : current_lane_.waypoints)
            marker.points.push_back(em.pose.pose.position);

        return marker;
    }
}

