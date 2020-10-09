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
#include "lane_array_to_lane_converter.h"

namespace lane_array_to_lane_converter {

    LaneArrayToLaneConverter::LaneArrayToLaneConverter() {}
    void LaneArrayToLaneConverter::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void LaneArrayToLaneConverter::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
         // init publishers
        lane_pub_ = nh_->advertise<autoware_msgs::Lane>("base_waypoints", 1);

        // init subscribers
        lane_array_sub_ = nh_->subscribe("traffic_waypoints_array", 1, &LaneArrayToLaneConverter::callbackFromLaneArray, this);
    }

    void LaneArrayToLaneConverter::callbackFromLaneArray(const autoware_msgs::LaneArrayConstPtr &msg)
    {
        if (msg->lanes.size() == 0)
        {
            ROS_WARN_STREAM("lane_array_to_lane_converter: Lane Array received has no lane inside! Returning");
            return;
        }
        current_lane_ = msg->lanes[0]; //currently hardcoded to expect only one lane

        if (current_lane_.waypoints.empty())
        {
            ROS_WARN_STREAM("lane_array_to_lane_converter: First lane in the Lane Array has no waypoints inside! Returning");
            return;
        }
        lane_pub_.publish(current_lane_);
    }
}

