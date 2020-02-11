#pragma once

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

#include <vector>
#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <cav_srvs/PlanManeuvers.h>

namespace route_following_plugin
{

    class RouteFollowingPlugin
    {

    public:

        static constexpr double TWENTY_FIVE_MPH_IN_MS = 11.176;

        RouteFollowingPlugin();

        // general starting point of this node
        void run();

    private:

        // node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // ROS publishers and subscribers
        ros::Publisher plugin_discovery_pub_;
        ros::Subscriber pose_sub_;
        ros::Subscriber twist_sub_;

        // ros service servers
        ros::ServiceServer plan_maneuver_srv_;        

        // minimal duration of maneuver
        double mvr_duration_;

        // plugin discovery message
        cav_msgs::Plugin plugin_discovery_msg_;

        // local copy of current vehicle speed
        double current_speed_;

        // local copy of current vehicle pose
        geometry_msgs::PoseStamped pose_msg_;

        // wm listener and pointer to the actual wm object
        carma_wm::WMListener wml_;
        carma_wm::WorldModelConstPtr wm_;

        // initialize this node
        void initialize();

        // callback for the subscribers
        void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);
        void twist_cd(const geometry_msgs::TwistStampedConstPtr& msg);

        // service callback for carma maneuver planning
        bool plan_maneuver_cb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

    };

}

