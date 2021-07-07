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
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_msgs/Plugin.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

namespace wz_strategic_plugin
{
    class WzStrategicPlugin
    {
        public:
            /**
             * \brief Default constructor for WzStrategicPlugin class
             */
            WzStrategicPlugin() = default;

            /**
             * \brief General entry point to begin the operation of this class
             */
            void run();

            /**
             * \brief Initialize ROS publishers, subscribers, service servers and service clients
             */
            void initialize();


            bool planManeuverCb(cav_srvs::PlanManeuversRequest &req, cav_srvs::PlanManeuversResponse &resp);

            ros::Publisher plugin_discovery_pub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;

            // ROS service servers
            ros::ServiceServer plan_maneuver_srv_;  

            // Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            //Queue of maneuver plans
            std::vector<cav_msgs::Maneuver> latest_maneuver_plan_;

            ros::Timer discovery_pub_timer_;


            // Current vehicle forward speed
            double current_speed_;

            // Current vehicle pose in map
            geometry_msgs::PoseStampedConstPtr pose_msg_;
            // lanelet::BasicPoint2d current_loc_;

            /**
             * \brief Callback for the pose subscriber, which will store latest pose locally
             * \param msg Latest pose message
             */
            void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

            /**
             * \brief Callback for the twist subscriber, which will store latest twist locally
             * \param msg Latest twist message
             */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);

        private:
            // CARMA ROS node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;
    };
}