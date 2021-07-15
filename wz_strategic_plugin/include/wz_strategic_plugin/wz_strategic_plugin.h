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
#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_utils/CARMAUtils.h>

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

            cav_msgs::Maneuver composeLaneFollowingManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id);

            cav_msgs::Maneuver composeStopandWaitManeuverMessage(double current_dist, double end_dist, double current_speed, int start_lane_id, int end_lane_id, ros::Time current_time, double end_time);

            cav_msgs::Maneuver composeWorkZoneManeuverMessage(double start_dist, double end_dist, double start_speed, double target_speed, lanelet::Id lane_id);

            std::string traffic_light_interpreter(int state);

        private:
            // CARMA ROS node handles
            std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

            // wm listener pointer and pointer to the actual wm object
            std::shared_ptr<carma_wm::WMListener> wml_;
            carma_wm::WorldModelConstPtr wm_;

            int min_distance_to_traffic_light = 30;
    };
}