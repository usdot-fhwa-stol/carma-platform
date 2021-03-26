/*
 * Copyright (C) 2021 LEIDOS.
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

#include <functional>
// ROS
#include <ros/ros.h>
#include <cav_msgs/Plugin.h>
// msgs
#include <cav_msgs/TrajectoryPlan.h>
#include <autoware_msgs/Lane.h>
#include "pure_pursuit_jerk_wrapper_config.hpp"
#include <std_msgs/Float64.h>
#include <trajectory_utils/trajectory_utils.h>

namespace pure_pursuit_jerk_wrapper {

using WaypointPub = std::function<void(autoware_msgs::Lane)>;
using PluginDiscoveryPub = std::function<void(cav_msgs::Plugin)>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */

class PurePursuitJerkWrapper {
    public:

        PurePursuitJerkWrapper(PurePursuitJerkWrapperConfig config,WaypointPub waypoint_pub, PluginDiscoveryPub plugin_discovery_pub);

        void trajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp);
        /**
         * \brief callback function for jerk published by constant jerk plugin. This function updates the local jerk variable used in time_to_speed calculation
         * \param jerk float64 message with jerk value from the plugin
         */
        void updatejerk(std_msgs::Float64 jerk);
        
        std::vector<double> apply_response_lag(const std::vector<double>& speeds, const std::vector<double> downtracks, double response_lag) const;

        bool onSpin();

    private:
    WaypointPub waypoint_pub_;
    PluginDiscoveryPub plugin_discovery_pub_;
    ros::Subscriber jerk_sub_; 
    cav_msgs::Plugin plugin_discovery_msg_;
    PurePursuitJerkWrapperConfig config_;
    double jerk_;

};

}  // namespace pure_pursuit_jerk_wrapper