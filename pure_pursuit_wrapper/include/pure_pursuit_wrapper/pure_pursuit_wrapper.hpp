/*
 * Copyright (C) 2018-2021 LEIDOS.
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
#include "pure_pursuit_wrapper_config.hpp"
#include <algorithm>
#include <trajectory_utils/trajectory_utils.h>

namespace pure_pursuit_wrapper {

using WaypointPub = std::function<void(autoware_msgs::Lane)>;
using PluginDiscoveryPub = std::function<void(cav_msgs::Plugin)>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */

class PurePursuitWrapper {
    public:

        PurePursuitWrapper(PurePursuitWrapperConfig config, WaypointPub waypoint_pub, PluginDiscoveryPub plugin_discovery_pub);

        void trajectoryPlanHandler(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

        bool onSpin();

        /**
         * \brief Applies a specified response lag in seconds to the trajectory shifting the whole thing by the specified lag time
         * \param speeds Velocity profile to shift. The first point should be the current vehicle speed
         * \param downtrack Distance points for each velocity point. Should have the same size as speeds and start from 0
         * \param response_lag The lag in seconds before which the vehicle will not meaningfully accelerate
         * 
         * \return A Shifted trajectory
         */ 
        std::vector<double> apply_response_lag(const std::vector<double>& speeds, const std::vector<double> downtracks, double response_lag) const;

        /**
         * \brief Drops any points that sequentially have same target_time and return new trajectory_points in order to avoid divide by zero situation
         * \param traj_points Velocity profile to shift. The first point should be the current vehicle speed
         * 
         * NOTE: This function assumes the target_time will not go backwards. In other words, it only removes "sequential" points that have same target_time
         * \return A new trajectory without any repeated time_stamps
         */ 
        std::vector<cav_msgs::TrajectoryPlanPoint> remove_repeated_timestamps(const std::vector<cav_msgs::TrajectoryPlanPoint>& traj_points);
    private:
    PurePursuitWrapperConfig config_;
    WaypointPub waypoint_pub_;
    PluginDiscoveryPub plugin_discovery_pub_;
    cav_msgs::Plugin plugin_discovery_msg_;

};

}  // namespace pure_pursuit_wrapper