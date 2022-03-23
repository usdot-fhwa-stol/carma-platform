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
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/Maneuver.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <cav_msgs/IntersectionTransitLeftTurnManeuver.h>
#include <cav_msgs/IntersectionTransitRightTurnManeuver.h>
#include <cav_msgs/IntersectionTransitStraightManeuver.h>
#include <cav_srvs/PlanTrajectory.h>
#include <functional>
#include <unordered_set>
#include <ros/ros.h>
#include <call_interface.h>

using PublishPluginDiscoveryCB = std::function<void(const cav_msgs::Plugin&)>;

namespace intersection_transit_maneuvering
{


    class IntersectionTransitManeuvering
    {
        public:
        /**
        * \brief Constructor
        * 
        * \param plugin_discovery_publisher Callback which will publish the current plugin discovery state
        * \param obj Interface object to initialize ros::Service::call
        */ 
        IntersectionTransitManeuvering(PublishPluginDiscoveryCB plugin_discovery_publisher, std::shared_ptr<CallInterface> obj);
                       
        /**
         * \brief Service callback for trajectory planning
         * 
         * \param req The service request
         * \param resp The service response
         * 
         * \return True if success. False otherwise
         */ 
        bool plan_trajectory_cb(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& resp);

        /**
         *  \brief Converts a sequence of INTERSECTION_TRANSIT maneuvers to LANE_FOLLOWING maneuvers
         * 
         * \param maneuvers The list of maneuvers to convert
         * 
         * \return The new list of converted maneuvers
        */
        std::vector<cav_msgs::Maneuver> convert_maneuver_plan(const std::vector<cav_msgs::Maneuver>& maneuvers);

        /**
         * @brief Method to call at fixed rate in execution loop. Will publish plugin discovery updates
         * 
         * @return true if successful, otherwise false
         */
        bool onSpin();

    private:

        std::shared_ptr<CallInterface> object_;
        std::vector<cav_msgs::Maneuver> converted_maneuvers_;
        cav_msgs::VehicleState vehicle_state_;
        cav_msgs::Plugin plugin_discovery_msg_;
        PublishPluginDiscoveryCB plugin_discovery_publisher_;

        




    };


}