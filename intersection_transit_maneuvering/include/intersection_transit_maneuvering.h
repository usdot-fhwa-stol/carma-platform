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
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_wm/WMListener.h>
#include <functional>
#include <unordered_set>
#include <autoware_msgs/Lane.h>
#include <ros/ros.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
#include <itm_helper.h>
#include <basic_autonomy/helper_functions.h>

namespace intersection_transit_maneuvering
{


    class IntersectionTransitManeuvering: public Interface
    {
        public:
        /**
        * \brief Constructor
        * 
        * \param wm Pointer to intialized instance of the carma world model for accessing semantic map data
        */ 
        IntersectionTransitManeuvering(carma_wm::WorldModelConstPtr wm);
                       
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

    private:
        //Plan Trajectory Service to send to Inlane Cruising
        cav_srvs::PlanTrajectory traj_srv;


    }
















}