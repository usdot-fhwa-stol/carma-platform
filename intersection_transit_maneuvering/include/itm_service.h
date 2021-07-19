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

#include <ros/ros.h>
#include <vector>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Maneuver.h>
#include <cav_msgs/Plugin.h>
#include <boost/shared_ptr.hpp>
#include <carma_utils/CARMAUtils.h>
#include <boost/geometry.hpp>
#include <carma_wm/Geometry.h>
#include <cav_srvs/PlanTrajectory.h>


namespace itm_servicer
{
class Servicer
{
    public:
        /**
         * @brief Construct a new Servicer object
         * 
         */
        Servicer();

        bool call(const std::string& service_name,cav_srvs::PlanTrajectory::Request& req, cav_srvs::PlanTrajectory::Response& resp);

        /**
        * \brief set the trajectory service client
        * 
        * \param client input trajectory service client
        */
        void set_client(ros::ServiceClient srv_client);

        cav_srvs::PlanTrajectory make_plan_trajectory(std::vector<cav_msgs::Maneuver> maneuvers, cav_msgs::VehicleState vehicle_state);
    
    private:
        ros::ServiceClient client;


}

}