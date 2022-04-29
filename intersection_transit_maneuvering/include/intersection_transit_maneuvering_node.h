#pragma once

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

#include <cav_msgs/Plugin.h>
#include <carma_utils/CARMAUtils.h>
#include <cav_srvs/PlanTrajectory.h>
#include <functional>
#include "intersection_transit_maneuvering.h"
#include "itm_service.h"

namespace intersection_transit_maneuvering
{
    /**
 * \brief ROS node for the InLaneCruisingPlugin
 */
class IntersectionTransitManeuveringNode
{
    public:

        void run()
        {
            //CARMA ROS node handles
            ros::CARMANodeHandle nh_;

            // ROS service servers
            ros::ServiceServer trajectory_srv_;

            //ROS publishers and subscribers
            ros::Publisher plugin_discovery_pub_;
            ros::Timer discovery_pub_timer_;

            //Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

            plugin_discovery_pub_ = nh_.advertise<cav_msgs::Plugin>("plugin_discovery",1);
                        
            std::shared_ptr<intersection_transit_maneuvering::Servicer> srv = std::make_shared<intersection_transit_maneuvering::Servicer>();
            ros::ServiceClient trajectory_client = nh_.serviceClient<cav_srvs::PlanTrajectory>("plugins/InLaneCruisingPlugin/plan_trajectory");
            srv->set_client(trajectory_client);
            IntersectionTransitManeuvering worker([&plugin_discovery_pub_](const auto& msg) {plugin_discovery_pub_.publish(msg);}, srv);
            trajectory_srv_ = nh_.advertiseService("plugins/IntersectionTransitPlugin/plan_trajectory",&IntersectionTransitManeuvering::plan_trajectory_cb, &worker);           
            
            if (!trajectory_client.waitForExistence(ros::Duration(20.0))) {
                throw std::invalid_argument("Required service is not available: " + trajectory_client.getService());
            }

            discovery_pub_timer_ = nh_.createTimer(
                    ros::Duration(ros::Rate(10.0)),
                    [&worker](const auto&) {worker.onSpin();});

            ros::CARMANodeHandle::spin();
        }

};


}