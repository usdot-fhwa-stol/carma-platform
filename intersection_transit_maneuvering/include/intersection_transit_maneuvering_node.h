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
#include <carma_wm/WMListener.h>
#include <functional>
#include <autoware_msgs/Lane.h>
#include <carma_debug_msgs/TrajectoryCurvatureSpeeds.h>
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
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            ros::Publisher jerk_pub_;
            ros::Timer discovery_pub_timer_;

            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            //Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;
  
            plugin_discovery_pub_ = nh_.advertise<cav_msgs::Plugin>("plugin_discovery",1);
            carma_wm::WMListener wml_;

            carma_wm::WorldModelConstPtr wm_ = wml_.getWorldModel();

            itm_servicer::Servicer srv;
            IntersectionTransitManeuvering worker(wm_,[&plugin_discovery_pub_](const auto& msg) {plugin_discovery_pub_.publish(msg);}, srv);
            ros::ServiceClient trajectory_client = nh_.serviceClient<cav_srvs::PlanTrajectory>("plugin/InlaneCruisingPlugin/plan_trajectory");
            srv.set_client(trajectory_client);

            trajectory_srv_ = nh_.advertiseService("plan_trajectory",&IntersectionTransitManeuvering::plan_trajectory_cb, &worker);           
            
            discovery_pub_timer_ = nh_.createTimer(
                    ros::Duration(ros::Rate(10.0)),
                    [&worker](const auto&) {worker.onSpin();});

            ros::CARMANodeHandle::spin();
        }

};


}