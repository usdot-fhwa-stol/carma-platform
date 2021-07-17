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
            ros::CARMANodeHandle nh_,pnh_, pnh2_;

            // ROS service servers
            ros::ServiceServer trajectory_srv_;

            //ROS publishers and subscribers
            ros::Publisher plugin_discovery_pub_;
            ros::Subscriber pose_sub_;
            ros::Subscriber twist_sub_;
            ros::Publisher jerk_pub_;
            ros::Timer discovery_pub_timer_;

            ros::ServiceServer trajectory_srv_;
            // Current vehicle pose in map
            geometry_msgs::PoseStamped pose_msg_;

            //Plugin discovery message
            cav_msgs::Plugin plugin_discovery_msg_;

  
            carma_wm::WorldModelConstPtr wm_;
            IntersectionTransitManeuvering::IntersectionTransitManeuvering worker(wm_);


            nh_.reset(new ros::CARMANodeHandle());
            pnh_.reset(new ros::CARMANodeHandle("~"));
            pnh2_.reset(new ros::CARMANodeHandle("/"));

            trajectory_srv_ = nh_.advertiseService("plan_trajectory",&IntersectionTransitManeuvering::plan_trajectory_cb, this);
        
            plugin_discovery_pub_ = nh_.advertise<cav_msgs::Plugin>("plugin_discovery",1);
        
            wml_.reset(new carma_wm::WMListener());
            wm_ = wml_->getWorldModel();

        

            ros::ServiceClient trajectory_client = nh_.serviceClient<cav_srvs::PluginTrajectory>("plugin/InlaneCruisingPlugin/plan_trajectory");

            worker.set_traajectory_client(trajectory_client);
            ROS_INFO_STREAM("InlaneCruising Trajectory Client Set");

            ros::Timer discovery_pub_timer_ = nh.createTimer(
                    ros::Duration(ros::Rate(10.0)),
                    [&worker](const auto&) {worker.onSpin();});

            ros::CARMANodeHandle::spin();
        }




}


}