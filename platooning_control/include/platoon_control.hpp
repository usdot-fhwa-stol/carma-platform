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
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/TwistStamped.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/Plugin.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>
#include <carma_utils/CARMAUtils.h>
#include "platoon_control_worker.hpp"




namespace platoon_control
{
    class PlatoonControlPlugin
    {
        public:
            
            // Default constructor for PlatoonControlPlugin class
            PlatoonControlPlugin();

            void initialize();

            // general starting point of this node
            void run();

        
        private:

        	// CARMA ROS node handles
        	std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        	PlatoonControlWorker pcw_;


        	void TrajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

        	geometry_msgs::TwistStamped composeTwist(cav_msgs::TrajectoryPlanPoint point);

        	void publishTwist(const geometry_msgs::TwistStamped& twist);

        	// Plugin discovery message
        	cav_msgs::Plugin plugin_discovery_msg_;


        	// ROS Subscriber
        	ros::Subscriber trajectory_plan_sub;
        	// ROS Publisher
        	ros::Publisher twist_pub_;
        	ros::Publisher current_twist_pub_;
        	ros::Publisher plugin_discovery_pub_;

			// TODO: add communication to receive leader
			PlatoonMember leader;




    
    };
}