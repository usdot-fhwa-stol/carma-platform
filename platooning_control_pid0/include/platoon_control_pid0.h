/*
 * Copyright (C) 2019-2021 LEIDOS.
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
#include <geometry_msgs/PoseStamped.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/Plugin.h>
#include <cav_msgs/PlatooningInfo.h>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <math.h>
#include <carma_utils/CARMAUtils.h>
#include "platoon_control_worker.h"


namespace platoon_control_pid0
{
    class PlatoonControlPid0Plugin
    {
        public:
            
			// Default constructor for the class
			PlatoonControlPid0Plugin();

			void initialize();

			// general starting point of this node
			void run();

			// callback function for pose
			void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

			// callback function for platoon info
			void platoon_info_cb(const cav_msgs::PlatooningInfoConstPtr& msg);

			// callback function for current twist
			void current_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist);

			// callback function for trajectory plan
        	void trajectory_plan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

			bool control_timer_cb();

        
        private:

			// member variables
        	std::shared_ptr<ros::CARMANodeHandle>	nh_, pnh_;					//ROS1 node handles
			PlatoonControlPluginConfig				config_;					//holds the plugin config params
        	PlatoonControlWorker					pcw_;						//platoon control worker object does all the math
        	cav_msgs::Plugin						plugin_discovery_msg_;		//holds ~constant info to publish for discovery
			ros::Timer 								discovery_pub_timer_;		//timer for publishing discovery messages
			ros::Timer								control_timer_;				//timer for running the control loop
			long									consecutive_input_ctr_ = 0;	//number of consecutive timely trajectory inputs since restart

        	// ROS Subscribers
        	ros::Subscriber							trajectory_plan_sub_;
			ros::Subscriber							current_twist_sub_;
			ros::Subscriber							pose_sub_;
			ros::Subscriber 						platoon_info_sub_;

        	// ROS Publishers
        	ros::Publisher 							twist_pub_;
			ros::Publisher							ctrl_pub_;
			ros::Publisher 							platoon_info_pub_;
        	ros::Publisher 							plugin_discovery_pub_;

			// Compose twist message by calculating speed and steering commands.
			geometry_msgs::TwistStamped compose_twist_cmd(double linear_vel, double angular_vel);

			// Compose control message by calculating speed and steering commands.
			autoware_msgs::ControlCommandStamped compose_ctrl_cmd(double linear_vel, double steering_angle);
    };
}
