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
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


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

			// Compose twist message by calculating speed and steering commands.
			void generateControlSignals(const cav_msgs::TrajectoryPlanPoint& point0, const cav_msgs::TrajectoryPlanPoint& point_end);
			
			// Compose twist message by calculating speed and steering commands.
			geometry_msgs::TwistStamped composeTwistCmd(double linear_vel, double angular_vel);

			// Compose control message by calculating speed and steering commands.
			autoware_msgs::ControlCommandStamped composeCtrlCmd(double linear_vel, double steering_angle);

			// find the point correspoding to the lookahead distance
			cav_msgs::TrajectoryPlanPoint getLookaheadTrajectoryPoint(cav_msgs::TrajectoryPlan trajectory_plan);

			// timer callback for control signal publishers
			// returns true if control signals are correctly calculated.
			bool controlTimerCb();
			
			// local copy of pose
        	geometry_msgs::PoseStamped pose_msg_;

			// current speed (in m/s)
			double current_speed_ = 0.0;
			double trajectory_speed_ = 0.0;

			bool emergency_stop_flag = false;

			cav_msgs::TrajectoryPlan latest_trajectory_;

        
        private:


        	// CARMA ROS node handles
        	std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

			// platoon control worker object
        	PlatoonControlWorker pcw_;

			// platooning config object
			PlatooningControlPluginConfig config_;


			// Variables
			PlatoonLeaderInfo platoon_leader_;
			long prev_input_time_ = 0;				//timestamp of the previous trajectory plan input received
			long consecutive_input_counter_ = 0;	//num inputs seen without a timeout

			// callback function for pose
			void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

			// callback function for platoon info
			void platoonInfo_cb(const cav_msgs::PlatooningInfoConstPtr& msg);

			// callback function for trajectory plan
        	void trajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

			// callback function for current twist
			void currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist);

			void emergency_stop_cb(const std_msgs::BoolConstPtr& msg);

			double getTrajectorySpeed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points);

        	// Plugin discovery message
        	cav_msgs::Plugin plugin_discovery_msg_;

        	// ROS Subscriber
        	ros::Subscriber trajectory_plan_sub;
			ros::Subscriber current_twist_sub_;
			ros::Subscriber pose_sub_;
			ros::Subscriber platoon_info_sub_;
			ros::Subscriber emergency_stop_sub_;
        	// ROS Publisher
        	ros::Publisher twist_pub_;
        	ros::Publisher controller_setting_pub_;
			ros::Publisher ctrl_pub_;
        	ros::Publisher plugin_discovery_pub_;
			ros::Publisher platoon_info_pub_;
			ros::Publisher preview_point_pub_;
			ros::Timer discovery_pub_timer_;
			ros::Timer control_pub_timer_;
    };
}
