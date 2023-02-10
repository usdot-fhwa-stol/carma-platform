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




namespace platoon_control
{
	/**
    * \brief This class includes logic for Platoon control. It includes publishers and subscribers and their callback functions
	*/
    class PlatoonControlPlugin
    {
        public:
            
			/**
			* \brief Default constructor for PlatoonControlPlugin class
			*/
			PlatoonControlPlugin();

			/**
			* \brief Initialize plugin variables and publishers/subscribers
			*/
			void initialize();

			/**
			* \brief  general starting point of this node
			*/
			void run();

			/**
			* \brief generate control signal by calculating speed and steering commands.
			* \param point0 start point of control window
			* \param point_end end point of control wondow
			*/
			void generateControlSignals(const cav_msgs::TrajectoryPlanPoint& point0, const cav_msgs::TrajectoryPlanPoint& point_end);
			
			/**
			* \brief Compose twist message from linear and angular velocity commands.
			* \param linear_vel linear velocity in m/s
			* \param angular_vel angular velocity in rad/s
			* \return twist message
			*/
			geometry_msgs::TwistStamped composeTwistCmd(double linear_vel, double angular_vel);

			/**
			* \brief Compose control message from speed and steering commands.
			* \param linear_vel linear velocity in m/s
			* \param steering_angle steering angle in rad
			* \return control command
			*/
			autoware_msgs::ControlCommandStamped composeCtrlCmd(double linear_vel, double steering_angle);

			/**
			* \brief find the point correspoding to the lookahead distance
			* \param trajectory_plan trajectory plan
			* \return trajectory point
			*/
			cav_msgs::TrajectoryPlanPoint getLookaheadTrajectoryPoint(cav_msgs::TrajectoryPlan trajectory_plan);

			/**
			* \brief timer callback for control signal publishers
			* \returns true if control signals are correctly calculated.
			*/
			bool controlTimerCb();
			
			// local copy of pose
        	geometry_msgs::PoseStamped pose_msg_;

			// current speed (in m/s)
			double current_speed_ = 0.0;
			double trajectory_speed_ = 0.0;

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

			/**
			* \brief callback function for pose
			* \param msg pose stamped msg
			*/
			void pose_cb(const geometry_msgs::PoseStampedConstPtr& msg);

			/**
			* \brief callback function for platoon info
			* \param msg platoon info msg
			*/
			void platoonInfo_cb(const cav_msgs::PlatooningInfoConstPtr& msg);

			/**
			* \brief callback function for trajectory plan
			* \param msg trajectory plan msg
			*/
        	void trajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp);

			/**
			* \brief callback function for current twist
			* \param msg twist stamped msg
			*/
			void currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist);

			/**
			* \brief calculate average speed of a set of trajectory points
			* \param trajectory_points set of trajectory points
			* \return trajectory speed
			*/
			double getTrajectorySpeed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points);

        	// Plugin discovery message
        	cav_msgs::Plugin plugin_discovery_msg_;

        	// ROS Subscriber
        	ros::Subscriber trajectory_plan_sub;
			ros::Subscriber current_twist_sub_;
			ros::Subscriber pose_sub_;
			ros::Subscriber platoon_info_sub_;
        	// ROS Publisher
        	ros::Publisher twist_pub_;
			ros::Publisher ctrl_pub_;
        	ros::Publisher plugin_discovery_pub_;
			ros::Publisher platoon_info_pub_;
			ros::Timer discovery_pub_timer_;
			ros::Timer control_pub_timer_;
    };
}
