
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

------------------------------------------------------------------------------*/


#include <ros/ros.h>
#include <math.h> 
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cav_msgs/TrajectoryPlan.h>
#include "platoon_control_ihp_config.h"







namespace platoon_control_ihp
{


    class PurePursuit
    {


    public:

		/**
        * \brief Default constructor for pure pursuit
        */
    	PurePursuit();

		/**
        * \brief calculates steering angle based on lookahead trajectory point
        */
    	void calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp);

		/**
        * \brief calculates curvature to the lookahead trajectory point
        */
		double calculateKappa(const cav_msgs::TrajectoryPlanPoint& tp);

		// geometry pose
		geometry_msgs::Pose current_pose_;
		double velocity_ = 0.0;
		double angular_velocity_ = 0;
		double steering_angle_ = 0;

		PlatooningControlIHPPluginConfig config_;

		/**
        * \brief calculates sin of the heading angle to the target point
        */ 
		double getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose);

		/**
        * \brief Lowpass filter to smoothen control signal
        */ 
		double lowPassfilter(double gain, double prev_value, double value);

		/**
        * \brief returns steering angle
        */ 
		double getSteeringAngle();
		
		/**angular velocity
        */ 
		double getAngularVelocity();

    private:

		// calculate the lookahead distance from next trajectory point
		double getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const;

		// calculate yaw angle of the vehicle
		double getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const;

		// calculate steering direction
		int getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const;

		


		double prev_steering = 0.0;
		double prev_ang_vel = 0.0;
		

		
		// previous trajectory point
		cav_msgs::TrajectoryPlanPoint tp0;


		// helper function (if needed)
		// inline double deg2rad(double deg) const
		// {
		// 	return deg * M_PI / 180;
		// }  // convert degree to radian


    };
}
