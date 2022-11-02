
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

	/**
    * \brief This class includes logic for Pure Pursuit controller. This controller is used to calculate a steering angle using the current pose of the vehcile and a lookahead point.
    */
    class PurePursuit
    {


    public:

		/**
        * \brief Default constructor for pure pursuit
        */
    	PurePursuit();

		/**
        * \brief calculates steering angle in rad based on lookahead trajectory point
		* \param tp lookahead trajectory point
        */
    	void calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp);

		/**
        * \brief calculates curvature to the lookahead trajectory point
		* \param tp lookahead trajectory point
		* \return curvature to the lookahead point
        */
		double calculateKappa(const cav_msgs::TrajectoryPlanPoint& tp);

		/**
        * \brief calculates sin of the heading angle to the target point
		* \param tp lookahead trajectory point
		* \param current_pose current pose of the vehicle
		* \return sin of the heading angle
        */ 
		double getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose);

		/**
        * \brief Lowpass filter to smoothen control signal
		* \param gain filter gain
		* \param prev_value previous value
		* \param value current value
		* \return smoothened control signal
        */ 
		double lowPassfilter(double gain, double prev_value, double value);

		/**
        * \brief returns steering angle
		* \return steering angle in rad
        */ 
		double getSteeringAngle();
		
		/**
        * \brief returns angular velocity
		* \return angular velocity in rad/s
        */  
		double getAngularVelocity();

		// geometry pose
		geometry_msgs::Pose current_pose_;
		double velocity_ = 0.0;
		double angular_velocity_ = 0;
		double steering_angle_ = 0;

		PlatooningControlIHPPluginConfig config_;

    private:

		/**
        * \brief calculate lookahead distance
		* \param tp trajectory point
		* \return lookahead distance from next trajectory point in m
        */
		double getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const;

		/**
        * \brief calculate yaw angle of the vehicle
		* \param tp trajectory point
		* \return yaw angle of the vehicle in rad
        */
		double getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const;

		/**
        * \brief calculate steering direction
		* \param tp trajectory point
		* \return steering direction (+1 is left and -1 is right)
        */
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
