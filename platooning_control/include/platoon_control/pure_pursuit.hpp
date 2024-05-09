/*------------------------------------------------------------------------------
* Copyright (C) 2024 LEIDOS.
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


#include <rclcpp/rclcpp.hpp>
#include <math.h>
#include <iostream>
#include <geometry_msgs/msg/pose_stamped.h>

#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Dense>

#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include "platoon_control/platoon_control_config.hpp"


namespace platoon_control
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
        * \brief calculates steering angle based on lookahead trajectory point
		* \param tp lookahead trajectory point
        */
    	void calculateSteer(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp);

        /**
        * \brief calculates curvature to the lookahead trajectory point
		* \param tp lookahead trajectory point
		* \return curvature to the lookahead point
        */
		double calculateKappa(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp);


        /**
        * \brief calculates sin of the heading angle to the target point
		* \param tp lookahead trajectory point
		* \param current_pose current pose of the vehicle
		* \return sin of the heading angle
        */
		double getAlphaSin(carma_planning_msgs::msg::TrajectoryPlanPoint tp, geometry_msgs::msg::Pose current_pose);


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
		std::shared_ptr<geometry_msgs::msg::Pose> current_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
		double velocity_ = 0.0;
		double angular_velocity_ = 0;
		double steering_angle_ = 0;

		std::shared_ptr<PlatooningControlPluginConfig> config_ = std::make_shared<PlatooningControlPluginConfig>();


    private:

        /**
        * \brief calculate lookahead distance
		* \param tp trajectory point
		* \return lookahead distance from next trajectory point
        */
		double getLookaheadDist(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp) const;

        /**
        * \brief calculate yaw angle of the vehicle
		* \param tp trajectory point
		* \return yaw angle of the vehicle in rad
        */
		double getYaw(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp) const;


        /**
        * \brief calculate steering direction
		* \param tp trajectory point
		* \return steering direction (+1 is left and -1 is right)
        */
		int getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const;


        double prev_steering = 0.0;
		double prev_ang_vel = 0.0;

        // previous trajectory point
		carma_planning_msgs::msg::TrajectoryPlanPoint tp0;

		double _integral = 0.0;


    };
}