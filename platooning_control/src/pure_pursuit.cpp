
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

#include "platoon_control/pure_pursuit.hpp"
// #include <tf/transform_datatypes.h>
// #include <tf/LinearMath/Matrix3x3.h>


namespace platoon_control
{
    PurePursuit::PurePursuit(){}

	double PurePursuit::getLookaheadDist(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp) const{
		double x_diff = (tp.x - current_pose_->position.x);
		double y_diff = (tp.y - current_pose_->position.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "calculated lookahead: " << dist);
		return dist;
	}


	double PurePursuit::getYaw(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp) const{
		double yaw = atan2(tp.y - current_pose_->position.y, tp.x - current_pose_->position.x);
		return yaw;
	}

	int PurePursuit::getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const{
		double corss_prod = v1[0]*v2[1] - v1[1]*v2[0];
        if (corss_prod >= 0.0){
			 return -1;
		}
        return 1;
	}

	double PurePursuit::getSteeringAngle()
	{
		return steering_angle_;
	}

	double PurePursuit::getAngularVelocity()
	{
		return angular_velocity_;
	}

	double PurePursuit::calculateKappa(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp)
	{
		double lookahead = getLookaheadDist(tp);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "used lookahead: " << lookahead);
		double alpha = getAlphaSin(tp, *current_pose_);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "calculated alpha: " << alpha);
		double kappa = 2 * (alpha)/(lookahead);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "calculated kappa: " << kappa);
		return kappa;
	}

	void PurePursuit::calculateSteer(const carma_planning_msgs::msg::TrajectoryPlanPoint& tp)
	{

		double kappa = calculateKappa(tp);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "kappa pp: " << kappa);

		double lookahead = getLookaheadDist(tp);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "lookahead pp: " << lookahead);


		double error=kappa*lookahead*lookahead/2;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "error term pp: " << error);

		// Integral term
	    _integral += error * config_->dt;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Integral term pp: " << _integral);

		if (_integral > config_->integrator_max_pp){
			 _integral = config_->integrator_max_pp;
		}
		else if (_integral < config_->integrator_min_pp){
			_integral = config_->integrator_min_pp;
		}
	    double Iout = config_->ki_pp * _integral;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Iout pp: " << Iout);
		double steering = atan(config_->wheel_base * kappa)+Iout;


		steering += config_->correction_angle;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "calculated steering angle: " << steering);
		double filtered_steering = lowPassfilter(config_->lowpass_gain, prev_steering, steering);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "filtered steering: " << filtered_steering);
		if (std::isnan(filtered_steering)) filtered_steering = prev_steering;
		prev_steering = filtered_steering;
		steering_angle_ = filtered_steering;

		double ang_vel = velocity_ * kappa;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "calculated angular velocity: " << ang_vel);
		double filtered_ang_vel = lowPassfilter(config_->lowpass_gain, prev_ang_vel, ang_vel);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "filtered angular velocity: " << filtered_ang_vel);
		prev_ang_vel = filtered_ang_vel;
		if (std::isnan(filtered_ang_vel)) filtered_ang_vel = prev_ang_vel;
		angular_velocity_ = filtered_ang_vel;
	}

	double PurePursuit::getAlphaSin(carma_planning_msgs::msg::TrajectoryPlanPoint tp, geometry_msgs::msg::Pose current_pose)
	{

		tf2::Transform inverse;
		tf2::fromMsg(current_pose, inverse);
		tf2::Transform transform = inverse.inverse();

		tf2::Vector3 p;
		p.setValue(tp.x, tp.y, current_pose.position.z);
		tf2::Vector3 tf_p = transform * p;
		geometry_msgs::msg::Point tf_point_msg;
		tf_point_msg.x = tf_p.getX();
		tf_point_msg.y = tf_p.getY();
		tf_point_msg.z = tf_p.getZ();


		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "relative latitude: " << tf_point_msg.y);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "relative longitude: " << tf_point_msg.x);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "relative z: " << tf_point_msg.z);
		double vec_mag = std::sqrt(tf_point_msg.y*tf_point_msg.y + tf_point_msg.x*tf_point_msg.x + tf_point_msg.z*tf_point_msg.z);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "relative vector mag: " << vec_mag);
		double sin_alpha = tf_point_msg.y/vec_mag;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "alpha sin from transform: " << sin_alpha);

		double angle_tp_map = atan2(tp.y, tp.x); // angle of vector to tp point in map frame
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "angle_tp_map: " << angle_tp_map);
		tf2::Quaternion quat;
		tf2::fromMsg(current_pose.orientation, quat);
		double roll, pitch, yaw;
		tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "yaw: " << yaw);
		double alpha = angle_tp_map - yaw;
		double sin_alpha2 = sin(alpha);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "alpha from orientation: " << alpha);
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "alpha sin from orientation: " << sin_alpha2);

		return sin_alpha;
	}

	double PurePursuit::lowPassfilter(double gain, double prev_value, double value)
	{
		value = prev_value + gain*(value - prev_value);
    	return value;
	}


}