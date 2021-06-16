#include "pure_pursuit.hpp"



namespace platoon_control
{
	PurePursuit::PurePursuit(){}

	double PurePursuit::getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double x_diff = (tp.x - current_pose_.position.x);
		double y_diff = (tp.y - current_pose_.position.y);
		double dist = std::sqrt(x_diff * x_diff + y_diff * y_diff);
		ROS_DEBUG_STREAM("calculated lookahead: " << dist);
		return dist;
	}


	double PurePursuit::getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double yaw = atan2(tp.y - current_pose_.position.y, tp.x - current_pose_.position.x);
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

	double PurePursuit::calculateKappa(const cav_msgs::TrajectoryPlanPoint& tp)
	{
		double lookahead = getLookaheadDist(tp);
		ROS_DEBUG_STREAM("calculated lookahead: " << lookahead);
		double alpha = getAlphaSin(tp, current_pose_);
		ROS_DEBUG_STREAM("calculated alpha: " << alpha);
		double kappa = 2 * (alpha)/(lookahead);
		ROS_DEBUG_STREAM("calculated kappa: " << kappa);
		return kappa;
	}

	void PurePursuit::calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp)
	{

		double kappa = calculateKappa(tp);
		
		double steering = atan(config_.wheelbase * kappa);
		ROS_DEBUG_STREAM("calculated steering angle: " << steering);
		double filtered_steering = lowPassfilter(config_.lowpass_gain, steering, prev_steering);
		ROS_DEBUG_STREAM("filtered steering: " << filtered_steering);
		if (std::isnan(filtered_steering)) filtered_steering = prev_steering;
		prev_steering = filtered_steering;
		steering_angle_ = filtered_steering;
		
		double ang_vel = velocity_ * kappa;
		ROS_DEBUG_STREAM("calculated angular velocity: " << ang_vel);
		double filtered_ang_vel = lowPassfilter(config_.lowpass_gain, ang_vel, prev_ang_vel);
		ROS_DEBUG_STREAM("filtered steering: " << filtered_steering);
		prev_ang_vel = filtered_ang_vel;
		if (std::isnan(filtered_ang_vel)) filtered_ang_vel = prev_ang_vel;
		angular_velocity_ = filtered_ang_vel;
	}

	double PurePursuit::getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose)
	{
		tf::Transform inverse;
		tf::poseMsgToTF(current_pose, inverse);
		tf::Transform transform = inverse.inverse();

		geometry_msgs::Point point_msg;
		point_msg.x = tp.x;
		point_msg.y = tp.y;
		tf::Point p;
		pointMsgToTF(point_msg, p);
		tf::Point tf_p = transform * p;
		geometry_msgs::Point tf_point_msg;
		pointTFToMsg(tf_p, tf_point_msg);
		double sin_alpha = tf_point_msg.y;
		return sin_alpha;
	}

	// double PurePursuit::lowPassfilter(double angle)
	// {	
	// 	// angle = config_.lowpass_gain * angle + (1 - config_.lowpass_gain) * prev_steering;
	// 	angle = prev_steering + config_.lowpass_gain*(angle - prev_steering);
    // 	return angle;
	// }

	double PurePursuit::lowPassfilter(double gain, double prev_value, double value)
	{	
		// angle = config_.lowpass_gain * angle + (1 - config_.lowpass_gain) * prev_steering;
		value = prev_value + gain*(value - prev_value);
    	return value;
	}
}