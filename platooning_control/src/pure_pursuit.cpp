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

	double PurePursuit::calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp){

		double lookahead = getLookaheadDist(tp);
		ROS_DEBUG_STREAM("calculated lookahead: " << lookahead);
		double alpha = getAlpha(tp, current_pose_);
		ROS_DEBUG_STREAM("calculated alpha: " << alpha);
		double kappa = 2 * (alpha)/(lookahead);
		ROS_DEBUG_STREAM("calculated kappa: " << kappa);
		double steering = atan((2 * config_.wheelbase * (alpha))/(lookahead));
		ROS_DEBUG_STREAM("calculated steering: " << steering);
		double filtered_steering = lowPassfilter(steering);
		ROS_DEBUG_STREAM("filtered steering: " << filtered_steering);
		if (std::isnan(filtered_steering)) return prev_steering;
		prev_steering = filtered_steering;
		// return filtered_steering;
		return kappa;
	}

	double PurePursuit::getAlpha(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose)
	{
		// double dot = tp.x*current_pose.position.x + tp.y*current_pose.position.y;
    	// double det = tp.x*tp.y*current_pose.position.y - tp.y*tp.y*current_pose.position.x;
    	// double alpha = atan2(det, dot);
		// return alpha;
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

	double PurePursuit::lowPassfilter(double angle)
	{	
		// angle = config_.lowpass_gain * angle + (1 - config_.lowpass_gain) * prev_steering;
		angle = prev_steering + config_.lowpass_gain*(angle - prev_steering);
    	return angle;
	}
}