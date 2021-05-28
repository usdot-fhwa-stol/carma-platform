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

	double PurePursuit::getVelocity(const cav_msgs::TrajectoryPlanPoint& tp, double delta_pos) const {
		// ros::Duration delta_t = tp.target_time - ros::Time::now();
		// double delta_t_second = fabs(delta_t.toSec());

		// if(delta_t_second != 0) {
		// 	return delta_pos / delta_t_second;
		// }
		return 0.0;
	}

	double PurePursuit::getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const{
		double yaw = atan2(tp.y - current_pose_.position.y, tp.x - current_pose_.position.x);
		return yaw;
	}

	double PurePursuit::getAlpha(double lookahead, std::vector<double> v1, std::vector<double> v2) const {
		
		double inner_prod = v1[0]*v2[0] + v1[1]*v2[1];
		double value = inner_prod/lookahead;
		if (value > 1) value = 1;
		else if (value < -1) value = -1;
		double alpha = acos(value);
        return alpha;
	}

	int PurePursuit::getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const{
		double corss_prod = v1[0]*v2[1] - v1[1]*v2[0];
        if (corss_prod >= 0.0){
			 return -1;
		}
        return 1;
	}

	double PurePursuit::calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp){
		// skip the first trajectory point
		// if (tp0.x == 0 && tp0.y == 0){
		// 	tp0 = tp;
		// 	return 0.0;
		// }
		double lookahead = getLookaheadDist(tp);
		double v = getVelocity(tp, lookahead);
		double yaw = 0;//getYaw(tp);
		ROS_DEBUG_STREAM("calculated yaw: " << yaw);
		std::vector<double> v1 = {tp.x - current_pose_.position.x , tp.y - current_pose_.position.y};
		std::vector<double> v2 = {cos(yaw), sin(yaw)};
		double alpha = getYaw(tp) - yaw;//getAlpha(lookahead, v1, v2);
		double alphaSin = getAlphaSin(tp, current_pose_);
		ROS_DEBUG_STREAM("alphaSin: " << alphaSin);
		int direction = getSteeringDirection(v1, v2);
		double steering = atan((2 * config_.wheelbase * alphaSin)/(lookahead));// change (lookahead) to (Kdd_*v) if steering is bad
		ROS_DEBUG_STREAM("calculated steering: " << steering);
		tp0 = tp;
		if (std::isnan(steering)) return prev_steering;
		prev_steering = steering;
		return steering;
	}

	double PurePursuit::getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose)
	{
		geometry_msgs::Point point_msg;
		point_msg.x = tp.x;
		point_msg.y = tp.y;
		point_msg.z = 0;

		tf::Transform inverse;
		tf::poseMsgToTF(current_pose, inverse);

		tf::Point p;
		tf::pointMsgToTF(point_msg, p);
		tf::Point tf_p = inverse * p;
		geometry_msgs::Point tf_point_msg;
		tf::pointTFToMsg(tf_p, tf_point_msg);
		return tf_point_msg.y;
	}
}