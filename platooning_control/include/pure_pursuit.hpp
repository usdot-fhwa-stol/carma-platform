#pragma once

#include <ros/ros.h>
#include <math.h> 
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cav_msgs/TrajectoryPlan.h>
#include "platoon_control_config.h"







namespace platoon_control
{


    class PurePursuit
    {


    public:
    	PurePursuit();

    	double calculateSteer(const cav_msgs::TrajectoryPlanPoint& tp);

		// geometry pose
		geometry_msgs::Pose current_pose_;
		double velocity_ = 0.0;

		PlatooningControlPluginConfig config_;

    private:

		// calculate the lookahead distance from next trajectory point
		double getLookaheadDist(const cav_msgs::TrajectoryPlanPoint& tp) const;

		// calculate yaw angle of the vehicle
		double getYaw(const cav_msgs::TrajectoryPlanPoint& tp) const;

		// calculate alpha angle
		double getAlpha(double lookahead, std::vector<double> v1, std::vector<double> v2) const;

		// calculate steering direction
		int getSteeringDirection(std::vector<double> v1, std::vector<double> v2) const;

		// calculate command velocity from trajecoty point
		double getVelocity(const cav_msgs::TrajectoryPlanPoint& tp, double delta_pos) const;

		double getAlphaSin(cav_msgs::TrajectoryPlanPoint tp, geometry_msgs::Pose current_pose);
		double lowPassfilter(double angle);


		double prev_steering = 0.0;
		

		
		// previous trajectory point
		cav_msgs::TrajectoryPlanPoint tp0;

		// helper function (if needed)
		// inline double deg2rad(double deg) const
		// {
		// 	return deg * M_PI / 180;
		// }  // convert degree to radian


    };
}