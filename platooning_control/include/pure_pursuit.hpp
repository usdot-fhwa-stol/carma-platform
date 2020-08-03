#pragma once

#include <ros/ros.h>
#include <math.h> 
#include <iostream>
// #include <Eigen/Core>
// #include <Eigen/LU>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cav_msgs/TrajectoryPlan.h>







namespace platoon_control
{


    class PurePursuit
    {


    public:
    	PurePursuit();

    	double calculateSteer(cav_msgs::TrajectoryPlanPoint tp);

		// geometry pose
		geometry_msgs::Pose current_pose_;

    private:

		// calculate the lookahead distance from next trajectory point
		double getLookaheadDist(cav_msgs::TrajectoryPlanPoint tp);

		// calculate yaw angle of the vehicle
		double getYaw();

		// calculate alpha angle
		double getAlpha(double lookahead, std::vector<double> v1, std::vector<double> v2);

		// calculate steering direction
		int getSteeringDirection(std::vector<double> v1, std::vector<double> v2);

		// calculate command velocity from trajecoty point
		double getVelocity(cav_msgs::TrajectoryPlanPoint tp, double delta_pos);
		
		// vehicle wheel base
		double wheelbase_ = 2.7;

		// coefficient for smooth steering
		double Kdd_ = 4.5;

		
		// previous trajectory point
		cav_msgs::TrajectoryPlanPoint tp0;

		// helper function (if needed)
		// inline double deg2rad(double deg) const
		// {
		// 	return deg * M_PI / 180;
		// }  // convert degree to radian


    };
}