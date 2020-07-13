#pragma once

#include <ros/ros.h>
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

    	double calcCurvature(geometry_msgs::Point target);


        double apply();
    private:

    	// constant
  		const double RADIUS_MAX_;
  		const double KAPPA_MIN_;

  		  // variables
		bool is_linear_interpolation_;
		int next_point_number_;
		geometry_msgs::Point next_target_position_;
		double lookahead_distance_;
		double minimum_lookahead_distance_;
		geometry_msgs::Pose current_pose_;
		double current_linear_velocity_;
    	std::vector<cav_msgs::TrajectoryPlanPoint> current_traj_points_;


    	void getNextWaypoint();

    	geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);

    	double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);

    	tf::Vector3 point2vector(geometry_msgs::Point point);







    	double getLookAheadDistance(const geometry_msgs::PoseStamped& pose) const;

    	double getLookAheadAngle(const geometry_msgs::PoseStamped& pose) const;

    	double getLookAheadThreshold() const;

    	double getArcDistance(const geometry_msgs::PoseStamped& pose) const;

    	double getArcDistance();

    	double _epsilon = 1e-6;

    	double _lookAheadRatio;
    	double _currentVelocity;


    };
}