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
		bool canGetCurvature(double* output_kappa);
		// for setting data
		void setLookaheadDistance(const double& ld)
		{
			lookahead_distance_ = ld;
		}

		void setMinimumLookaheadDistance(const double& minld)
		{
			minimum_lookahead_distance_ = minld;
		}

		bool interpolateNextTarget(int next_point, geometry_msgs::Point* next_target) const;

    private:

    	// constant
  		const double RADIUS_MAX_ = 9e10;
  		const double KAPPA_MIN_ = 1/RADIUS_MAX_;


  		  // variables
		bool is_linear_interpolation_ = false;
		int next_point_number_ = -1;
		geometry_msgs::Point next_target_position_;
		double lookahead_distance_ = 0.0;
		double minimum_lookahead_distance_ = 6.0;
		geometry_msgs::Pose current_pose_;
		double current_linear_velocity_ = 0.0;
    	std::vector<cav_msgs::TrajectoryPlanPoint> current_traj_points_;

		geometry_msgs::Point trajPointToPoint(cav_msgs::TrajectoryPlanPoint tp) const;
    	void getNextWaypoint();

		double getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c) const;

		bool getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c) const;

    	geometry_msgs::Point calcRelativeCoordinate(const geometry_msgs::Point& point_msg, const geometry_msgs::Pose& current_pose) const;

    	double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2) const;

    	tf::Vector3 point2vector(geometry_msgs::Point point) const;

		tf::Vector3 rotateUnitVector(tf::Vector3 unit_vector, double degree) const;

		// inline function (less than 10 lines )
		inline double kmph2mps(double velocity_kmph) 
		{
			return (velocity_kmph * 1000) / (60 * 60);
		}
		inline double mps2kmph(double velocity_mps)
		{
			return (velocity_mps * 60 * 60) / 1000;
		}
		inline double deg2rad(double deg) const
		{
			return deg * M_PI / 180;
		}  // convert degree to radian


    };
}