#include "pure_pursuit.hpp"



namespace platoon_control
{
	PurePursuit::PurePursuit():
	RADIUS_MAX_(9e10), 
	KAPPA_MIN_(1 / RADIUS_MAX_), 
	is_linear_interpolation_(false), 
  	next_point_number_(-1), 
  	lookahead_distance_(0), 
  	minimum_lookahead_distance_(6), 
  	current_linear_velocity_(0)
	{}


	double PurePursuit::calcCurvature(geometry_msgs::Point target)
	{
		double kappa;
	  	double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
	  	double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

	  	if (denominator != 0)
	  	{
	    	kappa = numerator / denominator;
	  	}
		else
		  {
		    if (numerator > 0)
		    {
		      kappa = KAPPA_MIN_;
		    }
		    else
		    {
		      kappa = -KAPPA_MIN_;
		    }
		  }
		  ROS_INFO("kappa : %lf", kappa);
		  return kappa;
	}

	// calculation relative coordinate of point from current_pose frame
	geometry_msgs::Point PurePursuit::calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
	{
		tf::Transform inverse;
		tf::poseMsgToTF(current_pose, inverse);
		tf::Transform transform = inverse.inverse();

		tf::Point p;
		pointMsgToTF(point_msg, p);
		tf::Point tf_p = transform * p;
		geometry_msgs::Point tf_point_msg;
		pointTFToMsg(tf_p, tf_point_msg);

		return tf_point_msg;
	}

	// distance between target 1 and target2 in 2-D
	double PurePursuit::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
	{
  		tf::Vector3 v1 = point2vector(target1);
  		v1.setZ(0);
  		tf::Vector3 v2 = point2vector(target2);
  		v2.setZ(0);
  		return tf::tfDistance(v1, v2);
	}

	
	tf::Vector3 PurePursuit::point2vector(geometry_msgs::Point point)
	{
  		tf::Vector3 vector(point.x, point.y, point.z);
  		return vector;
	}

	void PurePursuit::getNextWaypoint()
	{
  		int path_size = static_cast<int>(current_traj_points_.size());

  		// if waypoints are not given, do nothing.
  		if (path_size == 0)
  		{
   	 		next_point_number_ = -1;
    		return;
  		}

  		// look for the next waypoint.
  		for (int i = 0; i < path_size; i++)
  		{
    		// if search waypoint is the last
    		if (i == (path_size - 1))
    		{
      			ROS_INFO("search trajectory point is the last");
      			next_point_number_ = i;
      			return;
    		}

    		geometry_msgs::Point traj_point;
    		traj_point.x = current_traj_points_.at(i).x;
    		traj_point.y = current_traj_points_.at(i).y;
    		traj_point.z = 0.0;

    		// 	if there exists an effective waypoint
	    	if (getPlaneDistance(traj_point, current_pose_.position) > lookahead_distance_)
    		{
      			next_point_number_ = i;
      			return;
    		}
  		}

 	 	// if this program reaches here , it means we lost the waypoint!
  		next_point_number_ = -1;
  		return;
	}







	double PurePursuit::apply(){

		double angularVelocity = 0;

		geometry_msgs::PoseStamped pose;
		double _velocity;

		double lookAheadDistance = getLookAheadDistance(pose);
        double lookAheadAngle = getLookAheadAngle(pose);

        if (std::abs(std::sin(lookAheadAngle)) >= _epsilon) {
        	double radius = 0.5*(lookAheadDistance/std::sin(lookAheadAngle));

          	double linearVelocity = _velocity;
          	if (std::abs(radius) >= _epsilon)
          		angularVelocity = linearVelocity/radius; 
        }
 		
 		return angularVelocity;

	}


	double PurePursuit::getLookAheadDistance(const geometry_msgs::PoseStamped& pose) const
	{    
	    return 0.0;
  	}
  
  	double PurePursuit::getLookAheadAngle(const geometry_msgs::PoseStamped& pose) const 
  	{
  		return 0.0;
  	}
  
  	double PurePursuit::getLookAheadThreshold() const {
    	return _lookAheadRatio*_currentVelocity;
  	}

  	double PurePursuit::getArcDistance(const geometry_msgs::PoseStamped& pose) const {
	    double lookAheadDistance = getLookAheadDistance(pose);
	    double lookAheadAngle = getLookAheadAngle(pose);

	    if (std::abs(std::sin(lookAheadAngle)) >= _epsilon)
	    	return lookAheadDistance/sin(lookAheadAngle)*lookAheadAngle;
	    else
	      	return lookAheadDistance;
  	}

}