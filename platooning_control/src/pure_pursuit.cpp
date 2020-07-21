#include "pure_pursuit.hpp"



namespace platoon_control
{
	PurePursuit::PurePursuit(){}


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
	geometry_msgs::Point PurePursuit::calcRelativeCoordinate(const geometry_msgs::Point& point_msg, const geometry_msgs::Pose& current_pose) const
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
	double PurePursuit::getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2) const
	{
  		tf::Vector3 v1 = point2vector(target1);
  		v1.setZ(0);
  		tf::Vector3 v2 = point2vector(target2);
  		v2.setZ(0);
  		return tf::tfDistance(v1, v2);
	}

	
	tf::Vector3 PurePursuit::point2vector(geometry_msgs::Point point) const
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


	bool PurePursuit::canGetCurvature(double* output_kappa)
{
		// search next waypoint
		getNextWaypoint();
		if (next_point_number_ == -1)
		{
			ROS_INFO("lost next waypoint");
			return false;
		}
		// check whether curvature is valid or not
		bool is_valid_curve = false;
		for (const auto& el : current_traj_points_)
		{	
			geometry_msgs::Point el_pose;
			el_pose.x = el.x;
			el_pose.y = el.y;
			if (getPlaneDistance(el_pose, current_pose_.position) > minimum_lookahead_distance_)
			{
			is_valid_curve = true;
			break;
			}
		}
		if (!is_valid_curve)
		{
			return false;
		}
		// if is_linear_interpolation_ is false or next waypoint is first or last
		if (!is_linear_interpolation_ || next_point_number_ == 0 ||
			next_point_number_ == (static_cast<int>(current_traj_points_.size() - 1)))
		{
			next_target_position_.x = current_traj_points_.at(next_point_number_).x;
			next_target_position_.y = current_traj_points_.at(next_point_number_).y;
			*output_kappa = calcCurvature(next_target_position_);
			return true;
		}

		// linear interpolation and calculate angular velocity
		bool interpolation =
			interpolateNextTarget(next_point_number_, &next_target_position_);

		if (!interpolation)
		{
			ROS_INFO_STREAM("lost target! ");
			return false;
		}

		// ROS_INFO("next_target : ( %lf , %lf , %lf)",
		//  next_target.x, next_target.y,next_target.z);

		*output_kappa = calcCurvature(next_target_position_);
		return true;
	}


	// linear interpolation of next target
	bool PurePursuit::interpolateNextTarget(int next_point, geometry_msgs::Point* next_target) const
	{
		constexpr double ERROR = pow(10, -5);  // 0.00001

		int path_size = static_cast<int>(current_traj_points_.size());
		if (next_point == path_size - 1)
		{
			*next_target = trajPointToPoint(current_traj_points_.at(next_point));
			// *next_target->x = current_traj_points_.at(next_point).x;
			// *next_target->y = current_traj_points_.at(next_point).y;
			// *next_target->z = current_pose_.position.z;
			return true;
		}
		double search_radius = lookahead_distance_;
		geometry_msgs::Point zero_p;
		geometry_msgs::Point end;
		end.x = current_traj_points_.at(next_point).x;
		end.y = current_traj_points_.at(next_point).y;
		geometry_msgs::Point start;
		start.x = current_traj_points_.at(next_point - 1).x;
		start.y = current_traj_points_.at(next_point - 1).y;

		// let the linear equation be "ax + by + c = 0"
		// if there are two points (x1,y1) , (x2,y2),
		// a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
		double a = 0;
		double b = 0;
		double c = 0;
		double get_linear_flag = getLinearEquation(start, end, &a, &b, &c);
		if (!get_linear_flag)
			return false;

		// let the center of circle be "(x0,y0)", in my code ,
		// the center of circle is vehicle position
		// the distance  "d" between the foot of
		// a perpendicular line and the center of circle is ...
		//    | a * x0 + b * y0 + c |
		// d = -------------------------------
		//          √( a~2 + b~2)
		double d = getDistanceBetweenLineAndPoint(current_pose_.position, a, b, c);

		// ROS_INFO("a : %lf ", a);
		// ROS_INFO("b : %lf ", b);
		// ROS_INFO("c : %lf ", c);
		// ROS_INFO("distance : %lf ", d);

		if (d > search_radius)
			return false;

		// unit vector of point 'start' to point 'end'
		tf::Vector3 v((end.x - start.x), (end.y - start.y), 0);
		tf::Vector3 unit_v = v.normalize();

		// normal unit vectors of v
		// rotate to counter clockwise 90 degree
		tf::Vector3 unit_w1 = rotateUnitVector(unit_v, 90);
		// rotate to counter clockwise 90 degree
		tf::Vector3 unit_w2 = rotateUnitVector(unit_v, -90);

		// the foot of a perpendicular line
		geometry_msgs::Point h1;
		h1.x = current_pose_.position.x + d * unit_w1.getX();
		h1.y = current_pose_.position.y + d * unit_w1.getY();
		h1.z = current_pose_.position.z;

		geometry_msgs::Point h2;
		h2.x = current_pose_.position.x + d * unit_w2.getX();
		h2.y = current_pose_.position.y + d * unit_w2.getY();
		h2.z = current_pose_.position.z;

		// ROS_INFO("error : %lf", error);
		// ROS_INFO("whether h1 on line : %lf", h1.y - (slope * h1.x + intercept));
		// ROS_INFO("whether h2 on line : %lf", h2.y - (slope * h2.x + intercept));

		// check which of two foot of a perpendicular line is on the line equation
		geometry_msgs::Point h;
		if (fabs(a * h1.x + b * h1.y + c) < ERROR)
		{
			h = h1;
			//   ROS_INFO("use h1");
		}
		else if (fabs(a * h2.x + b * h2.y + c) < ERROR)
		{
			//   ROS_INFO("use h2");
			h = h2;
		}
		else
		{
			return false;
		}

		// get intersection[s]
		// if there is a intersection
		if (d == search_radius)
		{
			*next_target = h;
			return true;
		}
		else
		{
		// if there are two intersection
		// get intersection in front of vehicle
		double s = sqrt(pow(search_radius, 2) - pow(d, 2));
		geometry_msgs::Point target1;
		target1.x = h.x + s * unit_v.getX();
		target1.y = h.y + s * unit_v.getY();
		target1.z = current_pose_.position.z;

		geometry_msgs::Point target2;
		target2.x = h.x - s * unit_v.getX();
		target2.y = h.y - s * unit_v.getY();
		target2.z = current_pose_.position.z;

		// ROS_INFO("target1 : ( %lf , %lf , %lf)", target1.x, target1.y, target1.z);
		// ROS_INFO("target2 : ( %lf , %lf , %lf)", target2.x, target2.y, target2.z);
		// displayLinePoint(a, b, c, target1, target2, h);  // debug tool

		// check intersection is between end and start
		double interval = getPlaneDistance(end, start);
		if (getPlaneDistance(target1, end) < interval)
		{
			// ROS_INFO("result : target1");
			*next_target = target1;
			return true;
		}
		else if (getPlaneDistance(target2, end) < interval)
		{
			// ROS_INFO("result : target2");
			*next_target = target2;
			return true;
		}
		else
		{
		// ROS_INFO("result : false ");
			return false;
		}
		}
	}


	geometry_msgs::Point PurePursuit::trajPointToPoint(cav_msgs::TrajectoryPlanPoint tp) const {
		geometry_msgs::Point output;
		output.x = tp.x;
		output.y = tp.y;
		output.z = current_pose_.position.z;
		return output;
	}

	double PurePursuit::getDistanceBetweenLineAndPoint(geometry_msgs::Point point, double a, double b, double c) const
	{
		double d = fabs(a * point.x + b * point.y + c) / sqrt(pow(a, 2) + pow(b, 2));

		return d;
	}


	// let the linear equation be "ax + by + c = 0"
	// if there are two points (x1,y1) , (x2,y2), a = "y2-y1, b = "(-1) * x2 - x1" ,c = "(-1) * (y2-y1)x1 + (x2-x1)y1"
	bool PurePursuit::getLinearEquation(geometry_msgs::Point start, geometry_msgs::Point end, double *a, double *b, double *c) const
	{
		//(x1, y1) = (start.x, star.y), (x2, y2) = (end.x, end.y)
		double sub_x = fabs(start.x - end.x);
		double sub_y = fabs(start.y - end.y);
		double error = pow(10, -5);  // 0.00001

		if (sub_x < error && sub_y < error)
		{
			ROS_INFO("two points are the same point!!");
			return false;
		}

		*a = end.y - start.y;
		*b = (-1) * (end.x - start.x);
		*c = (-1) * (end.y - start.y) * start.x + (end.x - start.x) * start.y;

		return true;
	}


	tf::Vector3 PurePursuit::rotateUnitVector(tf::Vector3 unit_vector, double degree) const
	{
		tf::Vector3 w1(cos(deg2rad(degree)) * unit_vector.getX() - sin(deg2rad(degree)) * unit_vector.getY(),
						sin(deg2rad(degree)) * unit_vector.getX() + cos(deg2rad(degree)) * unit_vector.getY(), 0);
		tf::Vector3 unit_w1 = w1.normalize();

		return unit_w1;
	}

	

}