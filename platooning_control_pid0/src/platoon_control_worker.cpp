
/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
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

#include <boost/math/constants/constants.hpp>
#include "platoon_control_worker.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>


namespace platoon_control_pid0
{
    const double PI = boost::math::double_constants::pi;
    const double TWO_PI = 2.0*PI;

	PlatoonControlWorker::PlatoonControlWorker() {}


    PlatoonControlWorker::~PlatoonControlWorker() {
        if (pid_h_) {
            delete pid_h_;
        }
        if (pid_c_) {
            delete pid_c_;
        }
    }


    void PlatoonControlWorker::set_config_params(const PlatoonControlPluginConfig config) {

        // Store the params we need for future operations
        time_step_ = config.time_step;
        gamma_h_ = config.gamma_h;
        wheelbase_ = config.wheelbase;
        max_steering_angle_ = config.max_steering_angle;

        // Create the PID controllers
        ROS_DEBUG_STREAM("Creating heading PID");
        pid_h_ = new PIDController(config.pid_h_deadband, config.pid_h_slope_break, config.pid_h_kp1,
                                   config.pid_h_kp2, config.pid_h_ki, config.pid_h_kd, config.time_step,
                                   config.pid_h_integral_min, config.pid_h_integral_max,
                                   -config.max_steering_angle, config.max_steering_angle);
        ROS_DEBUG_STREAM("Creating CTE PID");
        pid_c_ = new PIDController(config.pid_c_deadband, config.pid_c_slope_break, config.pid_c_kp1,
                                   config.pid_c_kp2, config.pid_c_ki, config.pid_c_kd, config.time_step,
                                   config.pid_c_integral_min, config.pid_c_integral_max,
                                   -config.max_steering_angle, config.max_steering_angle);
    }


    void PlatoonControlWorker::set_current_pose(const geometry_msgs::PoseStamped p) {
		host_x_ = p.pose.position.x;
        host_y_ = p.pose.position.y;

        // Extract the vehicle's heading
        tf2::Quaternion q(p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w);
        tf2::Matrix3x3 mat(q);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);
        host_heading_ = normalize_yaw(yaw);
	}


    void PlatoonControlWorker::set_current_speed(const double speed) {
        current_speed_ = speed;
    }


    void PlatoonControlWorker::set_trajectory(const cav_msgs::TrajectoryPlan::ConstPtr& tp) {
        traj_ = tp->trajectory_points;
    }


    void PlatoonControlWorker::set_lead_info(const DynamicLeaderInfo& dl, const double tgt_gap, const double act_gap) {
        leader_ = dl;
        desired_gap_ = tgt_gap;
        actual_gap_ = act_gap;
        ROS_DEBUG_STREAM("desired gap = " << desired_gap_ << ", actual gap = " << actual_gap_);
    }


    void PlatoonControlWorker::generate_control_signal() {

        // Find the trajectory point nearest the vehicle but in front of it
        // TODO: may need to consider choosing a point a little farther forward (there may be multiple points
        //       between rear axle (vehicle origin) and front bumper!)
        find_nearest_point();
        ROS_DEBUG_STREAM("Nearest forward tp = " << tp_index_);


        //---------- Speed command

        // CAUTION: a vehicle running this code can only be a solo vehicle or platoon leader!
        ROS_WARN_STREAM("///// SPEED CONTROL FOR SOLO/LEAD VEHICLE ONLY!");

        // For now, just drive the speeds indicated by the trajectory, i.e. no gap control.  Once the
        // plugin is basically proven out, this section can be enhanced for platoon gap control.

        // Get the distance between current closest trajectory points
        size_t tpi = tp_index_;
        if (tpi == 0) {
            tpi = 1;
        }
        double x0 = traj_[tpi-1].x;
        double y0 = traj_[tpi-1].y;
        double x1 = traj_[tpi].x;
        double y1 = traj_[tpi].y;
        double dx = x0 - x1;
        double dy = y0 - y1;
        double tp_dist = std::sqrt(dx*dx + dy*dy);

        // Find the average speed implied by the target times on each point
        ros::Duration time_diff = traj_[tpi].target_time - traj_[tpi-1].target_time;
        double delta_time = time_diff.toSec();
        speed_cmd_ = tp_dist / (delta_time + 0.0001);
        ROS_DEBUG_STREAM("speed cmd = " << speed_cmd_ << ", tp_dist = " << tp_dist << ", delta_time = " << delta_time);


        //---------- Steering command

        // Find the heading of the next downtrack trajectory point
        double tp_heading = normalize_yaw(traj_[tp_index_].yaw);

        // Calculate the heading error (desired - actual), as a delta angle, accounting for the possibility
        // of crossing over the zero-heading cardinal direction
        double heading_error = subtract_headings(tp_heading, host_heading_);

        // Pass heading error to the heading PID
        double out_h = pid_h_->calculate(0.0, heading_error);

        // Find the cross-track error (lateral diff between vehicle position and nearest point
        // on the trajectory, which may be different from the global CTE normally discussed in
        // larger Carma contexts, as that typically references to the roadway centerline, not the
        // planned trajectory).
        double cte = calculate_cross_track();

        // Pass CTE to the CTE PID
        double out_c = pid_c_->calculate(0.0, cte);

        // Combine the PID outputs according to the mixing factor
        steering_cmd_ = gamma_h_*out_h + (1.0 - gamma_h_)*out_c;
        if (steering_cmd_ < -max_steering_angle_) {
            steering_cmd_ = -max_steering_angle_;
        }else if (steering_cmd_ > max_steering_angle_) {
            steering_cmd_ = max_steering_angle_;
        }
        ROS_DEBUG_STREAM("heading_error = " << heading_error << ", out_h = " << out_h << ", cte = " << cte << ", out_c = " << out_c);
        ROS_DEBUG_STREAM("steering cmd = " << steering_cmd_ << ", gamma = " << gamma_h_);


        //---------- Angular velocity command

        // Note that pure pursuit calculates its radius (i.e. kappa, which is 1/radius) based on the desired
        // trajectory.  It is independent of the steering command.  We could do a similar thing here,
        // computing radius of the trajectory arc in its next 3 points from 
        // https://stackoverflow.com/questions/22791951/algorithm-to-find-an-arc-its-center-radius-and-angles-given-3-points.
        // But it doesn't seem that is appropriate.  Rather, for controller to work correctly, angular rate cmd
        // should be intimately connected to the steering angle cmd. Therefore...

        // Radius of the immediate future trajectory of the vehicle (regardless of the path that was laid out) is,
        // from bicycle model, using the rear axle as vehicle reference point, as described in
        // https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357.
        double radius = 1.0e9;
        if (abs(steering_cmd_) > 1.0e-9) {
            radius = wheelbase_ / std::tan(steering_cmd_);
        }
        angular_vel_cmd_ = speed_cmd_ / radius;
        ROS_DEBUG_STREAM("ang vel cmd = " << angular_vel_cmd_ << "radius = " << radius << ", wheelbase = " << wheelbase_);
    }


    double PlatoonControlWorker::get_speed_cmd() {
        return speed_cmd_;
    }


    double PlatoonControlWorker::get_steering_cmd() {
        return steering_cmd_;
    }


    double PlatoonControlWorker::get_angular_vel_cmd() {
        return angular_vel_cmd_;
    }


    size_t PlatoonControlWorker::get_tp_index() {
        return tp_index_;
    }

    ////////////////// UNIT TEST SUPPORT ////////////////// TODO - redo all this

    void PlatoonControlWorker::unit_test_set_pose(const double x, const double y, const double heading) {
        host_x_ = x;
        host_y_ = y;
        host_heading_ = heading;
    }

    void PlatoonControlWorker::unit_test_set_traj(const std::vector<cav_msgs::TrajectoryPlanPoint> tr) {
        traj_ = tr;
    }

    double PlatoonControlWorker::unit_test_get_traj_px(const size_t index) {
        return traj_[index].x;
    }

    double PlatoonControlWorker::unit_test_get_traj_py(const size_t index) {
        return traj_[index].y;
    }



    ////////////////// PRIVATE METHODS ////////////////////

    void PlatoonControlWorker::find_nearest_point() {

        // Initialize default result value to first trajectory point
        size_t res = 0;
        double best_dist = 99999.0; //meters

        // Build a vector that represents the vehicle's direction of travel
        double vhx = std::cos(host_heading_);
        double vhy = std::sin(host_heading_); 

        // Loop through all points
        for (size_t index = 0;  index < traj_.size();  ++index) {

            // Calculate dist between this point and the vehicle
            double dx = host_x_ - traj_[index].x;
            double dy = host_y_ - traj_[index].y;
            double dist = std::sqrt(dx*dx + dy*dy);

            // If this distance is smaller than the best so far then
            if (dist < best_dist) {

                // Build a vector from the vehicle to this trajectory point
                double vtx = traj_[index].x - host_x_;
                double vty = traj_[index].y - host_y_;

                // Find the dot product between vehicle heading vector and this vector
                double dot = vhx*vtx + vhy*vty;

                // If dot product >= 0 then the traj point is in front of the vehicle, so save it as best so far
                if (dot >= 0.0) {
                    res = index;
                    best_dist = dist;
                }
            }
        }

        tp_index_ = res;
    }


    double PlatoonControlWorker::subtract_headings(const double h1, const double h2) {

        // Subtract the values, then wrap around the allowed output range if needed
        double res = h1 - h2;
        if (res <= -PI) {
            res += TWO_PI;
        }else if (res > PI) {
            res -= TWO_PI;
        }
        return res;
    }


    double PlatoonControlWorker::calculate_cross_track() {

        //CAUTION:  ASSUMES that find_nearest_point() has been called previously, to store an
        //          accurate value of tp_index_.

        // Find the perpendicular distance between the trajectory segment preceding the nearest
        // point and the vehicle.  There may be some cases where the closest approach is on the
        // previous or next segment and is slightly closer than this calc will give, but that
        // would only happen if there is a large heading error, which is much more problematic
        // than the slight inaccuracy of this assumption.

        // Create a vector for the line (define an infinitely long line from the two given points)
        int id = tp_index_;
        if (id == 0) {
            ++id;
        }
        double p1x = traj_[id-1].x;
        double p1y = traj_[id-1].y;
        double p2x = traj_[id].x;
        double p2y = traj_[id].y;
        double lvx = p2x - p1x;
        double lvy = p2y - p1y;

        // Create a vector between base of line and the vehicle
        double vvx = host_x_ - p1x;
        double vvy = host_y_ - p1y;

        // Calculate the dot product between the two vectors (ignore the magnitude)
        double dot = lvx*vvx + lvy*vvy;

        // Find a point on the extended line where the perpendicular crosses
        double mag2 = lvx*lvx + lvy*lvy;
        double b = dot / mag2; //safe since trajectory points should never be colocated
        double pbx = p1x + b*lvx;
        double pby = p1y + b*lvy;

        // Calculate the distance between this pb point and the vehicle (always positive)
        double dx = pbx - host_x_;
        double dy = pby - host_y_;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Determine which side of the trajectory the vehicle is on; sign of this result
        // is the sign of the cross-track error (positive is left of trajectory)
        double side = vvy*lvx - vvx*lvy;
        if (side < 0.0) {
            dist = -dist;
        }

        return dist;
    }


    double PlatoonControlWorker::normalize_yaw(const double yaw) {
        double res = yaw;

        if (yaw < 0.0) {
            do {
                res += TWO_PI;
            }while (res < 0.0);
        }else if (yaw >= TWO_PI) {
            do {
                res -= TWO_PI;
            }while (res >= TWO_PI);
        }
        return res;
    }
}
