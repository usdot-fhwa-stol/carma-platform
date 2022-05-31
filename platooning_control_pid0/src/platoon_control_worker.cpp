
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

#include <boost/math.h>
#include "platoon_control_worker.h"


namespace platoon_control_pid0
{
    const double PI = boost::math::constants::pi<double>();
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


    void PlatoonControlWorker::set_config_params(PlatooningControlPluginConfig config) {

        // Store the params we need for future operations
        time_step_ = config.time_step;






        // Create the PID controllers
        pid_h_ = new PIDController(config.pid_h_deadband, config.pid_h_slope_break, config.pid_h_kp1,
                                   config.pid_h_kp2, config.pid_h_ki, config.pid_h_kd, config.time_step,
                                   config.pid_h_integral_min, config.pid_h_integral_max,
                                   -config.max_steering_angle, config.max_steering_angle);
        pid_c_ = new PIDController(config.pid_c_deadband, config.pid_c_slope_break, config.pid_c_kp1,
                                   config.pid_c_kp2, config.pid_c_ki, config.pid_c_kd, config.time_step,
                                   config.pid_c_integral_min, config.pid_c_integral_max,
                                   -config.max_steering_angle, config.max_steering_angle);
    }


    void PlatoonControlWorker::set_current_pose(const geometry_msgs::PoseStamped msg) {
		current_pose_ = msg.pose;
	}


    void PlatoonControlWorker::set_current_speed(const double speed) {
        current_speed_ = speed;
    }


    void PlatoonControlWorker::set_trajectory(const cav_msgs::TrajectoryPlan::ConstPtr& tp) {
        traj_ = tp->trajectory_points;
    }


    void void PlatoonControlWorker::set_platoon_info(const std::string leader_id, const double leader_loc, const double leader_spd_cmd,
                                                     const int leader_pos, const int host_pos) {
        leader_loc_ = leader_loc;
        leader_spd_cmd_ = leader_spd_cmd;
        leader_pos_ = leader_pos;
        host_pos_ = host_pos;
    }


    double PlatoonControlWorker::get_speed_cmd() {

        //TODO fill in







    }


    double PlatoonControlWorker::get_angular_velocity_cmd() {

        //TODO fill in




    }


    double PlatoonControlWorker::get_steering_cmd() {

        // Find the trajectory point nearest the vehicle but in front of it
        // TODO: may need to consider choosing a point a little farther forward (there may be multiple points
        //       between rear axle (vehicle origin) and front bumper!)
        find_nearest_point();

        // Find the heading of that trajectory point
        double tp_heading = traj_[tp_index_].yaw;

        // Calculate the heading error (desired - actual), as a delta angle, accounting for the possibility
        // of crossing over the zero-heading cardinal direction
        double heading_error = subtract_headings(tp_heading, vehicle_heading);

        // Pass trajectory heading and vehicle heading to the heading PID
        double out_h = pid_h_->calculate(0.0, heading_error);

        // Find the cross-track error (lateral diff between vehicle position and nearest point
        // on the trajectory, which may be different from the global CTE normally discussed in
        // larger Carma contexts, as that typcially references to the roadway centerline, not the
        // planned trajectory).
        double cte = calculate_cross_track();

        // Pass CTE to the CTE PID
        double out_c = pid_c_->calculate(0.0, cte);

        // Combine the PID outputs according to the mixing factor
        double out = gamma_h_*out_h + (1.0 - gamma_h_)*out_c;
    }


    ////////////////// PRIVATE METHODS ////////////////////

    int PlatoonControlWorker::find_nearest_point() {

        // Initialize default return value to first trajectory point
        int res = 0;

        // Build a vector that represents the vehicle's direction of travel

        // Loop through all points

            // Calculate dist between this point and the vehicle

            // If this distance is smaller than the best so far then

                // Build a vector from the vehicle to this trajectory point

                // Find the dot product between vehicle heading vector and this vector

                // If dot product >= 0 then the traj point is in front of the vehicle, so save it

        return res;
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

        // Find the perpendicluar distance between the trajectory segment preceding the nearest
        // point and the vehicle.  There may be some cases where the closest approach is on the
        // previous or next segment and is slightly closer than this calc will give, but that
        // would only happen if there is a large heading error, which is much more problematic
        // than the slight inaccuracy of this assumption.

        // Create a vector for the line (define an infintely long line from the two given points)
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
        double vvx = current_pose_.x - p1x;
        double vvy = current_pose_.y - p1y;

        // Calculate the dot product between the two vectors (ignore the magnitude)
        double dot = lvx*vvx + lvy*vvy;

        // Find a point on the extended line where the perpendicular crosses
        double mag2 = lvx*lvx + lvy*lvy;
        double b = dot / mag2; //safe since trajectory points should never be colocated
        double pbx = p1x + b*lvx;
        double pby = p1y + b*lvy;

        // Calculate the distance between this pb point and the vehicle (always positive)
        double dx = pbx - vvx;
        double dy = pby - vvy;
        double dist = std::sqrt(dx*dx + dy*dy);

        // Determine which side of the trajectory the vehicle is on; sign of this result
        // is the sign of the cross-track error (positive is left of trajectory)
        double side = vvy*lvx - vvx*lvy;
        if (side < 0.0) {
            dist = -dist;
        }

        return dist;
    }








    //JOHN all below may be throwaway

	double PlatoonControlWorker::getLastSpeedCommand() const {
        return speedCmd_;
    }

    void PlatoonControlWorker::generateSpeed(const cav_msgs::TrajectoryPlanPoint& point)
    {
        double speed_cmd = 0;

        if (!last_cmd_set_)
        {
            // last speed command for smooth speed transition
            lastCmdSpeed = currentSpeed;
            last_cmd_set_ = true;
        }

        PlatoonLeaderInfo leader = platoon_leader;
    	if(leader.staticId != "" && leader.staticId != ctrl_config_.vehicleID)
        {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG_STREAM("The current leader position is " << leaderCurrentPosition);

            double desiredHostPosition = leaderCurrentPosition - desired_gap_;
            ROS_DEBUG_STREAM("desiredHostPosition = " << desiredHostPosition);

            double hostVehiclePosition = leaderCurrentPosition - actual_gap_;
            ROS_DEBUG_STREAM("hostVehiclePosition = " << hostVehiclePosition);
	        
	        controllerOutput = pid_ctrl_.calculate(desiredHostPosition, hostVehiclePosition);

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        ROS_DEBUG_STREAM("Adjusted Speed Cmd = " << adjSpeedCmd << "; Controller Output = " << controllerOutput
	        	<< "; Leader CmdSpeed= " << leader.commandSpeed << "; Adjustment Cap " << ctrl_config_.adjustmentCap);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap
	        
            speed_cmd = adjSpeedCmd;
            ROS_DEBUG_STREAM("A speed command is generated from command generator: " << speed_cmd << " m/s");

            if(enableMaxAdjustmentFilter)
            {
                if(speed_cmd > leader.commandSpeed + ctrl_config_.adjustmentCap) {
                    speed_cmd = leader.commandSpeed + ctrl_config_.adjustmentCap;
                } else if(speed_cmd < leader.commandSpeed - ctrl_config_.adjustmentCap) {
                    speed_cmd = leader.commandSpeed - ctrl_config_.adjustmentCap;
                }
                ROS_DEBUG_STREAM("The adjusted cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }

        }

        else if (leader.staticId == ctrl_config_.vehicleID)
        {
            ROS_DEBUG_STREAM("Host vehicle is the leader");
            speed_cmd = currentSpeed;

            if(enableMaxAdjustmentFilter) 
            {
                if(speed_cmd > ctrl_config_.adjustmentCap)
                {
                    speed_cmd = ctrl_config_.adjustmentCap;
                } 

                ROS_DEBUG_STREAM("The adjusted leader cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }   

            pid_ctrl_.reset();
        }

        else 
        {
            // If there is no leader available, the vehicle will stop. This means there is a mis-communication between platooning strategic and control plugins.
            ROS_DEBUG_STREAM("There is no leader available");
            speed_cmd = 0.0;
            pid_ctrl_.reset();
        }

        
        
        // Third: we allow do not a large gap between two consecutive speed commands
        if(enableMaxAccelFilter) {
                
                double max = lastCmdSpeed + (ctrl_config_.maxAccel * (ctrl_config_.cmdTmestamp / 1000.0));
                double min = lastCmdSpeed - (ctrl_config_.maxAccel * (ctrl_config_.cmdTmestamp / 1000.0));
                if(speed_cmd > max) {
                    speed_cmd = max; 
                } else if (speed_cmd < min) {
                    speed_cmd = min;
                }
                lastCmdSpeed = speed_cmd;
                ROS_DEBUG_STREAM("The speed command after max accel cap is: " << speed_cmd << " m/s");
        }

        speedCmd_ = speed_cmd;

        lastCmdSpeed = speedCmd_;

    }

    void PlatoonControlWorker::generateSteer(const cav_msgs::TrajectoryPlanPoint& point)
    {
        pp_.current_pose_ = current_pose_;
        pp_.velocity_ = currentSpeed;

        pp_.calculateSteer(point);
    	steerCmd_ = pp_.getSteeringAngle(); 
        angVelCmd_ = pp_.getAngularVelocity();
    }

    // TODO get the actual leader from strategic plugin
    void PlatoonControlWorker::setLeader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatoonControlWorker::setCurrentSpeed(double speed){
    	currentSpeed = speed;
        
    }




}
