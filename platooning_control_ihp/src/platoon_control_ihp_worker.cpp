
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

#include "platoon_control_ihp_worker.h"


namespace platoon_control_ihp
{
   
	PlatoonControlIHPWorker::PlatoonControlIHPWorker()
    {
        pid_ctrl_ = PIDController();
        pp_ = PurePursuit();
    }

    void PlatoonControlIHPWorker::updateConfigParams(PlatooningControlIHPPluginConfig new_config)
    {
        ctrl_config_ = new_config;
        pid_ctrl_.config_ = new_config;
        pp_.config_ = new_config;
    }

	double PlatoonControlIHPWorker::getLastSpeedCommand() const 
    {
        return speedCmd_;
    }

    void PlatoonControlIHPWorker::setCurrentPose(const geometry_msgs::PoseStamped msg)
	{
		current_pose_ = msg.pose;
	}

    double PlatoonControlIHPWorker::getIHPTargetPositionFollower(double leaderCurrentPosition)
    {
        /**
         * Calculate desired position based on previous vehicle's trajectory for followers.
         * 
         * TODO: The platoon trajectory regulation is derived with the assumption that all vehicle 
         *       have identical length (i.e., 5m). Future development is needed to include variable 
         *       vehicle length into consideration.
         */


        // 1. find the summation of "veh_len/veh_speed" for all predecessors
        double tmp_time_hdw = current_predecessor_time_headway_sum_;

        // 2. read host veh and front veh info 
        // Predecessor vehicle data.
        double pred_spd = predecessor_speed_;       // m/s 
        double pred_pos = predecessor_position_;    // m
        
        // host data. 
        double ego_spd = currentSpeed;                          // m/s
        double ego_pos = leaderCurrentPosition - actual_gap_;   // m
        
        // platoon position index
        int pos_idx = host_platoon_position_;

        double desirePlatoonGap = ctrl_config_.intra_tau;       //s
        
        // IHP desired position calculation methods
        double pos_g;      // desired downtrack position calculated with time-gap, in m.
        double pos_h;      // desired downtrack position calculated with distance headaway, in m.

        // 3. IHP gap regualtion 
        // intermediate variables 
        double timeGapAndStepRatio = desirePlatoonGap/ctrl_config_.time_step;     // The ratio between desired platoon time gap and the current time_step.
        double totalTimeGap = desirePlatoonGap*pos_idx;                           // The overall time gap from host vehicle to the platoon leader, in s.

        // calcilate pos_gap and pos_headway
        if ((pred_spd <= ego_spd) && (ego_spd <= ctrl_config_.ss_theta))
        {
            // pos_g 
            pos_g = (pred_pos - ctrl_config_.vehicleLength - ctrl_config_.standstill + ego_pos*timeGapAndStepRatio) / (1 + timeGapAndStepRatio);
            // pos_h
            double pos_h_nom = (pred_pos - ctrl_config_.standstill + ego_pos*(totalTimeGap + tmp_time_hdw)/ctrl_config_.time_step);
            double pos_h_denom = (1 + ((totalTimeGap + tmp_time_hdw)/ctrl_config_.time_step));
            pos_h = pos_h_nom/pos_h_denom;

        }
        else
        {   
            // pos_g 
            pos_g = (pred_pos - ctrl_config_.vehicleLength + ego_pos*(timeGapAndStepRatio)) / (1 + timeGapAndStepRatio);
            // pos_h
            double pos_h_nom = (pred_pos + ego_pos*(totalTimeGap + tmp_time_hdw)/ctrl_config_.time_step);
            double pos_h_denom = (1 + ((totalTimeGap + tmp_time_hdw)/ctrl_config_.time_step));
            pos_h = pos_h_nom/pos_h_denom;
        }

        // desire speed and desire location 
        double pos_des = ctrl_config_.gap_weight*pos_g + (1.0 - ctrl_config_.gap_weight)*pos_h;
        // double des_spd = (pos_des - ego_pos) / ctrl_config_.time_step;

        // return IHP calculated desired location
        return pos_des;
    }
    

    void PlatoonControlIHPWorker::generateSpeed(const cav_msgs::TrajectoryPlanPoint& point)
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

            // --------- Calculate desired gap ---------
	        double leaderCurrentPosition = leader.vehiclePosition;
	        ROS_DEBUG_STREAM("The current leader position is " << leaderCurrentPosition);
            
            double desiredHostPosition = leaderCurrentPosition - desired_gap_;
            ROS_DEBUG_STREAM("desiredHostPosition = " << desiredHostPosition);

            // --------- conisder use IHP here instead to regualte gap ----------
            // Call IHP gap regulation function to re-calculate desired Host position based on entire platoon's info
            double desiredHostPosition_IHP = getIHPTargetPositionFollower(leaderCurrentPosition);
            double hostVehiclePosition = leaderCurrentPosition - actual_gap_;
            ROS_DEBUG_STREAM("hostVehiclePosition = " << hostVehiclePosition);

	        // UCLA: Replace desiredPosition with desiredPosition_IHP here.
            controllerOutput = pid_ctrl_.calculate(desiredHostPosition_IHP, hostVehiclePosition);

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
        if(enableMaxAccelFilter) 
        {        
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

    void PlatoonControlIHPWorker::generateSteer(const cav_msgs::TrajectoryPlanPoint& point)
    {
        pp_.current_pose_ = current_pose_;
        pp_.velocity_ = currentSpeed;

        pp_.calculateSteer(point);
    	steerCmd_ = pp_.getSteeringAngle(); 
        angVelCmd_ = pp_.getAngularVelocity();
    }

    // TODO get the actual leader from strategic plugin
    void PlatoonControlIHPWorker::setLeader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatoonControlIHPWorker::setCurrentSpeed(double speed){
    	currentSpeed = speed;
        
    }




}
