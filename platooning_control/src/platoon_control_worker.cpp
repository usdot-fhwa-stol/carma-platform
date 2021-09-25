
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

#include "platoon_control_worker.h"


namespace platoon_control
{
   
	PlatoonControlWorker::PlatoonControlWorker()
    {
        pid_ctrl_ = PIDController();
        pp_ = PurePursuit();

    }

    void PlatoonControlWorker::updateConfigParams(PlatooningControlPluginConfig new_config)
    {
        ctrl_config_ = new_config;
        pid_ctrl_.config_ = new_config;
        pp_.config_ = new_config;
    }

	double PlatoonControlWorker::getLastSpeedCommand() const {
        return speedCmd_;
    }

    void PlatoonControlWorker::setCurrentPose(const geometry_msgs::PoseStamped msg)
	{
		current_pose_ = msg.pose;
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
