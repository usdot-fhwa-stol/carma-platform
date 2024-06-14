
/*------------------------------------------------------------------------------
* Copyright (C) 2024 LEIDOS.
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

#include "platooning_control/platooning_control_worker.hpp"


namespace platooning_control
{

    PlatooningControlWorker::PlatooningControlWorker()
    {
        pid_ctrl_ = PIDController();

    }

    double PlatooningControlWorker::get_last_speed_command() const {
        return speedCmd_;
    }

    void PlatooningControlWorker::generate_speed(const carma_planning_msgs::msg::TrajectoryPlanPoint& point)
    {
        double speed_cmd = 0;

        if (!last_cmd_set_)
        {
            // last speed command for smooth speed transition
            lastCmdSpeed = currentSpeed;
            last_cmd_set_ = true;
        }

        PlatoonLeaderInfo leader = platoon_leader;
    	if(!leader.staticId.empty() && leader.staticId != ctrl_config_->vehicle_id)
        {
            double controllerOutput = 0.0;


	        double leaderCurrentPosition = leader.vehiclePosition;
	        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "The current leader position is " << leaderCurrentPosition);

            double desiredHostPosition = leaderCurrentPosition - desired_gap_;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "desiredHostPosition = " << desiredHostPosition);

            double hostVehiclePosition = leaderCurrentPosition - actual_gap_;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "hostVehiclePosition = " << hostVehiclePosition);

	        controllerOutput = pid_ctrl_.calculate(desiredHostPosition, hostVehiclePosition);

		    double adjSpeedCmd = controllerOutput + leader.commandSpeed;
	        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "Adjusted Speed Cmd = " << adjSpeedCmd << "; Controller Output = " << controllerOutput
	        	<< "; Leader CmdSpeed= " << leader.commandSpeed << "; Adjustment Cap " << ctrl_config_->adjustment_cap_mps);
	            // After we get a adjSpeedCmd, we apply three filters on it if the filter is enabled
	            // First: we do not allow the difference between command speed of the host vehicle and the leader's commandSpeed higher than adjustmentCap

            speed_cmd = adjSpeedCmd;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "A speed command is generated from command generator: " << speed_cmd << " m/s");

            if(ctrl_config_->enable_max_adjustment_filter)
            {
                if(speed_cmd > leader.commandSpeed + ctrl_config_->adjustment_cap_mps) {
                    speed_cmd = leader.commandSpeed + ctrl_config_->adjustment_cap_mps;
                } else if(speed_cmd < leader.commandSpeed - ctrl_config_->adjustment_cap_mps) {
                    speed_cmd = leader.commandSpeed - ctrl_config_->adjustment_cap_mps;
                }
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "The adjusted cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }

        }

        else if (leader.staticId == ctrl_config_->vehicle_id)
        {
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "Host vehicle is the leader");
            speed_cmd = currentSpeed;

            if(ctrl_config_->enable_max_adjustment_filter)
            {
                if(speed_cmd > ctrl_config_->adjustment_cap_mps)
                {
                    speed_cmd = ctrl_config_->adjustment_cap_mps;
                }

                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "The adjusted leader cmd speed after max adjustment cap is " << speed_cmd << " m/s");
            }

            pid_ctrl_.reset();
        }

        else
        {
            // If there is no leader available, the vehicle will stop. This means there is a mis-communication between platooning strategic and control plugins.
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "There is no leader available");
            speed_cmd = 0.0;
            pid_ctrl_.reset();
        }



        // Third: we allow do not a large gap between two consecutive speed commands
        if(ctrl_config_->enable_max_accel_filter) {

                double max = lastCmdSpeed + (ctrl_config_->max_accel_mps2 * (ctrl_config_->cmd_timestamp_ms / 1000.0));
                double min = lastCmdSpeed - (ctrl_config_->max_accel_mps2 * (ctrl_config_->cmd_timestamp_ms / 1000.0));
                if(speed_cmd > max) {
                    speed_cmd = max;
                } else if (speed_cmd < min) {
                    speed_cmd = min;
                }
                lastCmdSpeed = speed_cmd;
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platooning_control"), "The speed command after max accel cap is: " << speed_cmd << " m/s");
        }

        speedCmd_ = speed_cmd;

        lastCmdSpeed = speedCmd_;

    }

    // TODO get the actual leader from strategic plugin
    void PlatooningControlWorker::set_leader(const PlatoonLeaderInfo& leader){
    	platoon_leader = leader;
    }

    void  PlatooningControlWorker::set_current_speed(double speed){
    	currentSpeed = speed;

    }
}