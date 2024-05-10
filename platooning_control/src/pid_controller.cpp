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

#include "platoon_control/pid_controller.hpp"

namespace platoon_control
{
    PIDController::PIDController(){}

    double PIDController::calculate( double setpoint, double pv ){
        // Calculate error
	    double error = setpoint - pv;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"),"PID error: " << error);

	    // Proportional term
	    double Pout = config_->kp * error;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Proportional term: " << Pout);

	    // Integral term
	    _integral += error * config_->dt;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"),"Integral term: " << _integral);

		if (_integral > config_->integrator_max){
			 _integral = config_->integrator_max;
		}
		else if (_integral < config_->integrator_min){
			_integral = config_->integrator_min;
		}
	    double Iout = config_->ki * _integral;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Iout: " << Iout);

	    // Derivative term
	    double derivative = (error - _pre_error) / config_->dt;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "derivative term: " << derivative);
	    double Dout = config_->kd * derivative;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Dout: " << Dout);

	    // Calculate total output
	    double output = Pout + Iout + Dout;
		RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "total controller output: " << output);

	    // Restrict to max/min
	    if( output > config_->max_delta_speed_per_timestep )
	        output = config_->max_delta_speed_per_timestep;
	    else if( output < config_->min_delta_speed_per_timestep )
	        output = config_->min_delta_speed_per_timestep;
	    // Save error to previous error
	    _pre_error = error;

	    return output;

    }

	void PIDController::reset() {
        _integral = 0.0;
        _pre_error = 0.0;
    }


}
