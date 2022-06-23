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

#include "pid_controller.h"


namespace platoon_control_ihp
{
	PIDController::PIDController(){}
	
	double PIDController::calculate( double setpoint, double pv )
	{
		// Calculate error
	    double error = setpoint - pv;
		ROS_DEBUG_STREAM("PID error: " << error);

	    // Proportional term
	    double Pout = config_.Kp * error;
		ROS_DEBUG_STREAM("Proportional term: " << Pout);

	    // Integral term
	    _integral += error * config_.dt;
		ROS_DEBUG_STREAM("Integral term: " << _integral);

		if (_integral > config_.integratorMax){
			 _integral = config_.integratorMax;
		}
		else if (_integral < config_.integratorMin){
			_integral = config_.integratorMin;
		}
	    double Iout = config_.Ki * _integral;
		ROS_DEBUG_STREAM("Iout: " << Iout);

	    // Derivative term
	    double derivative = (error - _pre_error) / config_.dt;
		ROS_DEBUG_STREAM("derivative term: " << derivative);
	    double Dout = config_.Kd * derivative;
		ROS_DEBUG_STREAM("Dout: " << Dout);

	    // Calculate total output
	    double output = Pout + Iout + Dout;
		ROS_DEBUG_STREAM("total controller output: " << output);

	    // Restrict to max/min
	    if( output > config_.maxValue )
	        output = config_.maxValue;
	    else if( output < config_.minValue )
	        output = config_.minValue;
	    // Save error to previous error
	    _pre_error = error;

	    return output;

	}



    void PIDController::reset() 
	{
        _integral = 0.0;
        _pre_error = 0.0;
    }

}
