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


namespace platoon_control_pid0
{
	PIDController::PIDController(std::string name, double deadband, double slope_break, double kp1, double kp2,
								 double ki, double kd, double time_step, double integral_min,
								 double integral_max, double output_min, double output_max) :
								 name_(name), deadband_(deadband), slope_break_(slope_break), kp1_(kp1),
								 kp2_(kp2), ki_(ki), kd_(kd), time_step_(time_step),
								 integral_min_(integral_min), integral_max_(integral_max),
								 output_min_(output_min), output_max_(output_max) {
		
		// Need to ensure that time step > 0 to avoid a division exception
		if (time_step < 0.0001){
			time_step_ = 0.0001;
			ROS_WARN_STREAM("PID Controller creates with time step of " << time_step);
		}

		// Other sanity checks
		if (deadband < 0.0){
			deadband_ = 0.0;
			ROS_WARN_STREAM("PID Controller illegal deadband specified: " << deadband);
		}
		if (slope_break < deadband){
			slope_break_ = deadband_;
			ROS_WARN_STREAM("PID Controller slope_break specified < deadband: " << slope_break);
		}
		if (integral_min >= integral_max){
			integral_max_ = integral_min + 0.00001;
			ROS_WARN_STREAM("PID Controller specified invalid pair of integral limits: " << integral_min << ", " << integral_max);
		}
		if (output_min >= output_max){
			output_max_ = output_min + 0.00001;
			ROS_WARN_STREAM("PID Controller specified invalid pair of output limits: " << output_min << ", " << output_max);
		}

		// Pre-compute intermediate kp information to save cpu cycles later
		double y1 = kp1_*(slope_break_ - deadband_);
		x2_intercept_ = slope_break_ - y1/kp2_;

		ROS_DEBUG_STREAM("PIDController '" << name_ << "' created with:");
		ROS_DEBUG_STREAM("    deadband = " << deadband_);
		ROS_DEBUG_STREAM("    slope_break = " << slope_break_);
		ROS_DEBUG_STREAM("    kp1 = " << kp1_);
		ROS_DEBUG_STREAM("    kp2 = " << kp2_);
		ROS_DEBUG_STREAM("    ki  = " << ki_);
		ROS_DEBUG_STREAM("    kd  = " << kd_);
		ROS_DEBUG_STREAM("    time_step = " << time_step_);
		ROS_DEBUG_STREAM("    integral_min = " << integral_min_);
		ROS_DEBUG_STREAM("    integral max = " << integral_max_);
		ROS_DEBUG_STREAM("    output_min = " << output_min_);
		ROS_DEBUG_STREAM("    output_max = " << output_max_);
	}
	
    void PIDController::reset(){
        integral_ = 0.0;
        prev_error_ = 0.0;
		ROS_DEBUG_STREAM("PID reset: " << name_);
    }

	double PIDController::calculate(const double setpoint, const double pv){

		// Calculate error
	    double error = setpoint - pv;
		ROS_DEBUG_STREAM(name_ << " setpoint = " << setpoint << ", pv = " << pv << ", error = " << error << ", prev error = " << prev_error_);

	    // Proportional term
		double p_out = 0.0;
		if (error > slope_break_){
			p_out = kp2_ * (error - x2_intercept_);
			ROS_DEBUG_STREAM("*** Applied kp2 to error " << error << ", x2_intercept = " << x2_intercept_);
		}else if (error > deadband_){
			p_out = kp1_ * (error - deadband_);
		}else if (error < -slope_break_){
			p_out = kp2_ * (error + x2_intercept_);
			ROS_DEBUG_STREAM("*** Applied kp2 to error " << error << ", x2_intercept = " << x2_intercept_);
		}else if (error < -deadband_){
			p_out = kp1_ * (error + deadband_);
		}

	    // Integral term
	    integral_ += error * time_step_;
		if (integral_ > integral_max_){
			integral_ = integral_max_;
		}else if (integral_ < integral_min_){
			integral_ = integral_min_;
		}
	    double i_out = ki_ * integral_;

	    // Derivative term
	    double derivative = (error - prev_error_) / time_step_;
		double d_out = kd_ * derivative;

	    // Calculate total output
	    double output = p_out + i_out + d_out;
		ROS_DEBUG_STREAM("p_out = " << p_out << ", i_out = " << i_out << ", d_out = " << d_out << ", raw output = " << output);

	    // Restrict to max/min
	    if (output > output_max_){
	        output = output_max_;
		}else if (output < output_min_){
	        output = output_min_;
		}
		ROS_DEBUG_STREAM("Limited output = " << output);

	    // Save error to previous error
	    prev_error_ = error;

	    return output;
	}
}
