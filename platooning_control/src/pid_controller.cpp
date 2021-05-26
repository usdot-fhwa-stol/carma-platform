#include "pid_controller.hpp"



namespace platoon_control
{
	PIDController::PIDController(){}
	
	double PIDController::calculate( double setpoint, double pv ){

		// Calculate error
	    double error = setpoint - pv;

	    // Proportional term
	    double Pout = config_.Kp * error;

	    // Integral term
	    _integral += error * config_.dt;
		if (_integral > config_.integratorMax){
			 _integral = config_.integratorMax;
		}
		else if (_integral < config_.integratorMin){
			_integral = config_.integratorMin;
		}
	    double Iout = config_.Ki * _integral;

	    // Derivative term
	    double derivative = (error - _pre_error) / config_.dt;
	    double Dout = config_.Kd * derivative;

	    // Calculate total output
	    double output = Pout + Iout + Dout;

	    // Restrict to max/min
	    if( output > config_.max_value )
	        output = config_.max_value;
	    else if( output < config_.min_value )
	        output = config_.min_value;

	    // Save error to previous error
	    _pre_error = error;

	    return output;

	}



    void PIDController::reset() {
        _integral = 0.0;
        _pre_error = 0.0;
    }

}