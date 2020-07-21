#include "pid_controller.hpp"



namespace platoon_control
{
	PIDController::PIDController(){}
	
	double PIDController::calculate( double setpoint, double pv ){

		// Calculate error
	    double error = setpoint - pv;

	    // Proportional term
	    double Pout = _Kp * error;

	    // Integral term
	    _integral += error * _dt;
		if (_integral > integratorMax) _integral = integratorMax;
		else if (_integral < integratorMin) _integral = integratorMin;
	    double Iout = _Ki * _integral;

	    // Derivative term
	    double derivative = (error - _pre_error) / _dt;
	    double Dout = _Kd * derivative;

	    // Calculate total output
	    double output = Pout + Iout + Dout;

	    // Restrict to max/min
	    if( output > _max )
	        output = _max;
	    else if( output < _min )
	        output = _min;

	    // Save error to previous error
	    _pre_error = error;

	    return output;

	}



    void PIDController::reset() {
        _integral = 0.0;
        _pre_error = 0.0;
    }

}