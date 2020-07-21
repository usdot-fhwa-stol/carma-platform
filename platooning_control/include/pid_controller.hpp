#pragma once

#include <ros/ros.h>





namespace platoon_control
{


    class PIDController
    {
    public:

    	PIDController();
    	
    	// ~PIDController();

        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        // PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double setpoint, double pv );
        // ~PID();

        void reset();

        




    private:

    	double _dt = 1.0;
        double _max = 1.0;
        double _min = 1.0;
        double _Kp = 1.0;
        double _Kd = 1.0;
        double _Ki = 1.0;
        double _pre_error = 1.0;
        double _integral = 1.0;

        double integratorMax;
        double integratorMin;



    	
    

    

    };
}