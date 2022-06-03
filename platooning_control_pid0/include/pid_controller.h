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

#include <ros/ros.h>
#include "platoon_control_config.h"


namespace platoon_control_pid0
{

    /** Defines a PID controller that has a possible deadband for the proportional gain
     *  and up to two values of proportional gain that apply at different error values
     *  outside of the deadband.  Integral & derivative gains are uniformly applied
     *  for any error value.
     */
    class PIDController
    {
    public:

        /**
         * \brief Constructor that takes all operating constants
         * \param deadband      abs value of process value deadband, within which proportional gain will be zero
         * \param slope_break   abs value of breakpoint in proportional gain where it changes from kp1 to kp2
         * \param kp1           lower proportional gain, applies to pv between deadband_width and slope_break
         * \param kp2           upper porportional gain, applies to pv > slope_break
         * \param ki            integral gain
         * \param kd            derivative gain
         * \param time_step     duration of the (uniform) time interval between calls to calculate(), sec
         * \param integral_min  min value that the integral term is allowed to achieve
         * \param integral_max  max value that the integral term is allowed to achieve
         * \param output_min    min limit of output value
         * \param output_max    max limit of output value
         */
    	PIDController(double deadband, double slope_break, double kp1, double kp2,
					  double ki, double kd, double time_step, double integral_min,
					  double integral_max, double output_min, double output_max);

        /**
         * \brief Clears the running history of the controller.
         */
        void reset();

        /**
         * \brief Returns the manipulated variable given a setpoint and current process value.
         * \param setpoint The desired process value (the target)
         * \param pv The current actual process value being controlled
         */
        double calculate(const double setpoint, const double pv);


    private:
        // values set in constructor (act as constants) - see constructor brief for description of each of these similarly named vars
        double                          deadband_;          
        double                          slope_break_;       
        double                          kp1_;               
        double                          kp2_;               
        double                          ki_;                
        double                          kd_;                
        double                          time_step_;         
        double                          integral_min_;      
        double                          integral_max_;      
        double                          output_min_;        
        double                          output_max_;        

        // historical values used in internal calcs
        double                          prev_error_ = 0.0;  //error from previous iteration
        double                          integral_ = 0.0;    //integrated error experience thus far
    };
}
