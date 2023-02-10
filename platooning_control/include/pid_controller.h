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


namespace platoon_control
{

    /**
    * \brief This class includes logic for PID controller. PID controller is used in this plugin to maintain the inter-vehicle gap by adjusting the speed.
    */

    class PIDController
    {
    public:

        /**
        * \brief Constructor for the PID controller class
        */
    	PIDController();

        /**
        * \brief plugin config object
        */
        PlatooningControlPluginConfig config_;
    	
    	// ~PIDController();

        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        // PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        /**
        * \brief function to calculate control command based on setpoint and process vale
        * \param setpoint desired value
        * \param pv current value
        * \return the manipulated variable given a setpoint and current process value
        */
        double calculate( double setpoint, double pv );
        // ~PID();

        void reset();

        




    private:

        double _pre_error = 0.0;
        double _integral = 0.0;

    };
}
