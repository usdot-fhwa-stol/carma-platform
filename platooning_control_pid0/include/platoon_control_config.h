#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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
 */

#include <iostream>

namespace platoon_control_pid0 {

    /**
     * \brief Stuct containing the algorithm configuration values for the PlatooningControlPlugin
     */
    struct PlatoonControlPluginConfig {

        // TODO: the PID-related variables may need to come from vehicle-specific configs, rather than package params

        double  pid_h_deadband = 0.0;
        double  pid_h_slope_break = 0.0;
        double  pid_h_kp1 = -0.1;
        double  pid_h_kp2 = -0.1;
        double  pid_h_ki = 0.0;
        double  pid_h_kd = 0.0;
        double  pid_h_integral_min = -1.0;
        double  pid_h_integral_max = 1.0;

        double  pid_c_deadband = 0.0;
        double  pid_c_slope_break = 0.0;
        double  pid_c_kp1 = 0.001;
        double  pid_c_kp2 = 0.001;
        double  pid_c_ki = 0.0;
        double  pid_c_kd = 0.0;
        double  pid_c_integral_min = -1.0;
        double  pid_c_integral_max = 1.0;

        double  time_step = 0.03333;
        double  gamma_h = 1.0;
        double  max_steering_angle = 1.05;

        //TODO: which of these do we still need?
        double  timeHeadway = 6.0;
        double  standStillHeadway = 12.0;
        double  maxAccel = 2.5;
        int     cmdTmestamp = 100;
        double  adjustmentCap = 10;
        double  wheelBase = 3.09;
        double  lowpassGain = 0.5;
        double  lookaheadRatio = 2.0;
        double  minLookaheadDist = 6.0;
        double  Kdd = 4.5; // coeficient for smooth steering TODO: need this?

        std::string vehicleID = "DEFAULT_VEHICLE_ID"; //TODO get the real thing, if we even need it

        friend std::ostream &operator<<(std::ostream &output, const PlatooningControlPluginConfig &c)
        {
            output << "PlatoonControlPluginConfig { " << std::endl
//JOHN





                << "shutdown_timeout: " << c.shutdown_timeout << std::endl




                << "timeHeadway: " << c.timeHeadway << std::endl
                << "standStillHeadway: " << c.standStillHeadway << std::endl
                << "maxAccel: " << c.maxAccel << std::endl
                << "cmdTmestamp: " << c.cmdTmestamp << std::endl
                << "adjustmentCap: " << c.adjustmentCap << std::endl
                << "wheelBase: " << c.wheelBase << std::endl
                << "lowpassGain: " << c.lowpassGain << std::endl
                << "lookaheadRatio: " << c.lookaheadRatio << std::endl
                << "minLookaheadDist: " << c.minLookaheadDist << std::endl
                << "Kdd: " << c.Kdd << std::endl
                << "}" << std::endl;
            return output;
        }
    };
}