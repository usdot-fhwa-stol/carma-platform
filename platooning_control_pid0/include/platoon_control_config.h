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
        double  max_accel = 2.5;
        double  speed_adjustment_cap = 10.0;

        // The following come from global vehicle params, not the local param file
        std::string vehicle_id = "DEFAULT_VEHICLE_ID";
        double  wheelbase = 3.09;
        int     shutdown_timeout = 400;
        int     ignore_initial_inputs = 0;

        friend std::ostream &operator<<(std::ostream &output, const PlatoonControlPluginConfig &c)
        {
            output << "PlatoonControlPluginConfig { " << std::endl
                << "pid_h_deadband:     " << c.pid_h_deadband       << std::endl
                << "pid_h_slope_break:  " << c.pid_h_slope_break    << std::endl
                << "pid_h_kp1:          " << c.pid_h_kp1            << std::endl
                << "pid_h_kp2:          " << c.pid_h_kp2            << std::endl
                << "pid_h_ki:           " << c.pid_h_ki             << std::endl
                << "pid_h_kd:           " << c.pid_h_kd             << std::endl
                << "pid_h_integral_min: " << c.pid_h_integral_min   << std::endl
                << "pid_h_integral_max: " << c.pid_h_integral_max   << std::endl
                << "pid_c_deadband      " << c.pid_c_deadband       << std::endl
                << "pid_c_slope_break:  " << c.pid_c_slope_break    << std::endl
                << "pid_c_kp1:          " << c.pid_c_kp1            << std::endl
                << "pid_c_kp2:          " << c.pid_c_kp2            << std::endl
                << "pid_c_ki:           " << c.pid_c_ki             << std::endl
                << "pid_c_kd:           " << c.pid_c_kd             << std::endl
                << "pid_c_integral_min: " << c.pid_c_integral_min   << std::endl
                << "pid_c_integral_max: " << c.pid_c_integral_max   << std::endl
                << "time_step:          " << c.time_step            << std::endl
                << "gamma_h:            " << c.gamma_h              << std::endl
                << "max_steering_angle: " << c.max_steering_angle   << std::endl
                << "max_accel:          " << c.max_accel            << std::endl
                << "speed_adjustment_cap:" << c.speed_adjustment_cap    << std::endl
                << "From global vehicle config:"                    << std::endl
                << "vehicle_id:         " << c.vehicle_id           << std::endl
                << "wheelbase:          " << c.wheelbase            << std::endl
                << "shutdown_timeout:   " << c.shutdown_timeout     << std::endl
                << "ignore_initial_inputs:" << c.ignore_initial_inputs  << std::endl
                << "}" << std::endl;
            return output;
        }
    };
}