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

/**
 * \brief Stuct containing the algorithm configuration values for the LightControlledIntersectionTacticalPlugin
 */
struct LightControlledIntersectionTacticalPluginConfig
{
    double trajectory_time_length = 6.0;     // Trajectory length in seconds
    double curve_resample_step_size = 1.0;   // Curve re-sampling step size in m
    bool publish_debug = false;
    double centerline_sampling_spacing = 1.0;  // The gap in meters between points sampled from the lanelet centerlines for planning trajectory positions
    int curvature_moving_average_window_size = 9;  // Size of the window used in the moving average filter to smooth the curvature profile
                                           // computed curvature and output speeds
    double lateral_accel_limit = 2.5;        // Maximum allowable lateral acceleration m/s^2   
    int speed_moving_average_window_size = 5;      // Size of the window used in the moving average filter to smooth both the speed profile 
    double back_distance = 20;               // Number of meters behind the first maneuver that need to be included in points for curvature calculation                                       

     //! The maximum allowable vehicle deceleration limit in m/s
    double vehicle_decel_limit = 2.0;

    //! A multiplier to apply to the maximum allowable vehicle deceleration limit so we plan under our capabilities
    double vehicle_decel_limit_multiplier = 0.75;

    //! The maximum allowable vehicle acceleration limit in m/s
    double vehicle_accel_limit = 2.0;

    //! A multiplier to apply to the maximum allowable vehicle acceleration limit so we plan under our capabilities
    double vehicle_accel_limit_multiplier = 0.75;

    //! A buffer infront of the stopping location which will still be considered a valid stop
    double stop_line_buffer = 2.0;

    //! The minimum period in seconds which a maneuver plan must cover if the plugin wishes to control the whole plan
    double min_maneuver_planning_period = 15.1;

    // Double: Approximate update time interval of carma streets
    double delta_t = 1.0;
    
    friend std::ostream& operator<<(std::ostream& output, const LightControlledIntersectionTacticalPluginConfig& c)
    {
        output <<"LightControlledIntersectionTacticalPluginConfig { " <<std::endl
                << "trajectory_time_length: " << c.trajectory_time_length << std::endl
                << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
                << "centerline_sampling_spacing: " << c.centerline_sampling_spacing << std::endl
                << "curvature_moving_average_window_size: " << c.curvature_moving_average_window_size << std::endl
                << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
                << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
                << "vehicle_decel_limit: " << c.vehicle_decel_limit << std::endl
                << "vehicle_decel_limit_multiplier: " << c.vehicle_decel_limit_multiplier << std::endl
                << "vehicle_accel_limit: " << c.vehicle_accel_limit << std::endl
                << "vehicle_accel_limit_multiplier: " << c.vehicle_accel_limit_multiplier << std::endl
                << "stop_line_buffer: " << c.stop_line_buffer << std::endl
                << "}" << std::endl;
        return output;
    }
};