#pragma once

/*
 * Copyright (C) 2022 LEIDOS.
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
    int default_downsample_ratio = 36;  // Amount to downsample input lanelet centerline data. Value corresponds to saving each nth point.
    int turn_downsample_ratio = 20;  // Amount to downsample input lanelet centerline data on turns. Value corresponds to saving each nth point.
    double trajectory_time_length = 6.0;     // Trajectory length in seconds
    double curve_resample_step_size = 1.0;   // Curve re-sampling step size in m
    int curvature_moving_average_window_size = 9;  // Size of the window used in the moving average filter to smooth the curvature profile
                                           // computed curvature and output speeds
    double lateral_accel_limit = 2.5;        // Maximum allowable lateral acceleration m/s^2   
    double lat_accel_multiplier = 0.50; // Multiplier of lat_accel to bring the value under lat_accel
    int speed_moving_average_window_size = 5;      // Size of the window used in the moving average filter to smooth both the speed profile 
    double back_distance = 20;               // Number of meters behind the first maneuver that need to be included in points for curvature calculation                                       
    double buffer_ending_downtrack = 20.0; // Additional distance beyond ending downtrack to ensure sufficient points
     //! The maximum allowable vehicle deceleration limit in m/s
    double vehicle_decel_limit = 2.0;
    double minimum_speed =  2.2352; // minimum allowable speed in m/s

    //! A multiplier to apply to the maximum allowable vehicle deceleration limit so we plan under our capabilities
    double vehicle_decel_limit_multiplier = 0.75;

    //! The maximum allowable vehicle acceleration limit in m/s
    double vehicle_accel_limit = 2.0;

    //! A multiplier to apply to the maximum allowable vehicle acceleration limit so we plan under our capabilities
    double vehicle_accel_limit_multiplier = 0.75;

    //! A buffer infront of the stopping location which will still be considered a valid stop
    double stop_line_buffer = 2.0;

    //! Distance from the nearest traffic light where the vehicle decides whether to run last successful trajectory or accept new one (in meters)
    double algorithm_evaluation_distance = 35.0;

    //! Period if scheduled entry time is within which the vehicle decides to run the last successful trajectory smoothing trajectory (in seconds)
    double algorithm_evaluation_period = 4.5;
    
    friend std::ostream& operator<<(std::ostream& output, const LightControlledIntersectionTacticalPluginConfig& c)
    {
        output <<"LightControlledIntersectionTacticalPluginConfig { " <<std::endl
                << "trajectory_time_length: " << c.trajectory_time_length << std::endl
                << "curve_resample_step_size: " << c.curve_resample_step_size << std::endl
                << "default_downsample_ratio: " << c.default_downsample_ratio << std::endl
                << "turn_downsample_ratio: " << c.turn_downsample_ratio << std::endl
                << "curvature_moving_average_window_size: " << c.curvature_moving_average_window_size << std::endl
                << "lateral_accel_limit: " << c.lateral_accel_limit << std::endl
                << "lat_accel_multiplier: " << c.lat_accel_multiplier << std::endl
                << "speed_moving_average_window_size: " << c.speed_moving_average_window_size << std::endl
                << "back_distance: " << c.back_distance << std::endl
                << "minimum_speed: " << c.minimum_speed << std::endl
                << "buffer_ending_downtrack: " << c.buffer_ending_downtrack << std::endl
                << "vehicle_decel_limit: " << c.vehicle_decel_limit << std::endl
                << "vehicle_decel_limit_multiplier: " << c.vehicle_decel_limit_multiplier << std::endl
                << "vehicle_accel_limit: " << c.vehicle_accel_limit << std::endl
                << "vehicle_accel_limit_multiplier: " << c.vehicle_accel_limit_multiplier << std::endl
                << "stop_line_buffer: " << c.stop_line_buffer << std::endl
                << "algorithm_evaluation_distance: " << c.algorithm_evaluation_distance << std::endl
                << "algorithm_evaluation_period: " << c.algorithm_evaluation_period << std::endl
                << "}" << std::endl;
        return output;
    }
};