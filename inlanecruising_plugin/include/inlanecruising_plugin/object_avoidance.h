#pragma once

/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include <vector>
#include <cmath>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/TrajectoryPlanPoint.h>
#include <cav_msgs/Plugin.h>
#include <trajectory_utils/trajectory_utils.h>
#include <carma_utils/CARMAUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cav_srvs/PlanManeuvers.h>
#include <cav_srvs/PlanTrajectory.h>

#include <carma_wm/WMListener.h>
#include <carma_wm/WorldModel.h>
#include <carma_wm/collision_detection.h>
#include <trajectory_utils/quintic_coefficient_calculator.h>



namespace inlanecruising_plugin
{
namespace object_avoidance
{

    class ObjectAvoidance
    {
        public:
        ObjectAvoidance();


        geometry_msgs::Vector3 host_vehicle_size;

        cav_msgs::TrajectoryPlan update_traj_for_object(const cav_msgs::TrajectoryPlan& original_tp, const carma_wm::WorldModelConstPtr& wm_, double current_speed_);

        // calculate the quintic polynomial with coefficients
        double polynomial_calc(std::vector<double> coeff, double x);
        
        // calculate the derivative of quintic polynomial with coefficients
        double polynomial_calc_d(std::vector<double> coeff, double x);

        // finds maximum speed in a set of trajectory points
        double max_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points);

        // calculates distance between trajectory points in a plan
        std::vector<double> get_relative_downtracks(const cav_msgs::TrajectoryPlan& trajectory_plan);

        


        private:
        
        // time horizon for collision avoidance in s
        const double collision_horizon_ = 10.0;
        // Minimum speed for moving obstacle in m/s
        const double min_obstacle_speed_ = 2.0;
        // Adjustment factor for safe and comfortable acceleration/deceleration 
        const double acceleration_adjustment_factor_ = 4.0;
        // current vehicle speed in m/s
        double current_speed_;
        // minimum planning time in s
        const double tpmin_ = 2;
        // max deceleration value in m/s^2
        const double maximum_deceleration_value_ = 3.0;
        // minimum safety gap in m
        const double x_gap_ = 2.0;

    };

};  // namespace object_avoidance
};  // namespace inlanecruising_plugin