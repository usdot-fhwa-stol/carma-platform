/*
 * Copyright (C) 2019 LEIDOS.
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

#include "arbitrator.hpp"
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanManeuvers.h>
#include "arbitrator_utils.hpp"
#include <ros/ros.h>
#include <exception>

namespace arbitrator
{
    void Arbitrator::run()
    {
        while (!ros::isShuttingDown())
        {
            ros::spinOnce();
            switch (sm_.get_state()) 
            {
                case INITIAL:
                    initial_state();
                    break;
                case PLANNING:
                    planning_state();
                    break;
                case WAITING:
                    waiting_state();
                    break;
                case PAUSED:
                    paused_state();
                    break;
                default:
                    throw new std::invalid_argument("State machine attempting to process an illegal state value");
            }
        }
    }
    
    void Arbitrator::guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg) 
    {
        switch (msg->state)
        {
            case cav_msgs::GuidanceState::STARTUP:
                // NO-OP
                break;
            case cav_msgs::GuidanceState::DRIVERS_READY:
                // NO-OP
                break;
            case cav_msgs::GuidanceState::ACTIVE:
                // NO-OP
                break;
            case cav_msgs::GuidanceState::ENGAGED:
                if (sm_.get_state() == ArbitratorState::INITIAL) {
                    sm_.submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
                } else if (sm_.get_state() == ArbitratorState::PAUSED) {
                    sm_.submit_event(ArbitratorEvent::ARBITRATOR_RESUMED);
                }
                break;
            case cav_msgs::GuidanceState::INACTIVE:
                sm_.submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
                break;
            case cav_msgs::GuidanceState::SHUTDOWN:
                sm_.submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED);
                break;
            default:
                break;
        }
    }

    void Arbitrator::initial_state()
    {
        final_plan_pub_ = nh_.advertise<cav_msgs::ManeuverPlan>("final_maneuver_plan", 5);
        guidance_state_sub_ = nh_.subscribe<cav_msgs::GuidanceState>("guidance_state", 5, &Arbitrator::guidance_state_cb, this);
        // TODO: load plan duration from parameters file
    }

    void Arbitrator::planning_state()
    {
        ros::Time planning_process_start = ros::Time::now();
        cav_msgs::ManeuverPlan plan = planning_strategy_.generate_plan();
        ros::Time plan_end_time = get_plan_end_time(plan);
        ros::Time plan_start_time = get_plan_start_time(plan);
        ros::Duration plan_duration = plan_end_time - plan_start_time;

        if (plan_duration < min_plan_duration_) 
        {
            ROS_FATAL_STREAM("Unable to generate a plan to minimum plan duration!");
            throw std::runtime_error("Unable to generate plan to minimum plan duration");
        } 
        else 
        {
            ROS_INFO_STREAM("Publishing plan " << plan.maneuver_plan_id << " of duration " << plan_duration << " as current maneuver plan");
            final_plan_pub_.publish(plan);
        }

        next_planning_process_start_ = planning_process_start + time_between_plans_;
    }

    void Arbitrator::waiting_state()
    {
        // Sleep in 100ms increments until our next planning cycle
        // This ensures we spin() at least a few times
        while (ros::Time::now() < next_planning_process_start_)
        {
            ros::Duration(0.1).sleep();
        }
        sm_.submit_event(ArbitratorEvent::PLANNING_TIMER_TRIGGER);
    }

    void Arbitrator::paused_state()
    {
        ros::Duration(0.1).sleep();
    }

    void Arbitrator::shutdown_state()
    {
        ROS_INFO_STREAM("Arbitrator shutting down...");
        ros::shutdown();
        exit(0);
    }
};
