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

namespace arbitrator
{
    void Arbitrator::run()
    {
        nh_ = ros::CARMANodeHandle("arbitrator");
        pnh_ = ros::CARMANodeHandle("~");

        while (!ros::isShuttingDown)
        {
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
                    // TODO: Throw exception or otherwise handle
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
                sm_.submit_event(ArbitratorEvent::ARBIRTATOR_PAUSED);
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
        guidance_state_sub_ = nh_.subscribe<cav_msgs::GuidanceState>("guidance_state", 5, &guidance_state_cb);
    }

    void Arbitrator::planning_state()
    {
        cav_srvs::PlanManeuversRequest inital_plan_request;
        std::map<std::string, cav_srvs::PlanManeuversResponse> responses = 
            capabilities_interface_.multiplex_service_call_for_capability
            <cav_srvs::PlanManeuversRequest, cav_srvs::PlanManeuversResponse>
            ("strategic_plan", inital_plan_request);
        
        for (auto it = responses.begin(); it != responses.end(); it++)
        {
            it->
        }
    }

    void Arbitrator::waiting_state()
    {

    }

    void Arbitrator::paused_state()
    {

    }

    void Arbitrator::shutdown_state()
    {

    }
};
