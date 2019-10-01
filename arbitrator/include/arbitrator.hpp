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

#ifndef __ARBITRATOR_HPP__
#define __ARBITRATOR_HPP__

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include "arbitrator_state_machine.hpp"
#include "planning_strategy.hpp"
#include "capabilities_interface.hpp"
#include <cav_msgs/GuidanceState.h>

namespace arbitrator 
{
    class Arbitrator 
    {
        public:
            Arbitrator(ros::CARMANodeHandle nh, ros::CARMANodeHandle pnh, ArbitratorStateMachine sm, CapabilitiesInterface ci, PlanningStrategy &planning_strategy):
                nh_(nh),
                pnh_(pnh),
                sm_(sm),
                capabilities_interface_(ci),
                planning_strategy_(planning_strategy) {};
            void run();
        protected:
            void initial_state();
            void planning_state();
            void waiting_state();
            void paused_state();
            void shutdown_state();
            void guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg);
        private:
            ArbitratorStateMachine sm_;
            ros::Publisher final_plan_pub_;
            ros::Subscriber guidance_state_sub_;
            ros::CARMANodeHandle nh_;
            ros::CARMANodeHandle pnh_;
            ros::Duration min_plan_duration;
            ros::Duration max_plan_duration;
            CapabilitiesInterface capabilities_interface_;
            PlanningStrategy &planning_strategy_;
    };
};

#endif
