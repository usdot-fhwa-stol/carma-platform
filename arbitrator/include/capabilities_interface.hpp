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

#ifndef __CAPABILITIES_INTERFACE_HPP__
#define __CAPABILITIES_INTERFACE_HPP__

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <vector>
#include <map>
#include <string>

namespace arbitrator
{
    const std::string STRATEGIC_PLAN_CAPABILITY = "strategic_plan/plan_maneuvers";
    const std::string TACTICAL_PLAN_CAPABILITY = "tactical_plan";
    const std::string CONTROL_CAPABILITY = "control";

    class CapabilitiesInterface
    {
        public:
            CapabilitiesInterface(ros::NodeHandle nh): nh_(nh) {};
            void initialize();
            std::vector<std::string> get_topics_for_capability(std::string query_string) const;

            template<typename MSrv>
            std::map<std::string, MSrv> multiplex_service_call_for_capability(std::string query_string, MSrv msg);

            /*template<typename Mmsg>
            void multiplex_publication_for_capability(std::string query_string, Mmsg msg) const; */
            
        protected:
        private:
            ros::NodeHandle nh_;
            std::map<std::string, ros::ServiceClient> service_clients_;
            //std::map<std::string, ros::Publisher> publishers_;
            std::map<std::string, std::vector<std::string>> capabilities_;
    };
};

#include "capabilities_interface.tpp"

#endif
