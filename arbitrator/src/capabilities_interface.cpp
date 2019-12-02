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

#include "capabilities_interface.hpp"
#include <cav_srvs/PluginList.h>
#include <cav_srvs/PlanManeuvers.h>
#include <exception>

namespace arbitrator
{
    const std::string CapabilitiesInterface::STRATEGIC_PLAN_CAPABILITY = "strategic_plan/plan_maneuvers";
    const std::string CapabilitiesInterface::TACTICAL_PLAN_CAPABILITY = "tactical_plan";
    const std::string CapabilitiesInterface::CONTROL_CAPABILITY = "control";

    void CapabilitiesInterface::initialize() 
    {
        static ros::ServiceClient sc = nh_->serviceClient<cav_srvs::PluginList>("/guidance/plugins/get_registered_plugins");
        cav_srvs::PluginList msg;
        if (sc.call(msg)) {
            for (auto it = msg.response.plugins.begin(); it != msg.response.plugins.end(); it++)
            {
                if (it->activated)
                {
                    std::string capability;
                    switch (it->type)
                    {
                        case cav_msgs::Plugin::STRATEGIC:
                            capability = STRATEGIC_PLAN_CAPABILITY;
                            break;
                        case cav_msgs::Plugin::TACTICAL:
                            capability = TACTICAL_PLAN_CAPABILITY;
                            break;
                        case cav_msgs::Plugin::CONTROL:
                            capability = CONTROL_CAPABILITY;
                            break;
                        case cav_msgs::Plugin::UNKNOWN:
                        default:
                            throw std::invalid_argument("Activated plugin of unknown type discovered: " + it->name);
                    }
                    // NOTE: It is assumed for now (due to lack of true capabilties register),
                    // That all plugins conform to the convention of providing a service for
                    // planning maneuvers at "/plugins/{PLUGIN_NAME}/strategic_plan/plan_maneuvers"
                    std::string topic = "/guidance/plugins/" + it->name + "/" + capability;
                    // Check if bucket for that capability already exists
                    if (capabilities_.find(capability) != capabilities_.end())
                    {
                        capabilities_[capability].push_back(topic);
                    }
                    else
                    {
                        capabilities_.emplace(capability, std::vector<std::string>{topic});
                    }

                    // TODO: Special cased for now due to lack of broader capabilities service
                    if (capability == STRATEGIC_PLAN_CAPABILITY)
                    {
                        ros::ServiceClient sc = nh_->serviceClient<cav_srvs::PlanManeuvers>(topic);
                        service_clients_.emplace(topic, sc);
                    }
                }
            }
        }
    }
    
    std::vector<std::string> CapabilitiesInterface::get_topics_for_capability(std::string query_string) const
    {
        auto it = capabilities_.find(query_string);
        if (it != capabilities_.end())
        {
            return it->second;
        }
        else
        {
            // Return an empty list to indicate failure
            return std::vector<std::string>{};
        }
    }
}
