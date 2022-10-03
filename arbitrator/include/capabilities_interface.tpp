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

#ifndef __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_TPP__ARBITRATOR_INCLUDE_
#define __ARBITRATOR_INCLUDE_CAPABILITIES_INTERFACE_TPP__ARBITRATOR_INCLUDE_

#include <vector>
#include <map>
#include <string>
#include <functional>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>

namespace arbitrator 
{
    template<typename MSrvReq, typename MSrvRes>
    std::map<std::string, std::shared_ptr<MSrvRes>> 
        CapabilitiesInterface::multiplex_service_call_for_capability(const std::string& query_string, std::shared_ptr<MSrvReq> msg)
    {
        std::vector<std::string> topics = get_topics_for_capability(query_string);

        std::map<std::string, std::shared_ptr<MSrvRes>> responses;

        for (auto i = topics.begin(); i != topics.end(); i++) 
        {
            auto topic = *i; //todo revert hardcode, lci_strategic is the last ros1 plugin
                             // https://github.com/usdot-fhwa-stol/carma-platform/issues/1949
            if (topic == "lci_strategic_plugin/plan_maneuvers")
                topic = "/guidance/plugins/lci_strategic_plugin/plan_maneuvers";
            auto sc = nh_->create_client<carma_planning_msgs::srv::PlanManeuvers>(topic);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"), "found client: " << topic);
            
            std::shared_future<std::shared_ptr<MSrvRes>> resp = sc->async_send_request(msg);

            auto future_status = resp.wait_for(std::chrono::milliseconds(500));
            
            if (future_status == std::future_status::ready) {
                responses.emplace(topic, resp.get());
            }
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "failed...: " << topic);
            }
        }
        return responses;
    }
};

#endif
