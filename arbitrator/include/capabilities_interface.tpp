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
#include <rclcpp/exceptions/exceptions.hpp>

namespace arbitrator
{
    template<typename MSrvReq, typename MSrvRes>
    std::map<std::string, std::shared_ptr<MSrvRes>>
        CapabilitiesInterface::multiplex_service_call_for_capability(const std::string& query_string, std::shared_ptr<MSrvReq> msg)
    {
        std::map<std::string, std::shared_ptr<MSrvRes>> responses;

        for (const auto & topic : get_topics_for_capability(query_string))
        {
            try {
                using std::literals::chrono_literals::operator""ms;

                const auto client = nh_->create_client<carma_planning_msgs::srv::PlanManeuvers>(topic);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"), "found client: " << topic);

                const auto response = client->async_send_request(msg);

                switch (const auto status{response.wait_for(500ms)}; status) {
                    case std::future_status::ready:
                        responses.emplace(topic, response.get());
                        break;
                    case std::future_status::deferred:
                    case std::future_status::timeout:
                        RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "failed...: " << topic);
                }
            } catch(const rclcpp::exceptions::RCLError& error) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("arbitrator"),
                    "Cannot make service request for service '" << topic << "': " << error.what());
                continue;
            }
        }

        return responses;
    }
}

#endif
