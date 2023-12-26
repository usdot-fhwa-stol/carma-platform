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
#include <chrono>
#include <thread>
namespace arbitrator
{
    constexpr auto MAX_RETRY_ATTEMPTS {10};

    template<typename MSrvReq, typename MSrvRes>
    std::map<std::string, std::shared_ptr<MSrvRes>>
        CapabilitiesInterface::multiplex_service_call_for_capability(const std::string& query_string, std::shared_ptr<MSrvReq> msg)
    {
        std::map<std::string, std::shared_ptr<MSrvRes>> responses;
        bool topics_call_successful = false;
        int retry_attempt = 0;
        std::vector<std::string> detected_topics;
        while (!topics_call_successful && retry_attempt < MAX_RETRY_ATTEMPTS)
        {
            try {
            detected_topics = get_topics_for_capability(query_string);
            topics_call_successful = true;
            if (retry_attempt > 0)
            {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("arbitrator")," Service call to get available strategic plugins successfully finished after retrying: " << retry_attempt);
            }
            } catch (const rclcpp::exceptions::RCLError& error) {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"),
                            "Could not make a service call to get available strategic service topic names, with detected error: " << error.what() <<". So retrying, attempt no: " << retry_attempt);
                retry_attempt ++;
            }

        }

        if (!topics_call_successful)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("arbitrator"),
                            "Failed to retrieve available strategic plugin service topics! Returning empty maneuver responses");
            return responses;
        }

        std::vector<std::string> topics_to_retry = detected_topics;
        std::vector<std::string> current_topics_to_check;
        retry_attempt = 0;
        while (!topics_to_retry.empty() && retry_attempt < MAX_RETRY_ATTEMPTS)
        {
            current_topics_to_check = topics_to_retry;
            topics_to_retry.clear();
            for (const auto & topic : current_topics_to_check)
            {
                try {
                    using std::literals::chrono_literals::operator""ms;

                    if (registered_strategic_plugins_.count(topic) == 0)
                        registered_strategic_plugins_[topic] = nh_->create_client<carma_planning_msgs::srv::PlanManeuvers>(topic);

                    auto client = registered_strategic_plugins_[topic];

                    if (client->wait_for_service(500ms))
                        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("arbitrator"), "found client: " << topic);
                    else
                    {
                        topics_to_retry.push_back(topic);
                        RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "Following client timed out: " << topic << ", retrying, attempt no: " << retry_attempt);
                        continue;
                    }

                    const auto response = client->async_send_request(msg);

                    switch (const auto status{response.wait_for(500ms)}) {
                        case std::future_status::ready:
                            responses.emplace(topic, response.get());
                            break;
                        case std::future_status::deferred:
                            RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "service call to " << topic << " is deferred... Please check if the plugin is active");
                            break;
                        case std::future_status::timeout:
                            RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "service call to " << topic << " is timed out... Please check if the plugin is active");
                            break;
                        default:
                            RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"), "service call to " << topic << " is failed... Please check if the plugin is active");
                            break;
                    }
                    if (retry_attempt > 0)
                    {
                        RCLCPP_INFO_STREAM(rclcpp::get_logger("arbitrator"),"Service call to: " << topic << ", successfully finished after retrying: " << retry_attempt);
                    }

                } catch(const rclcpp::exceptions::RCLError& error) {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"),
                        "Cannot make service request for service '" << topic << "': " << error.what() <<". So retrying, attempt no: " << retry_attempt);
                    topics_to_retry.push_back(topic);
                }
            }
            retry_attempt ++;
        }

        if (retry_attempt >= MAX_RETRY_ATTEMPTS)
        {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("arbitrator"),
                            "Failed to get a valid response from one or all of the strategic plugins...");
        }

        return responses;
    }
}

#endif
