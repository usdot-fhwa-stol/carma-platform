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

#include "capabilities_interface.hpp"
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include <exception>
#include <sstream>

namespace arbitrator
{
    const std::string CapabilitiesInterface::STRATEGIC_PLAN_CAPABILITY = "strategic_plan/plan_maneuvers";
    
    std::vector<std::string> CapabilitiesInterface::get_topics_for_capability(const std::string& query_string)
    {
        std::vector<std::string> topics = {};
        
        auto srv = std::make_shared<carma_planning_msgs::srv::GetPluginApi::Request>();
        srv->capability = "";
        
        auto plan_response = sc_s_->async_send_request(srv);

        auto future_status = plan_response.wait_for(std::chrono::milliseconds(200));

        if (query_string == STRATEGIC_PLAN_CAPABILITY && future_status == std::future_status::ready)
        {
            topics = plan_response.get()->plan_service;
            
            // Log the topics
            std::ostringstream stream;
            stream << "Received Topics: ";
            for (const auto& topic : topics) {
                stream << topic << ", ";
            }
            stream << std::endl;
            RCLCPP_INFO_STREAM(nh_->get_logger(), stream.str().c_str());
        }
        else
        {
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "service call failed...");
            RCLCPP_ERROR_STREAM(nh_->get_logger(), "client: " << sc_s_);
        }

        return topics;

    }
}
