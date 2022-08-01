/*
 * Copyright (C) 2020-2022 LEIDOS.
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

#include "route/route_state_worker.hpp"

namespace route {

    RouteStateWorker::RouteState RouteStateWorker::getRouteState() const {
        return state_;
    }

    void RouteStateWorker::onRouteEvent(RouteEvent event) {
        
        auto old_state = state_;

        switch (state_)
        {
        case RouteState::LOADING:
            if(event == RouteEvent::ROUTE_LOADED)
            {
                state_ = RouteState::SELECTION;
            }
            break;
        case RouteState::SELECTION:
            if(event == RouteEvent::ROUTE_SELECTED)
            {
                state_ = RouteState::ROUTING;
            }
            break;
        case RouteState::ROUTING:
            if(event == RouteEvent::ROUTE_STARTED)
            {
                state_ = RouteState::FOLLOWING;
            } else if(event == RouteEvent::ROUTE_GEN_FAILED)
            {
                state_ = RouteState::SELECTION;
            }
            break;
        case RouteState::FOLLOWING:
            if(event == RouteEvent::ROUTE_COMPLETED || event == RouteEvent::ROUTE_DEPARTED || event == RouteEvent::ROUTE_ABORTED)
            {
                state_ = RouteState::LOADING;
            } 
            else if(event == RouteEvent::ROUTE_INVALIDATION)
            {
                state_ = RouteState::ROUTING;
            }
            break;
        default:
            // should not reach here
            throw std::invalid_argument("Current state is illegal: " + std::to_string(state_));
        }

        RCLCPP_INFO_STREAM(logger_->get_logger(), "Received Route Event: " << event << " transitioning from: " << old_state << " to: " << state_);
    }

    void RouteStateWorker::setLoggerInterface(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
    {
        logger_ = logger;
    }
    
} // route