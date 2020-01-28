/*
 * Copyright (C) 2020 LEIDOS.
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

#include "route_state_worker.h"

namespace route {

    RouteStateWorker::RouteStateWorker() : state_(RouteState::ROUTE_LOADING) { }

    RouteStateWorker::RouteState RouteStateWorker::get_route_state() {
        return state_;
    }

    void RouteStateWorker::on_route_event(RouteEvent event) {
        switch (state_)
        {
        case RouteState::ROUTE_LOADING:
            if(event == RouteEvent::LOAD_ROUTE_FILES)
            {
                state_ = RouteState::ROUTE_SELECTION;
            }
            break;
        case RouteState::ROUTE_SELECTION:
            if(event == RouteEvent::ROUTE_SELECTED)
            {
                state_ = RouteState::ROUTING;
            }
            break;
        case RouteState::ROUTING:
            if(event == RouteEvent::ROUTING_SUCCESS)
            {
                state_ = RouteState::ROUTE_FOLLOWING;
            } else if(event == RouteEvent::ROUTING_FAILURE)
            {
                state_ = RouteState::ROUTE_SELECTION;
            }
            break;
        case RouteState::ROUTE_FOLLOWING:
            if(event == RouteEvent::ROUTE_COMPLETE || event == RouteEvent::LEFT_ROUTE || event == RouteEvent::ROUTE_ABORT)
            {
                state_ = RouteState::ROUTE_LOADING;
            }
            break;
        default:
            //No-Op
            break;
        }
    }

}
