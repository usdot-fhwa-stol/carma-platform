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
#include <gtest/gtest.h>

TEST(RouteStateTest, testStateTransitions)
{
    // Create a RouteStateWorker for this test
    auto node = std::make_shared<rclcpp::Node>("test_node");
    route::RouteStateWorker worker;
    worker.setLoggerInterface(node->get_node_logging_interface());
    
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_ABORTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_LOADED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::SELECTION, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_STARTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::FOLLOWING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_DEPARTED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::LOADING, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_LOADED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_GEN_FAILED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::SELECTION, worker.getRouteState());
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_LOADED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_STARTED);
    worker.onRouteEvent(route::RouteStateWorker::RouteEvent::ROUTE_INVALIDATION);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTING, worker.getRouteState());
}