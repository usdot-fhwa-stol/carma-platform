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
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(RouteStateTest, testStateTransitions)
{
    route::RouteStateWorker worker;
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_ABORT);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::LOAD_ROUTE_FILES);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_SELECTION, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTING_SUCCESS);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_FOLLOWING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETE);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::LEFT_ROUTE);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_COMPLETE);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_LOADING, worker.get_route_state());
    worker.on_route_event(route::RouteStateWorker::RouteEvent::LOAD_ROUTE_FILES);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTE_SELECTED);
    worker.on_route_event(route::RouteStateWorker::RouteEvent::ROUTING_FAILURE);
    ASSERT_EQ(route::RouteStateWorker::RouteState::ROUTE_SELECTION, worker.get_route_state());
}