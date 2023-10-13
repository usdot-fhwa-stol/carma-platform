/*
 * Copyright (C) 2023 LEIDOS.
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
 *
 */
#include <gtest/gtest.h>
#include "call_test.hpp"
#include <rclcpp/rclcpp.hpp>

namespace call_test
{

    void CallTest::call(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr& resp)
    {
        request = req;
        resp->trajectory_plan = req->initial_trajectory_plan;
        response = resp;
        return;
    }

    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr CallTest::getRequest()
    {
        return request;
    }
    
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr CallTest::getResponse()
    {
        return response;
    }



}