#pragma once
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
 */

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>


class CallInterface
{
    public:
        virtual void call(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp) = 0;
        virtual ~CallInterface() {};

                /**
        * \brief set the trajectory service client
        * 
        * \param client input trajectory service client
        */
       virtual void set_client(carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> srv_client) = 0;
};
