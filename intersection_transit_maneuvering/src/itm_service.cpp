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

#include "intersection_transit_maneuvering/itm_service.hpp"
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/trajectory_plan_point.hpp>
#include <carma_planning_msgs/msg/plugin.hpp>
#include <carma_planning_msgs/msg/maneuver.hpp>
#include <boost/shared_ptr.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <boost/geometry.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>

namespace intersection_transit_maneuvering
{

    Servicer::Servicer(){};

    void Servicer::call(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr& resp)
    {
       std::shared_future<carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr> resp_future = client->async_send_request(req);

       auto future_status = resp_future.wait_for(std::chrono::milliseconds(500));
                    
        if (future_status == std::future_status::ready) {
            resp = resp_future.get();
            
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("intersection_transit_maneuvering"), "Responsible tactical plugin was called and got trajectory size: " << resp->trajectory_plan.trajectory_points.size());
        }
        else
        {
            RCLCPP_DEBUG(rclcpp::get_logger("intersection_transit_maneuvering"), "Failed to call the responsible tactical plugin from intersection_transit_maneuvering");
        }
        return;

    }

    
    void Servicer::set_client( carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory> srv_client)
    {
        client = srv_client;
    }
}