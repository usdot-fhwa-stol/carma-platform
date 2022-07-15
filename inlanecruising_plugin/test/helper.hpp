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

 
#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace std_ph = std::placeholders;

class Node : public carma_ros2_utils::CarmaLifecycleNode
{
    public:
        explicit Node(const rclcpp::NodeOptions &options) : carma_ros2_utils::CarmaLifecycleNode(options){};
    
    void callback(
        std::shared_ptr<rmw_request_id_t> srv_header, 
        carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
        carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
    {
        if (req->initial_trajectory_plan.trajectory_id == "YieldReq"){
            resp->trajectory_plan.trajectory_id = "YieldResp";
        } 
        RCLCPP_INFO_STREAM(rclcpp::get_logger("inlanecruising_plugin"), "Yield callback done");
    }
};


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(Node)
