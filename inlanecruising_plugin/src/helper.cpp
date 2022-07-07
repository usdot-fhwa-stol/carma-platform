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

 
#include "ros/ros.hpp"
#include <carma_planning_msgs/srv/plan_trajectory.hpp>

bool callback(carma_planning_msgs::srv::PlanTrajectory::Request  &req,
         carma_planning_msgs::srv::PlanTrajectory::Response &res)
{
    if (req.initial_trajectory_plan.trajectory_id == "YieldReq"){
        res.trajectory_plan.trajectory_id = "YieldResp";
    } 
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("inlanecruising_plugin"), ("Yield callback");
    return true;
}

// Helper node to include the callback function for yield trajectory
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, "helper");
  rclcpp::NodeHandle n;

  rclcpp::ServiceServer service = n.advertiseService("plugins/YieldPlugin/plan_trajectory", callback);
  rclcpp::spin();
  rclcpp::Duration(5).sleep();

  return 0;
}