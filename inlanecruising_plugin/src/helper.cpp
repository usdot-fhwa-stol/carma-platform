/*
 * Copyright (C) 2019-2020 LEIDOS.
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

 
#include "ros/ros.h"
#include <cav_srvs/PlanTrajectory.h>

bool callback(cav_srvs::PlanTrajectory::Request  &req,
         cav_srvs::PlanTrajectory::Response &res)
{
    if (req.initial_trajectory_plan.trajectory_id == "YieldReq"){
        res.trajectory_plan.trajectory_id = "YieldResp";
    } 
    ROS_ERROR("Yield callback");
    return true;
}

// Helper node to include the callback function for yield trajectory
int main(int argc, char **argv)
{
  ros::init(argc, argv, "helper");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plugins/YieldPlugin/plan_trajectory", callback);
  ros::spin();
  ros::Duration(5).sleep();

  return 0;
}