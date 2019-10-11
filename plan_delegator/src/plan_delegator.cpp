/*
 * Copyright (C) 2019 LEIDOS.
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

#include <ros/ros.h>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanTrajectory.h>
#include <carma_utils/CARMAUtils.h>
#include "plan_delegator.hpp"

namespace plan_delegator
{
    int PlanDelegator::run() 
    {
        ros::NodeHandle nh = ros::CARMANodeHandle();
        ros::NodeHandle pnh = ros::CARMANodeHandle("~");

        ros::Publisher traj_pub = nh.advertise<cav_msgs::TrajectoryPlan>("trajectory_plan", 1);

        std::string planning_topic_prefix;
        pnh.param<std::string>("planning_topic_prefix", planning_topic_prefix, "/guidance/plugins/");
        std::string planning_topic_suffix;
        pnh.param<std::string>("planning_topic_suffix", planning_topic_suffix, "/plan_trajectory");

        std::map<std::string, ros::ServiceClient> trajectory_planners;

        ros::Subscriber plan_sub = nh.subscribe<cav_msgs::ManeuverPlan>(
            "maneuver_plan", 
            5,
            [planning_topic_prefix, planning_topic_suffix, &trajectory_planners, &traj_pub, &nh](cav_msgs::ManeuverPlan::ConstPtr plan){
                ROS_INFO_STREAM("Received request to delegate plan ID " << plan->maneuver_plan_id);
                if (plan->maneuvers.empty())
                {
                    ROS_WARN_STREAM("Received empty plan, no maneuvers found in plan ID " << plan->maneuver_plan_id);
                    return;
                }

                std::string first_maneuver_planner = GET_MANEUVER_PROPERTY(plan->maneuvers.front(), parameters.planning_strategic_plugin);
                auto it = trajectory_planners.find(first_maneuver_planner);
                if (it == trajectory_planners.end())
                {
                    // If its not in the map, add it to the map
                    ROS_INFO_STREAM("Discovered new trajectory planner: " << first_maneuver_planner);
                    std::string topic = planning_topic_prefix + first_maneuver_planner + planning_topic_suffix;
                    ros::ServiceClient client = nh.serviceClient<cav_srvs::PlanTrajectory>(topic);
                    trajectory_planners.emplace(first_maneuver_planner, client);
                }

                cav_srvs::PlanTrajectory plan_req;
                plan_req.request.maneuver_plan = *plan;
                if (trajectory_planners[first_maneuver_planner].call(plan_req))
                {
                    ROS_INFO_STREAM("Sucessful service call to trajctory planner: " << first_maneuver_planner << "for plan ID " << plan->maneuver_plan_id);
                    traj_pub.publish(plan_req.response.trajectory_plan);
                }
                else
                {
                    ROS_WARN_STREAM("Unsuccessful service call to trajectory planner:" << first_maneuver_planner << "for plan ID " << plan->maneuver_plan_id);
                }
            });

        ros::spin();

        return 0;
    }
}
