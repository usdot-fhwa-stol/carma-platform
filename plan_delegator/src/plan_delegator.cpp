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

#include <stdexcept>
#include "plan_delegator.hpp"

namespace plan_delegator
{
    void PlanDelegator::init()
    {
        ros::CARMANodeHandle nh_ = ros::CARMANodeHandle();
        ros::CARMANodeHandle pnh_ = ros::CARMANodeHandle("~");

        pnh_.param<std::string>("planning_topic_prefix", planning_topic_prefix_, "/guidance/plugins/");        
        pnh_.param<std::string>("planning_topic_suffix", planning_topic_suffix_, "/plan_trajectory");

        ros::Publisher traj_pub_ = nh_.advertise<cav_msgs::TrajectoryPlan>("trajectory_plan", 5);
        ros::Subscriber plan_sub_ = nh_.subscribe("maneuver_plan", 5, &PlanDelegator::ManeuverPlanCallback, this);
    }
    
    void PlanDelegator::run() 
    {
        ros::CARMANodeHandle::spin();
    }

    void PlanDelegator::ManeuverPlanCallback(const cav_msgs::ManeuverPlanConstPtr& plan)
    {
        ROS_INFO_STREAM("Received request to delegate plan ID " << plan->maneuver_plan_id);
        // do basic check to see if input is valid
        if (plan->maneuvers.empty())
        {
            ROS_WARN_STREAM("Received empty plan, no maneuvers found in plan ID " << plan->maneuver_plan_id);
            return;
        }
        // get corresponding ros service client for plan trajectory
        // TODO Before implement plugin capability interface, we use strategic plugin as target planner as default
        std::string first_maneuver_planner = GET_MANEUVER_PROPERTY(plan->maneuvers.front(), parameters.planning_strategic_plugin);
        auto client = GetPlannerClientByName(first_maneuver_planner);
        // compose service request
        auto plan_req = cav_srvs::PlanTrajectory{};
        plan_req.request.maneuver_plan = *plan;
        if (client.call(plan_req))
        {
            ROS_INFO_STREAM("Sucessful service call to trajctory planner: " << first_maneuver_planner << "for plan ID " << plan->maneuver_plan_id);
            traj_pub_.publish(plan_req.response.trajectory_plan);
        }
        else
        {
            ROS_WARN_STREAM("Unsuccessful service call to trajectory planner:" << first_maneuver_planner << "for plan ID " << plan->maneuver_plan_id);
        }
    }

    ros::ServiceClient& PlanDelegator::GetPlannerClientByName(const std::string& planner_name)
    {
        if(planner_name.size() == 0)
        {
            throw std::invalid_argument("Input maneuver planner name has zero length!");
        }
        if(trajectory_planners_.find(planner_name) == trajectory_planners_.end())
        {
            ROS_INFO_STREAM("Discovered new trajectory planner: " << planner_name);
            trajectory_planners_.emplace(planner_name, nh_.serviceClient<cav_srvs::PlanTrajectory>(planning_topic_prefix_ + planner_name + planning_topic_suffix_));
        }
        return trajectory_planners_[planner_name];
    }
}
