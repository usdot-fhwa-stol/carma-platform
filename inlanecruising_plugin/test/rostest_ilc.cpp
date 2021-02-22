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

#include <inlanecruising_plugin/inlanecruising_plugin.h>
#include <inlanecruising_plugin/inlanecruising_plugin_node.h>
#include <carma_utils/CARMAUtils.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "ros/service.h"
#include <thread>
#include <chrono>



bool srvCallback(cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res)
{
  return true;
}

// TEST(InLaneCruisingPluginTest, advertiseMultiple)
// {
//   ros::NodeHandle nh;
//   ros::ServiceServer srv = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", srvCallback);
//   std::this_thread::sleep_for(std::chrono::milliseconds(25000));
//   ASSERT_TRUE(srv);
//   ASSERT_TRUE(ros::service::exists("plugins/YieldPlugin/plan_trajectory", true));

// }

// TEST(InLaneCruisingPluginTest, rostest2)
// {
    

//     ros::NodeHandle nh;
//     bool flag_yield = false;

//     boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
//     {
//         flag_yield = true;
//         return true;
//     };

//     ros::ServiceServer srv = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", cb);
//     ros::Duration(5).sleep();
//     ros::spinOnce();
//     ASSERT_TRUE(srv);
//     ASSERT_TRUE(flag_yield);
// }


TEST(InLaneCruisingPluginTest, rostest1)
{
    

    ros::CARMANodeHandle nh;
    bool flag_trajectory = false;
    bool flag_yield = false;

    boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
    {
        ROS_ERROR("received yield service");
        flag_yield = true;
        // ros::spinOnce();
        // ros::Duration(5).sleep();
        return true;
    };

    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver maneuver;
    maneuver.type = maneuver.LANE_FOLLOWING;
    maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "InLane_Cruising";
    plan.maneuvers.push_back(maneuver);

    cav_msgs::TrajectoryPlan original_tp;
    cav_srvs::PlanTrajectory traj_srv;
    traj_srv.request.initial_trajectory_plan = original_tp;
    traj_srv.request.vehicle_state.X_pos_global= 1;
    traj_srv.request.vehicle_state.Y_pos_global = 1;
    traj_srv.request.vehicle_state.longitudinal_vel = 11;
    traj_srv.request.maneuver_plan = plan;


    ros::ServiceServer srv = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", cb);
    ros::Duration(5).sleep();
    

    ros::ServiceClient plugin1= nh.serviceClient<cav_srvs::PlanTrajectory>("plugins/InLaneCruisingPlugin/plan_trajectory");

    ROS_INFO_STREAM("ilc service: " << plugin1.getService());
    // if(ros::service::waitForService("plugins/InLaneCruisingPlugin/plan_trajectory",-1))
    // {
        ROS_ERROR("waiting");
        if (plugin1.call(traj_srv))
        {
            ROS_ERROR("ILC Traj Service called");
            flag_trajectory = true;
            
        }
        else
        {
            ROS_ERROR("Failed to call ILC Traj service");
        }
    // }

    // ros::ServiceServer srv = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", cb);
    // ros::Duration(5).sleep();
    // ros::spinOnce();
    


    
    // ros::Duration(5).sleep();
    // ros::spinOnce();

    EXPECT_TRUE(flag_trajectory);
    // ASSERT_TRUE(srv);
    ASSERT_TRUE(flag_yield);
}



int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rostest_ilc");
    ros::NodeHandle nh;
    auto res = RUN_ALL_TESTS();
    return res;
}

