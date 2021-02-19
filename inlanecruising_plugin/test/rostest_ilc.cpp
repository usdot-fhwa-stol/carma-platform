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

TEST(InLaneCruisingPluginTest, rostest1)
{
    

    ros::CARMANodeHandle nh;
    bool flag_trajectory = false;
    bool flag_yield = false;

    boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
    {
        ROS_ERROR("received yield service");
        flag_yield = true;
        return true;
    };

    cav_srvs::PlanTrajectory traj_srv;
    ros::ServiceServer srv = nh.advertiseService("plugins/YieldPlugin/plan_trajectory", cb);
    ros::Duration(5).sleep();
    ros::spinOnce();
    

    ros::ServiceClient plugin1= nh.serviceClient<cav_srvs::PlanTrajectory>("plugins/InLaneCruisingPlugin/plan_trajectory");

    ROS_INFO_STREAM("ilc service: " << plugin1.getService());
    if(ros::service::waitForService("plugins/InLaneCruisingPlugin/plan_trajectory",-1))
    {
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
    }



    EXPECT_TRUE(flag_trajectory);
    ASSERT_TRUE(flag_yield);
}



int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "rostest_ilc");
    ros::NodeHandle nh;
    auto res = RUN_ALL_TESTS();
    return res;
}

