/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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

#include <thread>
#include <chrono>
#include <cav_msgs/ManeuverPlan.h>
#include <cav_srvs/PlanTrajectory.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TestPlanDelegator, TestPlanDelegator) {
    ros::NodeHandle nh = ros::NodeHandle();
    cav_msgs::TrajectoryPlan res_plan;
    bool flag = false;
    ros::Publisher maneuver_pub = nh.advertise<cav_msgs::ManeuverPlan>("maneuver_plan", 5);
    // // ros::Subscriber traj_sub = nh.subscribe<cav_msgs::TrajectoryPlan>("trajectory_plan", 5, [&](cav_msgs::TrajectoryPlanConstPtr msg){
    // //     res_plan = msg.get();
        
    // // });
    boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
    {
        flag = true;
    // //     cav_msgs::TrajectoryPlan sending_plan;
    // //     sending_plan.trajectory_id = "plugin_A";
    // //     res.trajectory_plan = sending_plan;
       return true;
    };
    ros::ServiceServer plugin_A_server = nh.advertiseService("/guidance/plugins/plugin_A/plan_trajectory", cb);
    cav_msgs::ManeuverPlan plan;
    cav_msgs::Maneuver maneuver;
    maneuver.type = maneuver.LANE_FOLLOWING;
    maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
    plan.maneuvers.push_back(maneuver);
    maneuver_pub.publish(plan);
    // std::string res = res_plan->trajectory_id;
    // //EXPECT_EQ("plugin_A", res);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    auto num = maneuver_pub.getNumSubscribers();
    EXPECT_EQ(1, num);
    EXPECT_EQ(true, flag);
}

/*!
* \brief Main entrypoint for unit tests
*/
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_plan_delegator");
    std::thread spinner([] {while (ros::ok()) ros::spin();});
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}
