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
#include "plan_delegator.hpp"

    class PlanDelegatorTest : public plan_delegator::PlanDelegator
    {
        public:

            // Helper functions for unit test
            std::string GetPlanningTopicPrefix()
            {
                return this->planning_topic_prefix_;
            }

            void SetPlanningTopicPrefix(std::string prefix)
            {
                this->planning_topic_prefix_ = prefix;
            }

            std::string GetPlanningTopicSuffix()
            {
                return this->planning_topic_suffix_;
            }

            void SetPlanningTopicSuffix(std::string suffix)
            {
                this->planning_topic_suffix_ = suffix;
            }

            double GetSpinRate()
            {
                return this->spin_rate_;
            }

            double GetMaxTrajDuration()
            {
                return this->max_trajectory_duration_;
            }

            cav_msgs::ManeuverPlan GetLatestManeuverPlan()
            {
                return this->latest_maneuver_plan_;
            }

            std::unordered_map<std::string, ros::ServiceClient> GetServiceMap()
            {
                return this->trajectory_planners_;
            }
    };

    TEST(TestPlanDelegator, UnitTestPlanDelegator) {
        PlanDelegatorTest pd;
        // test initialization
        EXPECT_EQ(0, pd.GetPlanningTopicPrefix().compare(""));
        EXPECT_EQ(0, pd.GetPlanningTopicSuffix().compare(""));
        EXPECT_EQ(10.0, pd.GetSpinRate());
        EXPECT_EQ(6.0, pd.GetMaxTrajDuration());
        // test maneuver plan callback
        cav_msgs::ManeuverPlan plan;
        cav_msgs::Maneuver maneuver;
        maneuver.type = maneuver.LANE_FOLLOWING;
        maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
        plan.maneuvers.push_back(maneuver);
        pd.ManeuverPlanCallback(cav_msgs::ManeuverPlanConstPtr(new cav_msgs::ManeuverPlan(plan)));
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd.GetLatestManeuverPlan().maneuvers[0], parameters.planning_strategic_plugin));
        cav_msgs::ManeuverPlan new_plan;
        pd.ManeuverPlanCallback(cav_msgs::ManeuverPlanConstPtr(new cav_msgs::ManeuverPlan(new_plan)));
        // empty plan should not be stored locally
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd.GetLatestManeuverPlan().maneuvers[0], parameters.planning_strategic_plugin));
        // test create service client
        pd.SetPlanningTopicPrefix("/guidance/plugins/");
        pd.SetPlanningTopicSuffix("/plan_trajectory");
        ros::ServiceClient plugin_A = pd.GetPlannerClientByName("plugin_A");
        EXPECT_EQ("/guidance/plugins/plugin_A/plan_trajectory", plugin_A.getService());
        EXPECT_EQ(1, pd.GetServiceMap().size());
        ros::ServiceClient plugin_A_copy = pd.GetPlannerClientByName("plugin_A");
        EXPECT_EQ(true, plugin_A == plugin_A_copy);
        // test expired maneuver
        ros::Time test_time(0, 1000);
        cav_msgs::Maneuver test_maneuver;
        test_maneuver.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        test_maneuver.lane_following_maneuver.end_time = test_time;
        EXPECT_EQ(true, pd.IsManeuverExpired(test_maneuver));
        ros::Time test_time_eariler(0, 500);
        EXPECT_EQ(false, pd.IsManeuverExpired(test_maneuver, test_time_eariler));
        // test compose new plan trajectory request
        cav_msgs::TrajectoryPlan traj_plan;
        cav_msgs::TrajectoryPlanPoint point_1;
        point_1.x = 0.0;
        point_1.y = 0.0;
        point_1.target_time = 0;
        cav_msgs::TrajectoryPlanPoint point_2;
        point_2.x = 1.0;
        point_2.y = 1.0;
        point_2.target_time = 1.41421 / PlanDelegatorTest::MILLISECOND_TO_SECOND;
        traj_plan.trajectory_points.push_back(point_1);
        traj_plan.trajectory_points.push_back(point_2);
        cav_srvs::PlanTrajectory req = pd.ComposePlanTrajectoryRequest(traj_plan);
        EXPECT_NEAR(1.0, req.request.vehicle_state.X_pos_global, 0.01);
        EXPECT_NEAR(1.0, req.request.vehicle_state.Y_pos_global, 0.01);
        EXPECT_NEAR(1.0, req.request.vehicle_state.longitudinal_vel, 0.1);
    }

    TEST(TestPlanDelegator, TestPlanDelegator) {
        ros::NodeHandle nh = ros::NodeHandle();
        cav_msgs::TrajectoryPlan res_plan;
        // bool flag = false;
        ros::Publisher maneuver_pub = nh.advertise<cav_msgs::ManeuverPlan>("maneuver_plan", 5);
        // ros::Subscriber traj_sub = nh.subscribe<cav_msgs::TrajectoryPlan>("trajectory_plan", 5, [&](cav_msgs::TrajectoryPlanConstPtr msg){
        //     res_plan = msg.get();
            
        // });
        // boost::function<bool(cav_srvs::PlanTrajectoryRequest&, cav_srvs::PlanTrajectoryResponse&)> cb = [&](cav_srvs::PlanTrajectoryRequest& req, cav_srvs::PlanTrajectoryResponse& res) -> bool
        // {
        //     flag = true;
        // //     cav_msgs::TrajectoryPlan sending_plan;
        // //     sending_plan.trajectory_id = "plugin_A";
        // //     res.trajectory_plan = sending_plan;
        //     return true;
        // };
        //ros::ServiceServer plugin_A_server = nh.advertiseService("/guidance/plugins/plugin_A/plan_trajectory", cb);
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
    }

    /*!
    * \brief Main entrypoint for unit tests
    */
    int main (int argc, char **argv) {
        testing::InitGoogleTest(&argc, argv);
        ros::init(argc, argv, "test_plan_delegator");
        //std::thread spinner([] {while (ros::ok()) ros::spin();});
        auto res = RUN_ALL_TESTS();
        //ros::shutdown();
        return res;
    }
