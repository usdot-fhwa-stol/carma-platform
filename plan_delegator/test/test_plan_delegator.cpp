/*
 * Copyright (C) 2022 LEIDOS.
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
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/srv/plan_trajectory.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include "plan_delegator.hpp"

    class PlanDelegatorTest : public plan_delegator::PlanDelegator
    {
        public:
            explicit PlanDelegatorTest(const rclcpp::NodeOptions &options) : plan_delegator::PlanDelegator(options){};

            // Helper functions for unit test
            std::string getPlanningTopicPrefix()
            {
                return this->config_.planning_topic_prefix;
            }

            void setPlanningTopicPrefix(std::string prefix)
            {
                this->config_.planning_topic_prefix = prefix;
            }

            std::string getPlanningTopicSuffix()
            {
                return this->config_.planning_topic_suffix;
            }

            void setPlanningTopicSuffix(std::string suffix)
            {
                this->config_.planning_topic_suffix = suffix;
            }

            double getSpinRate()
            {
                return this->config_.trajectory_planning_rate;
            }

            double getMaxTrajDuration()
            {
                return this->config_.max_trajectory_duration;
            }

            carma_planning_msgs::msg::ManeuverPlan getLatestManeuverPlan()
            {
                return this->latest_maneuver_plan_;
            }

            std::unordered_map<std::string, carma_ros2_utils::ClientPtr<carma_planning_msgs::srv::PlanTrajectory>> getServiceMap()
            {
                return this->trajectory_planners_;
            }
    };

    TEST(TestPlanDelegator, UnitTestPlanDelegator) {
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        auto pd = std::make_shared<PlanDelegatorTest>(options);
        rclcpp_lifecycle::State dummy;
        pd->handle_on_configure(dummy);
        pd->handle_on_activate(dummy);
        // test initialization
        EXPECT_EQ(0, pd->getPlanningTopicPrefix().compare("/plugins/"));
        EXPECT_EQ(0, pd->getPlanningTopicSuffix().compare("/plan_trajectory"));
        EXPECT_EQ(10.0, pd->getSpinRate());
        EXPECT_EQ(6.0, pd->getMaxTrajDuration());
        // test maneuver plan callback
        carma_planning_msgs::msg::ManeuverPlan plan;
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = maneuver.LANE_FOLLOWING;
        maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
        plan.maneuvers.push_back(maneuver);
        pd->maneuverPlanCallback(std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(plan));
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd->getLatestManeuverPlan().maneuvers[0], parameters.planning_strategic_plugin));
        carma_planning_msgs::msg::ManeuverPlan new_plan;
        pd->maneuverPlanCallback(std::make_unique<carma_planning_msgs::msg::ManeuverPlan>(new_plan));
        // empty plan should not be stored locally
        EXPECT_EQ("plugin_A", GET_MANEUVER_PROPERTY(pd->getLatestManeuverPlan().maneuvers[0], parameters.planning_strategic_plugin));
        // test create service client
        EXPECT_THROW(pd->getPlannerClientByName(""), std::invalid_argument);
        pd->setPlanningTopicPrefix("/guidance/plugins/");
        pd->setPlanningTopicSuffix("/plan_trajectory");
        auto plugin_A = pd->getPlannerClientByName("plugin_A");
        EXPECT_EQ(0, std::string(plugin_A->get_service_name()).compare("/guidance/plugins/plugin_A/plan_trajectory"));
        EXPECT_EQ(1, pd->getServiceMap().size());
        auto plugin_A_copy = pd->getPlannerClientByName("plugin_A");
        EXPECT_EQ(true, plugin_A == plugin_A_copy);
        // test expired maneuver
        rclcpp::Time test_time(0, 1000);
        carma_planning_msgs::msg::Maneuver test_maneuver;
        test_maneuver.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        test_maneuver.lane_following_maneuver.end_time = test_time;
        EXPECT_EQ(true, pd->isManeuverExpired(test_maneuver, pd->get_clock()->now()));
        rclcpp::Time test_time_earlier(0, 500, pd->get_clock()->get_clock_type());
        EXPECT_EQ(false, pd->isManeuverExpired(test_maneuver, test_time_earlier));
        // test compose new plan trajectory request
        uint16_t current_maneuver_index = 0;
        carma_planning_msgs::msg::TrajectoryPlan traj_plan;
        carma_planning_msgs::msg::TrajectoryPlanPoint point_1;
        point_1.x = 0.0;
        point_1.y = 0.0;
        point_1.target_time = rclcpp::Time(0, 0, pd->get_clock()->get_clock_type());
        carma_planning_msgs::msg::TrajectoryPlanPoint point_2;
        point_2.x = 1.0;
        point_2.y = 1.0;
        point_2.target_time = rclcpp::Time(1.41421e9, pd->get_clock()->get_clock_type());
        traj_plan.trajectory_points.push_back(point_1);
        traj_plan.trajectory_points.push_back(point_2);
        auto req = pd->composePlanTrajectoryRequest(traj_plan, current_maneuver_index);
        EXPECT_NEAR(1.0, req->vehicle_state.x_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.y_pos_global, 0.01);
        EXPECT_NEAR(1.0, req->vehicle_state.longitudinal_vel, 0.1);
        EXPECT_EQ(0, req->maneuver_index_to_plan);
    }

    TEST(TestPlanDelegator, TestPlanDelegator) {
        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        auto pd = std::make_shared<PlanDelegatorTest>(options);
        rclcpp_lifecycle::State dummy;
        pd->handle_on_configure(dummy);
        pd->handle_on_activate(dummy);
        
        carma_planning_msgs::msg::TrajectoryPlan res_plan;

        auto maneuver_pub = pd->create_publisher<carma_planning_msgs::msg::ManeuverPlan>("final_maneuver_plan", 5);

        carma_planning_msgs::msg::ManeuverPlan plan;
        carma_planning_msgs::msg::Maneuver maneuver;
        maneuver.type = maneuver.LANE_FOLLOWING;
        maneuver.lane_following_maneuver.parameters.planning_strategic_plugin = "plugin_A";
        plan.maneuvers.push_back(maneuver);
        maneuver_pub->on_activate();
        maneuver_pub->publish(plan);
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
        auto num = maneuver_pub->get_subscription_count();
        EXPECT_EQ(1, num);
    }

    /*!
    * \brief Main entrypoint for unit tests
    */
    int main (int argc, char **argv) {
        ::testing::InitGoogleTest(&argc, argv);

        //Initialize ROS
        rclcpp::init(argc, argv);
        auto ret = rcutils_logging_set_logger_level(
                rclcpp::get_logger("plan_delegator").get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

        bool success = RUN_ALL_TESTS();

        //shutdown ROS
        rclcpp::shutdown();

        return success;
    }
