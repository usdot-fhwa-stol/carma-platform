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

#include "test_utils.h"
#include "tree_planner.hpp"
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "vehicle_state.hpp"

using ::testing::A;
using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::ReturnArg;
using ::testing::InSequence;

namespace arbitrator
{

    class MockSearchStrategy : public SearchStrategy
    {
        public:
            using PlanAndCost = std::pair<carma_planning_msgs::msg::ManeuverPlan, double>;
            MOCK_CONST_METHOD1(prioritize_plans, std::vector<PlanAndCost>(std::vector<PlanAndCost>));
            ~MockSearchStrategy(){};
    };

    class MockCostFunction : public CostFunction
    {
        public:
            MOCK_METHOD1(compute_total_cost, double(const carma_planning_msgs::msg::ManeuverPlan&));
            MOCK_METHOD1(compute_cost_per_unit_distance, double(const carma_planning_msgs::msg::ManeuverPlan&));
            ~MockCostFunction(){};

    };

    class MockNeighborGenerator : public NeighborGenerator
    {
        public:
            MOCK_CONST_METHOD2(generate_neighbors, std::vector<carma_planning_msgs::msg::ManeuverPlan>(carma_planning_msgs::msg::ManeuverPlan, const VehicleState&));
            ~MockNeighborGenerator(){};
    };

    class TreePlannerTest : public ::testing::Test 
    {
        public:
            TreePlannerTest():
                tp{mcf, 
                mng, 
                mss, 
                rclcpp::Duration(5, 0)} {};
            std::shared_ptr<MockSearchStrategy> mss = std::shared_ptr<MockSearchStrategy>(new MockSearchStrategy);
            std::shared_ptr<MockCostFunction> mcf =  std::shared_ptr<MockCostFunction>(new MockCostFunction);
            std::shared_ptr<MockNeighborGenerator> mng =  std::shared_ptr<MockNeighborGenerator>(new MockNeighborGenerator);
            TreePlanner tp;
    };

    TEST_F(TreePlannerTest, testGeneratePlan1)
    {
        carma_planning_msgs::msg::ManeuverPlan plan1;
        carma_planning_msgs::msg::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = rclcpp::Time(0, 0);
        mvr1.lane_following_maneuver.end_time = rclcpp::Time(5, 0);

        plan1.maneuvers.push_back(mvr1);
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans{plan1};

        {
            InSequence seq;
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillOnce(
                    Return(plans)
                );
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillRepeatedly(
                    Return(std::vector<carma_planning_msgs::msg::ManeuverPlan>())
                );
        }


        EXPECT_CALL(*mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(*mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        VehicleState state;
        carma_planning_msgs::msg::ManeuverPlan plan = tp.generate_plan(state);
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(1, plan.maneuvers.size());
        ASSERT_EQ(rclcpp::Time(0, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.start_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(5, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.end_time, RCL_SYSTEM_TIME));
    }

    TEST_F(TreePlannerTest, testGeneratePlan2)
    {
        carma_planning_msgs::msg::ManeuverPlan plan1, plan2, plan3;
        carma_planning_msgs::msg::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = rclcpp::Time(0);
        mvr1.lane_following_maneuver.end_time = rclcpp::Time(5, 0);

        mvr2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_time = rclcpp::Time(0);
        mvr2.lane_following_maneuver.end_time = rclcpp::Time(5, 0);

        mvr3.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr3.lane_following_maneuver.start_time = rclcpp::Time(0);
        mvr3.lane_following_maneuver.end_time = rclcpp::Time(5, 0);

        plan1.maneuvers.push_back(mvr1);
        plan2.maneuvers.push_back(mvr2);
        plan3.maneuvers.push_back(mvr3);
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans{plan1, plan2, plan3};

        {
            InSequence seq;
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillOnce(
                    Return(plans)
                );
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillRepeatedly(
                    Return(std::vector<carma_planning_msgs::msg::ManeuverPlan>())
                );
        }

        EXPECT_CALL(*mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(*mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        VehicleState state;
        carma_planning_msgs::msg::ManeuverPlan plan = tp.generate_plan(state);
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(1, plan.maneuvers.size());
        ASSERT_EQ(rclcpp::Time(0, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.start_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(5, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.end_time, RCL_SYSTEM_TIME));
    }

    TEST_F(TreePlannerTest, testGeneratePlan3)
    {
        carma_planning_msgs::msg::ManeuverPlan plan1, plan2, plan3;
        carma_planning_msgs::msg::Maneuver mvr1, mvr2, mvr3;

        mvr1.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr1.lane_following_maneuver.start_time = rclcpp::Time(0);
        mvr1.lane_following_maneuver.end_time = rclcpp::Time(2, 0);

        mvr2.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr2.lane_following_maneuver.start_time = rclcpp::Time(2, 0);
        mvr2.lane_following_maneuver.end_time = rclcpp::Time(4, 0);

        mvr3.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
        mvr3.lane_following_maneuver.start_time = rclcpp::Time(4, 0);
        mvr3.lane_following_maneuver.end_time = rclcpp::Time(5, 0);

        plan1.maneuvers.push_back(mvr1);

        plan2.maneuvers.push_back(mvr1);
        plan2.maneuvers.push_back(mvr2);

        plan3.maneuvers.push_back(mvr1);
        plan3.maneuvers.push_back(mvr2);
        plan3.maneuvers.push_back(mvr3);

        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans1{plan1};
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans2{plan2};
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans3{plan3};

        {
            InSequence seq;
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillOnce(
                    Return(plans1)
                );
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillOnce(
                    Return(plans2)
                );
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillOnce(
                    Return(plans3)
                );
            EXPECT_CALL(*mng, generate_neighbors(_,_))
                .WillRepeatedly(
                    Return(std::vector<carma_planning_msgs::msg::ManeuverPlan>())
                );
        }

        EXPECT_CALL(*mcf, compute_cost_per_unit_distance(_))
            .WillRepeatedly(
                Return(5.0)
            );

        EXPECT_CALL(*mss, prioritize_plans(_))
            .WillRepeatedly(
                ReturnArg<0>()
            );

        VehicleState state;
        carma_planning_msgs::msg::ManeuverPlan plan = tp.generate_plan(state);
        ASSERT_FALSE(plan.maneuvers.empty());
        ASSERT_EQ(3, plan.maneuvers.size());
        ASSERT_EQ(rclcpp::Time(0, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.start_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(2, 0), rclcpp::Time(plan.maneuvers[0].lane_following_maneuver.end_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(2, 0), rclcpp::Time(plan.maneuvers[1].lane_following_maneuver.start_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(4, 0), rclcpp::Time(plan.maneuvers[1].lane_following_maneuver.end_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(4, 0), rclcpp::Time(plan.maneuvers[2].lane_following_maneuver.start_time, RCL_SYSTEM_TIME));
        ASSERT_EQ(rclcpp::Time(5, 0), rclcpp::Time(plan.maneuvers[2].lane_following_maneuver.end_time, RCL_SYSTEM_TIME));
    }
}