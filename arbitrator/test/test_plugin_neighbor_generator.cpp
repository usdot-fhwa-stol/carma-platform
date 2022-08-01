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
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "plugin_neighbor_generator.hpp"

namespace arbitrator
{

    class MockCapabilitiesInterface 
    {
        public:
            using PluginResponses = std::map<std::string, std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Response>>;
            MOCK_METHOD2(get_plans, PluginResponses(std::string, std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Request>));
            MOCK_METHOD1(get_topics_for_capability, std::vector<std::string>(const std::string&));

            template<typename MSrvReq, typename MSrvRes>
            std::map<std::string, std::shared_ptr<MSrvRes>> multiplex_service_call_for_capability(std::string query_string, std::shared_ptr<MSrvReq> msg);
            ~MockCapabilitiesInterface(){};

    };

    template<>
    std::map<std::string, std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Response>> 
    MockCapabilitiesInterface::multiplex_service_call_for_capability(
        std::string query_string, 
        std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Request> msg)
    {
        return get_plans(query_string, msg);
    }

    class PluginNeighborGeneratorTest : public ::testing::Test 
    {
        public:
            PluginNeighborGeneratorTest():
            png{mci} {};

            std::shared_ptr<MockCapabilitiesInterface> mci = std::shared_ptr<MockCapabilitiesInterface>(new MockCapabilitiesInterface());
            PluginNeighborGenerator<MockCapabilitiesInterface> png;
    };

    TEST_F(PluginNeighborGeneratorTest, testInitalize)
    {
        // Verify that initialize just doesn't throw any exceptions
    }

    TEST_F(PluginNeighborGeneratorTest, testGetNeighbors1)
    {
        EXPECT_CALL(*mci, 
            get_plans(::testing::_, ::testing::_))
            .WillRepeatedly(
                ::testing::Return(
                    std::map<std::string, std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Response>>()));

        carma_planning_msgs::msg::ManeuverPlan plan;
        VehicleState vs;
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans = png.generate_neighbors(plan, vs);

        ASSERT_EQ(0, plans.size());
        ASSERT_TRUE(plans.empty());
    }

    TEST_F(PluginNeighborGeneratorTest, testGetNeighbors2)
    {
        std::map<std::string, std::shared_ptr<carma_planning_msgs::srv::PlanManeuvers::Response>> responses;
        auto resp1 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto resp2 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        auto resp3 = std::make_shared<carma_planning_msgs::srv::PlanManeuvers::Response>();
        
        resp1->new_plan.maneuver_plan_id = "1";
        resp2->new_plan.maneuver_plan_id = "2";
        resp3->new_plan.maneuver_plan_id = "3";
        responses["test_plugin_a"] = resp1;
        responses["test_plugin_b"] = resp2;
        responses["test_plugin_c"] = resp3;
        
        EXPECT_CALL(*mci, 
            get_plans(::testing::_, ::testing::_))
            .WillRepeatedly(
                ::testing::Return(
                    responses));
        carma_planning_msgs::msg::ManeuverPlan plan;
        VehicleState vs;
        std::vector<carma_planning_msgs::msg::ManeuverPlan> plans = png.generate_neighbors(plan, vs);
        ASSERT_FALSE(plans.empty());
        ASSERT_EQ(3, plans.size());
        ASSERT_EQ("1", plans[0].maneuver_plan_id);
        ASSERT_EQ("2", plans[1].maneuver_plan_id);
        ASSERT_EQ("3", plans[2].maneuver_plan_id);
    }
}