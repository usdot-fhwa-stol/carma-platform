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

namespace arbitrator
{
    TEST_F(BeamSearchStrategyTest, testEmptyList)
    {
        auto vec = bss.prioritize_plans(std::vector<std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>());
        ASSERT_EQ(0, vec.size());
        ASSERT_TRUE(vec.empty());
    }

    TEST_F(BeamSearchStrategyTest, testSort1)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(5.0, vec[1].second, 0.01);
        ASSERT_NEAR(10.0, vec[2].second, 0.01);
    }

    TEST_F(BeamSearchStrategyTest, testSort2)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(5.0, vec[1].second, 0.01);
        ASSERT_NEAR(10.0, vec[2].second, 0.01);
    }

    TEST_F(BeamSearchStrategyTest, testSort3)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(5.0, vec[1].second, 0.01);
        ASSERT_NEAR(10.0, vec[2].second, 0.01);
    }

    TEST_F(BeamSearchStrategyTest, testSortAndPrune1)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 3.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 7.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 6.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 1.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 9.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 8.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 2.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 4.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(1.0, vec[1].second, 0.01);
        ASSERT_NEAR(2.0, vec[2].second, 0.01);
    }

    TEST_F(BeamSearchStrategyTest, testSortAndPrune2)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 9.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 8.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 7.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 6.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 4.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 3.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 2.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 1.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(1.0, vec[1].second, 0.01);
        ASSERT_NEAR(2.0, vec[2].second, 0.01);
    }

    TEST_F(BeamSearchStrategyTest, testSortAndPrune3)
    {
        auto vec = bss.prioritize_plans(std::vector<
            std::pair<carma_planning_msgs::msg::ManeuverPlan, double>>({
                { carma_planning_msgs::msg::ManeuverPlan(), 0.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 1.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 2.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 3.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 4.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 5.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 6.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 7.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 8.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 9.0 },
                { carma_planning_msgs::msg::ManeuverPlan(), 10.0 },
            }));

        ASSERT_EQ(3, vec.size());

        ASSERT_NEAR(0.0, vec[0].second, 0.01);
        ASSERT_NEAR(1.0, vec[1].second, 0.01);
        ASSERT_NEAR(2.0, vec[2].second, 0.01);
    }
}