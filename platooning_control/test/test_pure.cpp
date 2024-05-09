/*
 * Copyright (C) 2024 LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "platoon_control/pure_pursuit.hpp"



TEST(PurePursuitTest, test_filter)
{

    platoon_control::PlatooningControlPluginConfig config;
	config.lowpass_gain = 0.5;

    platoon_control::PurePursuit pp;
    pp.config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>(config);
    double new_angle = pp.lowPassfilter(3.0, 0, config.lowpass_gain);
    EXPECT_EQ(1.5, new_angle);

}

TEST(PurePursuitTest, test1)
{

    carma_planning_msgs::msg::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = rclcpp::Time(1.0,0.0);

    platoon_control::PlatooningControlPluginConfig config;
    platoon_control::PurePursuit pp;
    pp.config_ = std::make_shared<platoon_control::PlatooningControlPluginConfig>(config);
    pp.calculateSteer(point);
    EXPECT_NEAR(0.6, pp.steering_angle_, 0.1);


}