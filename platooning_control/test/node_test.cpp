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

#include "platoon_control/platoon_control.hpp"



TEST(PlatoonControlTest, testCase1)
{
    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<platoon_control::PlatoonControlPlugin>(options);

    worker_node->configure();
    worker_node->activate();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto num = worker_node->count_subscribers("/current_pose");
    EXPECT_EQ(1, num);
}
