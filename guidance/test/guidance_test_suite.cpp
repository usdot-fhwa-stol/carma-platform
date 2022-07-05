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

#include <gtest/gtest.h>
#include "guidance/guidance_worker.hpp"

namespace guidance_test_suite
{

    namespace std_ph = std::placeholders;

    /**
     * GuidanceTestSuite: Test fixture for GuidanceWorker testing
     * Maintains publishers, subscribers, and message tracking for all tests.
     * State is reset between tests to ensure clean results.
     */
    class GuidanceTestSuite : public carma_ros2_utils::CarmaLifecycleNode
    {

    public:
        /*!
        * \brief Constructor for GuidanceTestSuite
        */
        explicit GuidanceTestSuite(const rclcpp::NodeOptions &options) 
            : carma_ros2_utils::CarmaLifecycleNode(options) {}

        // Member for storing the latest Guidance State message
        carma_planning_msgs::msg::GuidanceState latest_state;

        // Subscribers
        carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> state_sub_;

        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) {
            // Setup Subscribers
            state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("state", 5,
                                std::bind(&GuidanceTestSuite::stateCallback, this, std_ph::_1));

            return CallbackReturn::SUCCESS;
        }

        // Callback Function for Guidance State
        void stateCallback(carma_planning_msgs::msg::GuidanceState::UniquePtr msg) {
            latest_state = *msg;
        }

    };

} // guidance_test_suite