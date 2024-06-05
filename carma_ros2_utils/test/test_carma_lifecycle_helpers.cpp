/*
 * Copyright (C) 2021 LEIDOS.
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
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "std_srvs/srv/empty.hpp"
#include "boost/core/ignore_unused.hpp"
#include "boost/optional.hpp"
using std_msec = std::chrono::milliseconds;

namespace test_2
{
    class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
    {            
            
        public:

            int my_param_= 0;

            int get_my_param()
            {
                return my_param_;
            }

            int set_my_param(int param)
            {
                int temp = my_param_;
                my_param_ = param;
                return temp;
            }

            //using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
            CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
                : CarmaLifecycleNode(options)
            {
            }

            ~CarmaLifecycleNodeTest(){};

            carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
            {

                RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Configured!");
                return CallbackReturn::SUCCESS;
            }

        
    };
}



// To evaluate errors in primary states we'll need to add some sort of
// Test to evaluate the nominal transition cases
TEST(CARMALifecycleNode, UpdateParams)
{
    rclcpp::NodeOptions options;
    auto node = std::make_shared<test_2::CarmaLifecycleNodeTest>(options);

    // Clean Transitions
    node->configure();
    node->activate();

    // Evaluate parameter callbacks
    auto param_handle = node->add_on_set_parameters_callback(
        [node](auto param_vec) {
            
            auto error = node->update_params<int>({ {"my_param", node->my_param_} }, param_vec);

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !error;

            if (error) {
                result.reason = error.get();
            }

            return result;
        });

    // Trigger the callback
    rclcpp::Parameter param("my_param", static_cast<int>(1));

    auto result = param_handle->callback( { param } );

    ASSERT_EQ(result.successful, true);
    ASSERT_EQ(node->my_param_, 1);

    auto node_2 = std::make_shared<test_2::CarmaLifecycleNodeTest>(options);

    // Clean Transitions
    node_2->configure();
    node_2->activate();
    
    // Evaluate parameter callbacks with functions
    auto param_handle_2 = node_2->add_on_set_parameters_callback(
        [node_2](auto param_vec) {
            
            auto error = node_2->update_params<int>({ {"my_param", std::bind(&test_2::CarmaLifecycleNodeTest::set_my_param, node_2, std::placeholders::_1) } }, param_vec);

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !error;

            if (error) {
                result.reason = error.get();
            }

            return result;
        });

    // Trigger the callback
    rclcpp::Parameter param_2("my_param", static_cast<int>(2));

    result = param_handle_2->callback( { param_2 } );

    ASSERT_EQ(result.successful, true);
    ASSERT_EQ(node_2->my_param_, 2);



    auto node_3 = std::make_shared<test_2::CarmaLifecycleNodeTest>(options);

    // Clean Transitions
    node_3->configure();
    node_3->activate();
    
    // Evaluate parameter with mismatched types
    auto param_handle_3 = node_3->add_on_set_parameters_callback(
        [node_3](auto param_vec) {
            
            auto error = node_3->update_params<int>({ {"my_param", std::bind(&test_2::CarmaLifecycleNodeTest::set_my_param, node_3, std::placeholders::_1) } }, param_vec);

            rcl_interfaces::msg::SetParametersResult result;
            result.successful = !error;

            if (error) {
                result.reason = error.get();
            }

            return result;
        });

    // Trigger the callback
    rclcpp::Parameter param_3("my_param", "a");

    result = param_handle_3->callback( { param_3 } );

    ASSERT_EQ(result.successful, false);
    ASSERT_EQ(node_3->my_param_, 0);
    

}
