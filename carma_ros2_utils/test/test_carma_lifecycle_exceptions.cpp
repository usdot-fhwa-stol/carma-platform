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

namespace test_1
{
    // This is a test node to support unit tests for the carma_lifecycle_node
    // NOTE: To make unit testing easier this node by default returns to unconfigured from ErrorProcessing unless an exception occurs in ErrorProcessing
    //       This is different from the default CarmaLifecycleNode behavior which shuts the node down on exceptions
    class CarmaLifecycleNodeTest : public carma_ros2_utils::CarmaLifecycleNode
    {
    public:
        enum class TransitionState
        {
            Configuring,
            CleaningUp,
            ShuttingDown,
            Activating,
            Deactivating,
            ErrorProcessing
        };

        boost::optional<TransitionState> exception_state_; // Flag to throw an exception in the provided transition state
        bool exception_on_error_ = false;                  // Extra flag for the error processing state so that we can enter it then trigger a new exception

        void mark_for_exception(TransitionState state)
        {
            exception_state_ = state;
        }

        void mark_error_processing_exception(bool status)
        {
            exception_on_error_ = status;
        }

        void throw_if_marked(TransitionState state)
        {
            RCLCPP_INFO(get_logger(), "Checking for exception");

            if (!exception_state_)
                return;

            if (exception_state_.get() == state)
            {
                throw std::runtime_error("Marked Exception");
            }
        }

        //using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
        CarmaLifecycleNodeTest(const rclcpp::NodeOptions &options)
            : CarmaLifecycleNode(options)
        {
        }

        ~CarmaLifecycleNodeTest(){};

        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State & /*state*/) override
        {
            throw_if_marked(TransitionState::Configuring);

            RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Configured!");
            return CallbackReturn::SUCCESS;
        }

        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State & /*state*/) override
        {
            throw_if_marked(TransitionState::Activating);

            RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Activated!");
            return CallbackReturn::SUCCESS;
        }

        carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
        {
            throw_if_marked(TransitionState::Deactivating);

            RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Deactivated!");
            return CallbackReturn::SUCCESS;
        }

        carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
        {
            throw_if_marked(TransitionState::CleaningUp);

            RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Cleanup!");
            return CallbackReturn::SUCCESS;
        }

        carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
        {
            throw_if_marked(TransitionState::ShuttingDown);

            RCLCPP_INFO(get_logger(), "CARMA Lifecycle Test node is Shutdown!");
            return CallbackReturn::SUCCESS;
        }

        carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State & /*state*/, const std::string &exception_string) override
        {
            throw_if_marked(TransitionState::ErrorProcessing);

            if (exception_on_error_)
                throw std::runtime_error("Error processin exception");

            RCLCPP_INFO_STREAM(get_logger(), "CARMA Lifecycle Test node is encountered an error! Error: " << exception_string);
            return CallbackReturn::SUCCESS;
        }
    };
}

// This test evaluates the error handling behavior of the transition states
TEST(CARMALifecycleNode, TransitionExceptions)
{
    rclcpp::NodeOptions options;
    auto node = std::make_shared<test_1::CarmaLifecycleNodeTest>(options);

    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::Configuring);

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    // Trigger transition and check state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->configure().id());

    // Move exception to activating state
    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::Activating);

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());

    // Due to the exception we will end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->activate().id());

    // Move exception to deactivate state
    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::Deactivating);

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());

    // Due to the exception we will end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->deactivate().id());

    // Move exception to cleanup
    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::CleaningUp);
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());

    // Due to exception we will end up back in unconfigured
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->cleanup().id());

    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::ShuttingDown);

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());

    // Due to exception we will end up back in unconfigured
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->shutdown().id());

    node->mark_error_processing_exception(true);
    node->mark_for_exception(test_1::CarmaLifecycleNodeTest::TransitionState::Configuring);

    // Due to exception in the error processing state we will end up in finalized
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, node->configure().id());
}

// Test to evaluate the nominal transition cases
TEST(CARMALifecycleNode, CleanTransitions)
{
    rclcpp::NodeOptions options;
    auto node = std::make_shared<test_1::CarmaLifecycleNodeTest>(options);

    // Clean Transitions

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    node->configure();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->get_current_state().id());

    node->activate();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->get_current_state().id());

    node->deactivate();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->get_current_state().id());

    node->cleanup();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    node->shutdown();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, node->get_current_state().id());
}

// To evaluate errors in primary states we'll need to add some sort of
// Test to evaluate the nominal transition cases
TEST(CARMALifecycleNode, PrimaryStateErrors)
{
    rclcpp::NodeOptions options;
    auto node = std::make_shared<test_1::CarmaLifecycleNodeTest>(options);

    // Clean Transitions

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    node->configure();

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->get_current_state().id());

    node->activate();

    // Evaluate timers
    auto exception_timer = node->create_timer(
        node->get_clock(),
        rclcpp::Duration(std::chrono::nanoseconds{10}),
        []()
        {
            throw std::runtime_error("Timer exception");
        });


    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->get_current_state().id());

    // Trigger the callback
    exception_timer->execute_callback();

    // Due to exception we should end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    exception_timer->cancel(); // Cleanup timer

    // Bring node back to active state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());

    // Evaluate wall timers
    auto excep_wall_timer = node->create_wall_timer(
        std::chrono::milliseconds(10000),
        []()
        {
            throw std::runtime_error("Wall Timer exception");
        });

    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->get_current_state().id());

    // Trigger the callback
    excep_wall_timer->execute_callback();

    // Due to exception we should end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    excep_wall_timer->cancel(); // Cleanup timer

    // Bring node back to active state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());

    // Evaluate subscription
    auto exception_sub = node->create_subscription<carma_msgs::msg::SystemAlert>(
        "/exception_topic",
        1,
        [](std::unique_ptr<carma_msgs::msg::SystemAlert>) {
            throw std::runtime_error("Subscription exception");
        });

    auto msg = exception_sub->create_message();
    rmw_message_info_t info;
    exception_sub->handle_message(msg, info); // Trigger the callback

    // Due to exception we should end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    // Bring node back to active state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());

    // Evaluate parameter callbacks
    auto param_handle = node->add_on_set_parameters_callback(
        [](auto) {
            rcl_interfaces::msg::SetParametersResult result;
            throw std::runtime_error("Parameter exception");
            return result;
        });

    // Trigger the callback
    param_handle->callback({});

    // Due to exception we should end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    // Bring node back to active state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, node->activate().id());


    auto service = node->create_service<std_srvs::srv::Empty>(
        "/exception_service",
        [](auto, auto, auto){
            throw std::runtime_error("Service exception");
        });

    auto header = service->create_request_header();
    auto request = service->create_request();

    service->handle_request(header, request);

    // Due to exception we should end up back in the unconfigured state
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED, node->get_current_state().id());

    // Evaluate Exception in non-active primary state
    // Bring node back to inactive state to trigger non-active state error
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, node->configure().id());

    // Force the service callback to trigger the exception
    service->handle_request(header, request);
    ASSERT_EQ(lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, node->get_current_state().id());


}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    // initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    // shutdown ROS
    rclcpp::shutdown();

    return success;
}
