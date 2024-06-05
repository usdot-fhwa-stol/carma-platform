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

/**
 * This file is loosely based on the reference architecture developed by OSRF for Leidos located here
 * https://github.com/mjeronimo/carma2/blob/master/carma_utils/carma_utils/src/carma_lifecycle_node.cpp
 *
 * That file has the following license and some code snippets from it may be present in this file as well and are under the same license.
 *
 * Copyright 2021 Open Source Robotics Foundation, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "carma_ros2_utils/carma_lifecycle_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace carma_ros2_utils
{

  CarmaLifecycleNode::CarmaLifecycleNode(const rclcpp::NodeOptions &options)
      : rclcpp_lifecycle::LifecycleNode("carma_node", "", options)
  {
    RCLCPP_INFO(get_logger(), "CarmaLifecycleNode node launched, waiting on state transition requests");

    // NOTE: When creating this callback group it was not immediately clear if it should go in on_configure or the constructor
    //       Further usage of callback groups may elucidate this in the future
    service_callback_group_ = get_node_base_interface()->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  }

  CarmaLifecycleNode::~CarmaLifecycleNode()
  {
    RCLCPP_INFO(get_logger(), "Destroying");

    // In case this lifecycle node wasn't properly shut down, do it here
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_WARN(get_logger(), "Destructor called while in ACTIVE state. Attempting to cleanup");
      on_deactivate(get_current_state());
      on_cleanup(get_current_state());
    }
  }

  CallbackReturn
  CarmaLifecycleNode::on_configure(const rclcpp_lifecycle::State &prev_state)
  {

    // NOTE: Currently, intra-process comms must be disabled for a publisher that is transient_local: https://github.com/ros2/rclcpp/issues/1753
    rclcpp::PublisherOptions intra_proc_disabled;
    intra_proc_disabled.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;

    // Create a publisher that will send previously published messages to late-joining subscribers ONLY If the subscriber is transient_local too
    auto pub_qos_transient_local = rclcpp::QoS(rclcpp::KeepLast(5)); // A publisher with this QoS will store the last 5 messages that it has sent on the topic
    pub_qos_transient_local.transient_local();

    system_alert_pub_ = create_publisher<carma_msgs::msg::SystemAlert>(
        system_alert_topic_, pub_qos_transient_local, intra_proc_disabled);

    return handle_on_configure(prev_state);
  }

  CallbackReturn
  CarmaLifecycleNode::on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    activate_publishers();
    return handle_on_activate(prev_state);
  }

  CallbackReturn
  CarmaLifecycleNode::on_deactivate(const rclcpp_lifecycle::State &prev_state)
  {
    if (caught_exception_)
    { // Handling exceptions from Active state will result in a deactivate call which will then return an error to transition us to on_error();
      return CallbackReturn::ERROR;
    }

    deactivate_publishers();
    return handle_on_deactivate(prev_state);
  }

  CallbackReturn
  CarmaLifecycleNode::on_cleanup(const rclcpp_lifecycle::State &prev_state)
  {
    auto result = handle_on_cleanup(prev_state);
    cleanup_publishers();
    cleanup_timers();
    return result;
  }

  CallbackReturn
  CarmaLifecycleNode::on_error(const rclcpp_lifecycle::State &prev_state)
  {
    std::string error_string;
    if (caught_exception_)
    {

      error_string = caught_exception_.get();
      RCLCPP_ERROR_STREAM(get_logger(), caught_exception_.get());
      caught_exception_ = boost::none;
    }
    else
    {
      error_string = "Exception occurred during lifecycle state transitions. Look for 'Caught exception' in the logs for details.";
      RCLCPP_ERROR_STREAM(get_logger(), error_string);

    }

    try
    {

      send_error_alert_msg_for_string(error_string);
      RCLCPP_ERROR_STREAM(get_logger(), "Sent on_error system alert");
    }
    catch (const std::exception &e)
    {

      RCLCPP_ERROR_STREAM(get_logger(), "Failed to send on_error system alert. Forcing shutdown.");
      return CallbackReturn::FAILURE;
    }

    // Call the user error handling before clean up of the publishers to allow them to publish if needed
    auto return_val = handle_on_error(prev_state, error_string);

    cleanup_publishers();
    cleanup_timers();
    return  return_val;
  }

  CallbackReturn
  CarmaLifecycleNode::on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    auto result = handle_on_shutdown(prev_state);
    cleanup_publishers();
    cleanup_timers();
    return result;
  }

  std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode>
  CarmaLifecycleNode::shared_from_this()
  {
    return std::static_pointer_cast<carma_ros2_utils::CarmaLifecycleNode>(
        rclcpp_lifecycle::LifecycleNode::shared_from_this());
  }

  void
  CarmaLifecycleNode::publish_system_alert(const carma_msgs::msg::SystemAlert &msg)
  {
    carma_msgs::msg::SystemAlert pub_msg = msg;

    if (!get_node_base_interface())
      throw std::runtime_error("get_node_base_interface() returned null pointer. May indicate ROS2 bug.");

    pub_msg.source_node = get_node_base_interface()->get_fully_qualified_name(); // The the source name for the message

    if (!system_alert_pub_->is_activated()) {
      RCLCPP_WARN_STREAM(get_logger(), "Sending SystemAlert likely failed as publisher is deactivated.");
    }

    system_alert_pub_->publish(pub_msg);
  }

  void CarmaLifecycleNode::send_error_alert_msg_for_string(const std::string &alert_string)
  {
    carma_msgs::msg::SystemAlert alert_msg;
    alert_msg.type = carma_msgs::msg::SystemAlert::FATAL;
    alert_msg.description = alert_string;

    publish_system_alert(alert_msg); // Notify the rest of the system
  }

  void CarmaLifecycleNode::handle_primary_state_exception(const std::exception &e)
  {
    std::lock_guard<std::mutex> lock(exception_mutex_);

    rclcpp_lifecycle::State state_at_exception = get_current_state();

    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) // If the exception was caught in the ACTIVE state we can try to gracefully fail to on_error, by transitioning to deactivate and then throwing an exception
    {
      std::string error_msg = "Uncaught Exception from node: " + std::string(get_name()) + " exception: " + e.what() + " while in ACTIVE state.";
      caught_exception_ = error_msg;
      deactivate();
    }
    else
    {
      std::string error_msg = "Uncaught Exception from node: " + std::string(get_name()) + " caught exception: " + e.what() + " while in unsupported exception handling state: " + std::to_string(get_current_state().id());

      RCLCPP_ERROR_STREAM(get_logger(), error_msg); // Log exception

      try
      {

        send_error_alert_msg_for_string(error_msg);
        RCLCPP_ERROR_STREAM(get_logger(), "Sent handle_primary_state_exception system alert");
      }
      catch (const std::exception &e)
      {

        RCLCPP_ERROR_STREAM(get_logger(), "Failed to send handle_primary_state_exception system alert. Forcing shutdown.");
      }

      shutdown(); // Shutdown as an exception while in a non-active state is very likely not recoverable and notification to the larger system may not work
    }
  }

  void CarmaLifecycleNode::activate_publishers()
  {
    for (auto pub : lifecycle_publishers_)
    {
      if (!pub)
        continue;
      pub->on_activate();
    }
  }
  void CarmaLifecycleNode::deactivate_publishers()
  {
    for (auto pub : lifecycle_publishers_)
    {
      if (!pub)
        continue;
      pub->on_deactivate();
    }
  }
  void CarmaLifecycleNode::cleanup_publishers()
  {
    for (auto pub : lifecycle_publishers_)
    {
      pub.reset();
    }
  }
  void CarmaLifecycleNode::cleanup_timers()
  {
    for (auto timer : timers_)
    {
      if (timer) {
        timer->cancel();
      }
      timer.reset();
    }
  }

  rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr
  CarmaLifecycleNode::add_on_set_parameters_callback(
      rclcpp_lifecycle::LifecycleNode::OnParametersSetCallbackType callback)
  {

    auto handle = rclcpp_lifecycle::LifecycleNode::add_on_set_parameters_callback(
        [callback = std::move(callback), this](auto params)
        {
          try
          {

            return callback(params);
          }
          catch (const std::exception &e)
          {

            handle_primary_state_exception(e);

            rcl_interfaces::msg::SetParametersResult msg;
            msg.successful = false;
            msg.reason = e.what();

            return msg;
          }
        });

    param_callback_handles_.push_back(handle);

    return handle;
  }

  CallbackReturn CarmaLifecycleNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn CarmaLifecycleNode::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn CarmaLifecycleNode::handle_on_deactivate(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn CarmaLifecycleNode::handle_on_cleanup(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS;
  }
  CallbackReturn CarmaLifecycleNode::handle_on_error(const rclcpp_lifecycle::State &, const std::string &)
  {
    return CallbackReturn::FAILURE; // By default an error will take us into the finalized sate.
  }
  CallbackReturn CarmaLifecycleNode::handle_on_shutdown(const rclcpp_lifecycle::State &)
  {
    return CallbackReturn::SUCCESS; // By default shutdown will take us into the finalized state.
  }

} // namespace carma_ros2_utils
