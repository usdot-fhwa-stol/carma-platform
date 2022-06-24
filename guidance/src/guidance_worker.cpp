/*
 * Copyright (C) 2018-2022 LEIDOS.
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
#include "guidance/guidance_worker.hpp"

namespace guidance
{
  namespace std_ph = std::placeholders;

  GuidanceWorker::GuidanceWorker(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options),
        gsm_(this->get_node_logging_interface())
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.spin_rate_hz = declare_parameter<double>("spin_rate_hz", config_.spin_rate_hz);
  }

  rcl_interfaces::msg::SetParametersResult GuidanceWorker::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({{"spin_rate_hz", config_.spin_rate_hz}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn GuidanceWorker::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "GuidanceWorker trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("spin_rate_hz", config_.spin_rate_hz);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&GuidanceWorker::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    robot_status_subscriber_ = create_subscription<carma_driver_msgs::msg::RobotEnabled>("robot_status", 5,
                                                              std::bind(&GuidanceWorker::robot_status_cb, this, std_ph::_1));
    route_event_subscriber_ = create_subscription<carma_planning_msgs::msg::RouteEvent>("route_event", 5,
                                                              std::bind(&GuidanceWorker::route_event_cb, this, std_ph::_1));
    vehicle_status_subscriber_ = create_subscription<autoware_msgs::msg::VehicleStatus>("vehicle_status", 5,
                                                              std::bind(&GuidanceWorker::vehicle_status_cb, this, std_ph::_1));

    // Setup publishers
    state_publisher_  = create_publisher<carma_planning_msgs::msg::GuidanceState>("state", 5);

    // Setup service clients
    enable_client_  = create_client<carma_driver_msgs::srv::SetEnableRobotic>("enable_robotic");

    // Setup service servers
    guidance_activate_service_server_  = create_service<carma_planning_msgs::srv::SetGuidanceActive>("set_guidance_active",
                                                            std::bind(&GuidanceWorker::guidance_activation_cb, this, std_ph::_1, std_ph::_2, std_ph::_3));

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn GuidanceWorker::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    // Setup timer
    auto guidance_spin_period_ms = int ((1 / config_.spin_rate_hz) * 1000); // Conversion rom frequency (Hz) to milliseconds time period
    spin_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(guidance_spin_period_ms),
        std::bind(&GuidanceWorker::spin_cb, this));

    // Update Guidance State Machine since node activation by the lifecycle system indicates that all required drivers are ready
    gsm_.onGuidanceInitialized();

    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn GuidanceWorker::handle_on_shutdown(const rclcpp_lifecycle::State &prev_state)
  {
    // Update Guidance State Machine with SHUTDOWN signal
    gsm_.onGuidanceShutdown();
  }

  bool GuidanceWorker::guidance_activation_cb(const std::shared_ptr<rmw_request_id_t>,
                                      const std::shared_ptr<carma_planning_msgs::srv::SetGuidanceActive::Request> req,
                                      std::shared_ptr<carma_planning_msgs::srv::SetGuidanceActive::Response> resp) 
  {
    // Translate message type from GuidanceActiveRequest to SetEnableRobotic
    if(!req->guidance_active)
    {
      auto enable_robotic_request = std::make_shared<carma_driver_msgs::srv::SetEnableRobotic::Request>();
      enable_robotic_request->set = carma_driver_msgs::srv::SetEnableRobotic::Request::DISABLE;
      enable_client_->async_send_request(enable_robotic_request);
    }

    gsm_.onSetGuidanceActive(req->guidance_active);
    resp->guidance_status = (gsm_.getCurrentState() == GuidanceStateMachine::ACTIVE);
    return true;  
  }

  bool GuidanceWorker::spin_cb()
  {
    if(gsm_.shouldCallSetEnableRobotic()) {
      auto req = std::make_shared<carma_driver_msgs::srv::SetEnableRobotic::Request>();
      req->set = carma_driver_msgs::srv::SetEnableRobotic::Request::ENABLE;
      enable_client_->async_send_request(req);
    }

    carma_planning_msgs::msg::GuidanceState state;
    state.state = gsm_.getCurrentState();
    state_publisher_->publish(state);
    return true;
  }

  void GuidanceWorker::route_event_cb(carma_planning_msgs::msg::RouteEvent::UniquePtr msg)
  {
    gsm_.onRouteEvent(move(msg));
  }

  void GuidanceWorker::robot_status_cb(carma_driver_msgs::msg::RobotEnabled::UniquePtr msg)
  {
    gsm_.onRoboticStatus(move(msg));
  }

  void GuidanceWorker::vehicle_status_cb(autoware_msgs::msg::VehicleStatus::UniquePtr msg)
  {
    gsm_.onVehicleStatus(move(msg));
  }

} // guidance

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(guidance::GuidanceWorker)
