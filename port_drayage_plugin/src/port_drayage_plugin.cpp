/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include "port_drayage_plugin/port_drayage_plugin.hpp"

namespace port_drayage_plugin
{
  namespace std_ph = std::placeholders;

  PortDrayagePlugin::PortDrayagePlugin(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options),
      pdw_(get_node_logging_interface(), get_clock(),
           std::bind(&PortDrayagePlugin::publishMobilityOperation, this, std_ph::_1),
           std::bind(&PortDrayagePlugin::publishUIInstructions, this, std_ph::_1),
           std::bind(&PortDrayagePlugin::callSetActiveRouteClient, this, std_ph::_1))
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.cmv_id = declare_parameter<std::string>("vehicle_id", config_.cmv_id);
    config_.cargo_id = declare_parameter<std::string>("cargo_id", config_.cargo_id);
    config_.enable_port_drayage = declare_parameter<bool>("enable_port_drayage", config_.enable_port_drayage);
    config_.starting_at_staging_area = declare_parameter<bool>("starting_at_staging_area", config_.starting_at_staging_area);
  }

  rcl_interfaces::msg::SetParametersResult PortDrayagePlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<std::string>({
      {"vehicle_id", config_.cmv_id},
      {"cargo_id", config_.cargo_id}}, parameters);
    auto error_2 = update_params<bool>({
      {"enable_port_drayage", config_.enable_port_drayage},
      {"starting_at_staging_area", config_.starting_at_staging_area}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    if (result.successful) {
      // Set PortDrayageWorker parameters
      pdw_.setVehicleID(config_.cmv_id);
      pdw_.setCargoID(config_.cargo_id);
      pdw_.setEnablePortDrayageFlag(config_.enable_port_drayage);
      pdw_.setStartingAtStagingAreaFlag(config_.starting_at_staging_area);
    }

    return result;
  }

  carma_ros2_utils::CallbackReturn PortDrayagePlugin::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "port_drayage_plugin trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<std::string>("vehicle_id", config_.cmv_id);
    get_parameter<std::string>("cargo_id", config_.cargo_id);
    get_parameter<bool>("enable_port_drayage", config_.enable_port_drayage);
    get_parameter<bool>("starting_at_staging_area", config_.starting_at_staging_area);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&PortDrayagePlugin::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    pose_subscriber_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 5,
                                                              std::bind(&PortDrayageWorker::onNewPose, &pdw_, std_ph::_1));
    inbound_mobility_operation_subscriber_ = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 5,
                                                              std::bind(&PortDrayageWorker::onInboundMobilityOperation, &pdw_, std_ph::_1));
    guidance_state_subscriber_ = create_subscription<carma_planning_msgs::msg::GuidanceState> ("guidance_state", 5,
                                                              std::bind(&PortDrayageWorker::onGuidanceState, &pdw_, std_ph::_1));
    route_event_subscriber_ = create_subscription<carma_planning_msgs::msg::RouteEvent>("route_event", 5,
                                                              std::bind(&PortDrayageWorker::onRouteEvent, &pdw_, std_ph::_1));
    georeference_subscriber_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&PortDrayageWorker::onNewGeoreference, &pdw_, std_ph::_1));

    // Setup publishers
    outbound_mobility_operations_publisher_ = create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 5);
    ui_instructions_publisher_ = create_publisher<carma_msgs::msg::UIInstructions>("ui_instructions", 5);

    // Setup service clients
    set_active_route_client_ = create_client<carma_planning_msgs::srv::SetActiveRoute>("set_active_route");

    // Set PortDrayageWorker variables
    pdw_.setVehicleID(config_.cmv_id);
    pdw_.setCargoID(config_.cargo_id);
    pdw_.setEnablePortDrayageFlag(config_.enable_port_drayage);
    pdw_.setStartingAtStagingAreaFlag(config_.starting_at_staging_area);

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  bool PortDrayagePlugin::callSetActiveRouteClient(std::shared_ptr<carma_planning_msgs::srv::SetActiveRoute::Request> req)
  {
    // Send request to set_active_route service 
    auto route_result = set_active_route_client_->async_send_request(req);

    // Wait for response from service call
    auto future_status = route_result.wait_for(std::chrono::milliseconds(100));

    if (future_status == std::future_status::ready) {
      if(route_result.get()->error_status == carma_planning_msgs::srv::SetActiveRoute::Response::NO_ERROR){
        RCLCPP_DEBUG_STREAM(get_logger(), "Route Generation succeeded for Set Active Route service call.");
        return true;
      }
      else{
        RCLCPP_DEBUG_STREAM(get_logger(), "Route Generation failed for Set Active Route service call.");
        return false;
      }
    }
    else {
      // No response was received after making the service call
      RCLCPP_DEBUG_STREAM(get_logger(), "Set Active Route service call was not successful.");
      return false;
    }

  }

  void PortDrayagePlugin::publishMobilityOperation(const carma_v2x_msgs::msg::MobilityOperation& msg)
  {
    outbound_mobility_operations_publisher_->publish(msg);
  }

  void PortDrayagePlugin::publishUIInstructions(const carma_msgs::msg::UIInstructions& msg)
  {
    ui_instructions_publisher_->publish(msg);
  }

} // port_drayage_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(port_drayage_plugin::PortDrayagePlugin)
