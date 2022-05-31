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
#include "route/route_node.hpp"

namespace route
{
  namespace std_ph = std::placeholders;

  Route::Route(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameterstrajectraj
    config_.max_crosstrack_error = declare_parameter<double>("max_crosstrack_error", config_.max_crosstrack_error);
    config_.destination_downtrack_range = declare_parameter<double>("destination_downtrack_range", config_.destination_downtrack_range);
    config_.cte_max_count = declare_parameter<int>("cte_max_count", config_.cte_max_count);
  }

  rcl_interfaces::msg::SetParametersResult Route::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({
      {"max_crosstrack_error", config_.max_crosstrack_error}}
      {"destination_downtrack_range", config_.destination_downtrack_range}, parameters);
    auto error_2 = update_params<int>({{"cte_max_count", config_.cte_max_count}}, parameters)

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    return result;
  }

  carma_ros2_utils::CallbackReturn Route::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<std::string>("example_param", config_.example_param);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Route::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    example_sub_ = create_subscription<std_msgs::msg::String>("example_input_topic", 10,
                                                              std::bind(&Route::example_callback, this, std_ph::_1));

    // Setup publishers
    example_pub_ = create_publisher<std_msgs::msg::String>("example_output_topic", 10);

    // Setup service clients
    example_client_ = create_client<std_srvs::srv::Empty>("example_used_service");

    // Setup service servers
    example_service_ = create_service<std_srvs::srv::Empty>("example_provided_service",
                                                            std::bind(&Route::example_service_callback, this, std_ph::_1, std_ph::_2, std_ph::_3));

    // Setup timers
    // NOTE: You will not be able to actually publish until in the ACTIVE state 
    // so it may often make more sense for timers to be created in handle_on_activate
    example_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(1000),
        std::bind(&Route::example_timer_callback, this));

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  // Parameter names not shown to prevent unused compile warning. The user may add them back
  void Route::example_service_callback(const std::shared_ptr<rmw_request_id_t>,
                                      const std::shared_ptr<std_srvs::srv::Empty::Request>,
                                      std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    RCLCPP_INFO(  get_logger(), "Example service callback");
  }

  void Route::example_timer_callback()
  {
    RCLCPP_DEBUG(get_logger(), "Example timer callback");
    std_msgs::msg::String msg;
    msg.data = "Hello World!";
    example_pub_->publish(msg);
  }

  void Route::example_callback(std_msgs::msg::String::UniquePtr msg)
  {
    RCLCPP_INFO_STREAM(  get_logger(), "example_sub_ callback called with value: " << msg->data);
  }

} // route

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(route::Route)
