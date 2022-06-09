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
      : carma_ros2_utils::CarmaLifecycleNode(options),
        tf2_buffer_(this->get_clock()),
        wml_(this->get_node_base_interface(), this->get_node_logging_interface(),
             this->get_node_topics_interface(), this->get_node_parameters_interface()),
        rg_worker_(tf2_buffer_)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.max_crosstrack_error = declare_parameter<double>("max_crosstrack_error", config_.max_crosstrack_error);
    config_.destination_downtrack_range = declare_parameter<double>("destination_downtrack_range", config_.destination_downtrack_range);
    config_.route_spin_rate = declare_parameter<double>("route_spin_rate", config_.route_spin_rate);
    config_.cte_max_count = declare_parameter<int>("cte_max_count", config_.cte_max_count);
    config_.route_file_path = declare_parameter<std::string>("route_file_path", config_.route_file_path);
  }

  rcl_interfaces::msg::SetParametersResult Route::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({
      {"max_crosstrack_error", config_.max_crosstrack_error},
      {"destination_downtrack_range", config_.destination_downtrack_range}, 
      {"route_spin_rate", config_.route_spin_rate}}, parameters);
    auto error_2 = update_params<int>({{"cte_max_count", config_.cte_max_count}}, parameters);
    auto error_3 = update_params<std::string>({{"route_file_path", config_.route_file_path}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2 && !error_3;

    return result;
  }

  carma_ros2_utils::CallbackReturn Route::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Route trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("max_crosstrack_error", config_.max_crosstrack_error);
    get_parameter<double>("destination_downtrack_range", config_.destination_downtrack_range);
    get_parameter<double>("route_spin_rate", config_.route_spin_rate);
    get_parameter<int>("cte_max_count", config_.cte_max_count);
    get_parameter<std::string>("route_file_path", config_.route_file_path);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Route::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 1,
                                                              std::bind(&RouteGeneratorWorker::twistCb, &rg_worker_, std_ph::_1));
    geo_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&RouteGeneratorWorker::georeferenceCb, &rg_worker_, std_ph::_1));

    // Setup publishers
    route_pub_ = create_publisher<carma_planning_msgs::msg::Route>("route", 1);
    route_state_pub_ = create_publisher<carma_planning_msgs::msg::RouteState>("route_state", 1);
    route_event_pub_ = create_publisher<carma_planning_msgs::msg::RouteEvent>("route_event", 1);
    route_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("route_marker", 1);

    // Setup service servers
    get_available_route_srv_ = create_service<carma_planning_msgs::srv::GetAvailableRoutes>("get_available_routes",
                                                            std::bind(&RouteGeneratorWorker::getAvailableRouteCb, &rg_worker_, std_ph::_1, std_ph::_2, std_ph::_3));
    set_active_route_srv_ = create_service<carma_planning_msgs::srv::SetActiveRoute>("set_active_route",
                                                            std::bind(&RouteGeneratorWorker::setActiveRouteCb, &rg_worker_, std_ph::_1, std_ph::_2, std_ph::_3));
    abort_active_route_srv_ = create_service<carma_planning_msgs::srv::AbortActiveRoute>("abort_active_route",
                                                            std::bind(&RouteGeneratorWorker::abortActiveRouteCb, &rg_worker_, std_ph::_1, std_ph::_2, std_ph::_3));

    // Set world model pointer from wm listener
    wm_ = wml_.getWorldModel();
    wml_.enableUpdatesWithoutRouteWL();

    // Configure route generator worker parameters
    rg_worker_.setClock(get_clock());
    rg_worker_.setLoggerInterface(get_node_logging_interface());
    rg_worker_.setWorldModelPtr(wm_);
    rg_worker_.setReroutingChecker(std::bind(&carma_wm::WMListener::checkIfReRoutingNeededWL, &wml_));
    rg_worker_.setDowntrackDestinationRange(config_.destination_downtrack_range);
    rg_worker_.setCrosstrackErrorDistance(config_.max_crosstrack_error);
    rg_worker_.setCrosstrackErrorCountMax(config_.cte_max_count);
    rg_worker_.setPublishers(route_event_pub_, route_state_pub_, route_pub_, route_marker_pub_);
    rg_worker_.initializeBumperTransformLookup();

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn Route::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    rg_worker_.setRouteFilePath(config_.route_file_path);

    // Timer for route generator worker's spin callback
    auto rg_worker_spin_period_ms = int ((1 / config_.route_spin_rate) * 1000); // Conversion from frequency (Hz) to milliseconds time period
    spin_timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(rg_worker_spin_period_ms),
                          std::bind(&RouteGeneratorWorker::spinCallback, &rg_worker_));

    return CallbackReturn::SUCCESS;
  }

} // route

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(route::Route)
