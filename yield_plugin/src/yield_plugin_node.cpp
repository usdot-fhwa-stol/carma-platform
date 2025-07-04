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

#include <yield_plugin/yield_plugin_node.hpp>

namespace yield_plugin
{
  namespace std_ph = std::placeholders;

  YieldPluginNode::YieldPluginNode(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options),
        version_id_("v1.0"),
        config_(YieldPluginConfig())
  {
    // Declare parameters
    config_.acceleration_adjustment_factor = declare_parameter<double>("acceleration_adjustment_factor", config_.acceleration_adjustment_factor);
    config_.time_horizon_until_collision_to_commit_to_stop_in_s = declare_parameter<double>("time_horizon_until_collision_to_commit_to_stop_in_s", config_.time_horizon_until_collision_to_commit_to_stop_in_s);
    config_.obstacle_zero_speed_threshold_in_ms = declare_parameter<double>("obstacle_zero_speed_threshold_in_ms", config_.obstacle_zero_speed_threshold_in_ms);
    config_.on_route_vehicle_collision_horizon_in_s = declare_parameter<double>("on_route_vehicle_collision_horizon_in_s", config_.on_route_vehicle_collision_horizon_in_s);
    config_.collision_check_radius_in_m = declare_parameter<double>("collision_check_radius_in_m", config_.collision_check_radius_in_m);
    config_.yield_max_deceleration_in_ms2 = declare_parameter<double>("yield_max_deceleration_in_ms2", config_.yield_max_deceleration_in_ms2);
    config_.min_obj_avoidance_plan_time_in_s = declare_parameter<double>("min_obj_avoidance_plan_time_in_s", config_.min_obj_avoidance_plan_time_in_s);
    config_.minimum_safety_gap_in_meters = declare_parameter<double>("minimum_safety_gap_in_meters", config_.minimum_safety_gap_in_meters);
    config_.max_stop_speed_in_ms= declare_parameter<double>("max_stop_speed_in_ms", config_.max_stop_speed_in_ms);
    config_.enable_cooperative_behavior = declare_parameter< bool>("enable_cooperative_behavior", config_.enable_cooperative_behavior);
    config_.always_accept_mobility_request = declare_parameter< bool>("always_accept_mobility_request", config_.always_accept_mobility_request);
    config_.acceptable_passed_timesteps = declare_parameter<int>("acceptable_passed_timesteps", config_.acceptable_passed_timesteps);
    config_.consecutive_clearance_count_for_passed_obstacles_threshold = declare_parameter<int>("consecutive_clearance_count_for_passed_obstacles_threshold",
      config_.consecutive_clearance_count_for_passed_obstacles_threshold);
    config_.intervehicle_collision_distance_in_m = declare_parameter<double>("intervehicle_collision_distance_in_m", config_.intervehicle_collision_distance_in_m);
    config_.safety_collision_time_gap_in_s = declare_parameter<double>("safety_collision_time_gap_in_s", config_.safety_collision_time_gap_in_s);
    config_.enable_adjustable_gap = declare_parameter<bool>("enable_adjustable_gap", config_.enable_adjustable_gap);
    config_.acceptable_urgency = declare_parameter<int>("acceptable_urgency", config_.acceptable_urgency);
    config_.speed_moving_average_window_size = declare_parameter<double>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    config_.vehicle_length = declare_parameter<double>("vehicle_length", config_.vehicle_length);
    config_.vehicle_height = declare_parameter<double>("vehicle_height", config_.vehicle_height);
    config_.vehicle_width = declare_parameter<double>("vehicle_width", config_.vehicle_width);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);

  }

  carma_ros2_utils::CallbackReturn YieldPluginNode::on_configure_plugin()
  {
    config_ = YieldPluginConfig();

    get_parameter<double>("acceleration_adjustment_factor", config_.acceleration_adjustment_factor);
    get_parameter<double>("time_horizon_until_collision_to_commit_to_stop_in_s", config_.time_horizon_until_collision_to_commit_to_stop_in_s);
    get_parameter<double>("obstacle_zero_speed_threshold_in_ms", config_.obstacle_zero_speed_threshold_in_ms);
    get_parameter<double>("on_route_vehicle_collision_horizon_in_s", config_.on_route_vehicle_collision_horizon_in_s);
    get_parameter<double>("collision_check_radius_in_m", config_.collision_check_radius_in_m);
    get_parameter<double>("yield_max_deceleration_in_ms2", config_.yield_max_deceleration_in_ms2);
    get_parameter<double>("minimum_safety_gap_in_meters", config_.minimum_safety_gap_in_meters);
    get_parameter<double>("max_stop_speed_in_ms", config_.max_stop_speed_in_ms);
    get_parameter<bool>("enable_cooperative_behavior", config_.enable_cooperative_behavior);
    get_parameter<bool>("always_accept_mobility_request", config_.always_accept_mobility_request);
    get_parameter<int>("acceptable_passed_timesteps", config_.acceptable_passed_timesteps);
    get_parameter<int>("consecutive_clearance_count_for_passed_obstacles_threshold", config_.consecutive_clearance_count_for_passed_obstacles_threshold);
    get_parameter<double>("intervehicle_collision_distance_in_m", config_.intervehicle_collision_distance_in_m);
    get_parameter<double>("safety_collision_time_gap_in_s", config_.safety_collision_time_gap_in_s);
    get_parameter<bool>("enable_adjustable_gap", config_.enable_adjustable_gap);
    get_parameter<int>("acceptable_urgency", config_.acceptable_urgency);
    get_parameter<double>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    get_parameter<double>("min_obj_avoidance_plan_time_in_s", config_.min_obj_avoidance_plan_time_in_s);
    get_parameter<double>("vehicle_length", config_.vehicle_length);
    get_parameter<double>("vehicle_height", config_.vehicle_height);
    get_parameter<double>("vehicle_width", config_.vehicle_width);
    get_parameter<std::string>("vehicle_id", config_.vehicle_id);

    RCLCPP_INFO_STREAM(get_logger(), "YieldPlugin Params: " << config_);

    worker_ = std::make_shared<YieldPlugin>(shared_from_this(), get_world_model(), config_,
                                                          [this](auto msg) { mob_resp_pub_->publish(msg); },
                                                          [this](auto msg) { lc_status_pub_->publish(msg); });

     // Publisher
    mob_resp_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityResponse>("outgoing_mobility_response", 1);
    lc_status_pub_ = create_publisher<carma_planning_msgs::msg::LaneChangeStatus>("cooperative_lane_change_status", 10);

    // Subscriber
    mob_request_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityRequest>("incoming_mobility_request", 10, std::bind(&YieldPlugin::mobilityrequest_cb,worker_.get(),std_ph::_1));
    bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("bsm_outbound", 1, std::bind(&YieldPlugin::bsm_cb,worker_.get(),std_ph::_1));
    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 10,
                                                                  [this](const std_msgs::msg::String::SharedPtr msg) {
                                                                    worker_->set_georeference_string(msg->data);
                                                                  });

    // Return success if everything initialized successfully
    external_objects_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>("external_object_predictions", 20,
                                                                                                [this](const carma_perception_msgs::msg::ExternalObjectList::SharedPtr msg) {
                                                                                                  worker_->set_external_objects(msg->objects);
                                                                                                });
    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

    void YieldPluginNode::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t> srv_header,
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    worker_->plan_trajectory_callback(req, resp);
  }

    bool YieldPluginNode::get_availability()
  {
    return true;
  }

  std::string YieldPluginNode::get_version_id()
  {
    return version_id_;
  }

} // yield_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(yield_plugin::YieldPluginNode)
