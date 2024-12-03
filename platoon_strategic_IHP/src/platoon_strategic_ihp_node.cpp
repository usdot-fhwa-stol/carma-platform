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
#include "platoon_strategic_ihp/platoon_strategic_plugin_node_ihp.h"
#include <carma_ros2_utils/timers/TimerFactory.hpp>
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>

namespace platoon_strategic_ihp
{
  namespace std_ph = std::placeholders;

  Node::Node(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::StrategicPlugin(options)
  {
    // Create initial config
    config_ = PlatoonPluginConfig();

    // Declare parameters

    declare_parameter<bool>("allowCutinJoin", config_.allowCutinJoin);
    declare_parameter<bool>("test_cutin_join", config_.test_cutin_join);
    declare_parameter<bool>("test_front_join", config_.test_front_join);

    declare_parameter<int>("maxPlatoonSize", config_.maxPlatoonSize);
    declare_parameter<int>("algorithmType", config_.algorithmType);
    declare_parameter<int>("statusMessageInterval", config_.statusMessageInterval);
    declare_parameter<int>("infoMessageInterval", config_.infoMessageInterval);
    declare_parameter<int>("join_index", config_.join_index);

    declare_parameter<double>("timeHeadway", config_.timeHeadway);
    declare_parameter<double>("standStillHeadway", config_.standStillHeadway);
    declare_parameter<double>("maxAllowedJoinTimeGap", config_.maxAllowedJoinTimeGap);
    declare_parameter<double>("maxAllowedJoinGap", config_.maxAllowedJoinGap);
    declare_parameter<double>("minAllowedJoinGap", config_.minAllowedJoinGap);
    declare_parameter<double>("desiredJoinTimeGap", config_.desiredJoinTimeGap);
    declare_parameter<double>("desiredJoinGap", config_.desiredJoinGap);
    declare_parameter<double>("waitingStateTimeout", config_.waitingStateTimeout);
    declare_parameter<double>("cmdSpeedMaxAdjustment", config_.cmdSpeedMaxAdjustment);
    declare_parameter<double>("minAllowableHeadaway", config_.minAllowableHeadaway);
    declare_parameter<double>("maxAllowableHeadaway", config_.maxAllowableHeadaway);
    declare_parameter<double>("headawayStableLowerBond", config_.headawayStableLowerBond);
    declare_parameter<double>("headawayStableUpperBond", config_.headawayStableUpperBond);
    declare_parameter<double>("minCutinGap", config_.minCutinGap);
    declare_parameter<double>("maxCutinGap", config_.maxCutinGap);
    declare_parameter<double>("maxCrosstrackError", config_.maxCrosstrackError);
    declare_parameter<double>("minPlatooningSpeed", config_.minPlatooningSpeed);
    declare_parameter<double>("significantDTDchange", config_.significantDTDchange);
    declare_parameter<double>("longitudinalCheckThresold", config_.longitudinalCheckThresold);
    declare_parameter<double>("vehicle_length", config_.vehicleLength);
    declare_parameter<std::string>("vehicle_id", config_.vehicleID);
  }

  rcl_interfaces::msg::SetParametersResult Node::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {

    auto error = update_params<bool>({
      {"test_front_join", config_.test_front_join},
      {"allowCutinJoin", config_.allowCutinJoin},
      {"test_cutin_join", config_.test_cutin_join}
    }, parameters);
    
    auto error2 = update_params<int>({
      {"maxPlatoonSize", config_.maxPlatoonSize},
      {"algorithmType", config_.algorithmType},
      {"statusMessageInterval", config_.statusMessageInterval},
      {"infoMessageInterval", config_.infoMessageInterval},
      {"join_index", config_.join_index}
    }, parameters);

    auto error3 = update_params<double>({
      {"timeHeadway", config_.timeHeadway},
      {"standStillHeadway", config_.standStillHeadway},
      {"maxAllowedJoinTimeGap", config_.maxAllowedJoinTimeGap},
      {"maxAllowedJoinGap", config_.maxAllowedJoinGap},
      {"minAllowedJoinGap", config_.minAllowedJoinGap},
      {"desiredJoinTimeGap", config_.desiredJoinTimeGap},
      {"desiredJoinGap", config_.desiredJoinGap},
      {"waitingStateTimeout", config_.waitingStateTimeout},
      {"cmdSpeedMaxAdjustment", config_.cmdSpeedMaxAdjustment},
      {"minAllowableHeadaway", config_.minAllowableHeadaway},
      {"maxAllowableHeadaway", config_.maxAllowableHeadaway},
      {"headawayStableLowerBond", config_.headawayStableLowerBond},
      {"headawayStableUpperBond", config_.headawayStableUpperBond},
      {"minCutinGap", config_.minCutinGap},
      {"maxCutinGap", config_.maxCutinGap},
      {"maxCrosstrackError", config_.maxCrosstrackError},
      {"minPlatooningSpeed", config_.minPlatooningSpeed},
      {"significantDTDchange", config_.significantDTDchange},
      {"longitudinalCheckThresold", config_.longitudinalCheckThresold}
      // vehicle_length and id excluded since they should not be updated at runtime
    }, parameters);


    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error2 && !error3;

    if (result.successful && worker_)
    {
      worker_->setConfig(config_);
    }

    return result;
  }

  carma_ros2_utils::CallbackReturn Node::on_configure_plugin()
  {
    // Reset config
    config_ = PlatoonPluginConfig();

    // Load parameters
    get_parameter<bool>("test_front_join", config_.test_front_join);
    get_parameter<bool>("allowCutinJoin", config_.allowCutinJoin);
    get_parameter<bool>("test_cutin_join", config_.test_cutin_join);

    get_parameter<int>("maxPlatoonSize", config_.maxPlatoonSize);
    get_parameter<int>("algorithmType", config_.algorithmType);
    get_parameter<int>("statusMessageInterval", config_.statusMessageInterval);
    get_parameter<int>("infoMessageInterval", config_.infoMessageInterval);
    get_parameter<int>("join_index", config_.join_index);

    get_parameter<double>("timeHeadway", config_.timeHeadway);
    get_parameter<double>("standStillHeadway", config_.standStillHeadway);
    get_parameter<double>("maxAllowedJoinTimeGap", config_.maxAllowedJoinTimeGap);
    get_parameter<double>("maxAllowedJoinGap", config_.maxAllowedJoinGap);
    get_parameter<double>("minAllowedJoinGap", config_.minAllowedJoinGap);
    get_parameter<double>("desiredJoinTimeGap", config_.desiredJoinTimeGap);
    get_parameter<double>("desiredJoinGap", config_.desiredJoinGap);
    get_parameter<double>("waitingStateTimeout", config_.waitingStateTimeout);
    get_parameter<double>("cmdSpeedMaxAdjustment", config_.cmdSpeedMaxAdjustment);
    get_parameter<double>("minAllowableHeadaway", config_.minAllowableHeadaway);
    get_parameter<double>("maxAllowableHeadaway", config_.maxAllowableHeadaway);
    get_parameter<double>("headawayStableLowerBond", config_.headawayStableLowerBond);
    get_parameter<double>("headawayStableUpperBond", config_.headawayStableUpperBond);
    get_parameter<double>("minCutinGap", config_.minCutinGap);
    get_parameter<double>("maxCutinGap", config_.maxCutinGap);
    get_parameter<double>("maxCrosstrackError", config_.maxCrosstrackError);
    get_parameter<double>("minPlatooningSpeed", config_.minPlatooningSpeed);
    get_parameter<double>("significantDTDchange", config_.significantDTDchange);
    get_parameter<double>("longitudinalCheckThresold", config_.longitudinalCheckThresold);
    get_parameter<double>("vehicle_length", config_.vehicleLength);
    get_parameter<std::string>("vehicle_id", config_.vehicleID);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&Node::parameter_update_callback, this, std_ph::_1));

    // Setup publishers
    mob_response_pub = create_publisher<carma_v2x_msgs::msg::MobilityResponse>("outgoing_mobility_response", 5);
    mob_request_pub = create_publisher<carma_v2x_msgs::msg::MobilityRequest>("outgoing_mobility_request", 5);
    mob_operation_pub = create_publisher<carma_v2x_msgs::msg::MobilityOperation>("outgoing_mobility_operation", 5);
    platoon_info_pub = create_publisher<carma_planning_msgs::msg::PlatooningInfo>("platoon_info", 1);

    // Build worker

    worker_ = std::make_shared<PlatoonStrategicIHPPlugin>(get_world_model(), config_, [this](auto msg) { this->mob_response_pub->publish(msg); },
                                    [this](auto msg) { this->mob_request_pub->publish(msg); }, [this](auto msg) { this->mob_operation_pub->publish(msg); },
                                    [this](auto msg) { this->platoon_info_pub->publish(msg); },
                                    std::make_shared<carma_ros2_utils::timers::ROSTimerFactory>(shared_from_this()));


    // Setup subscribers
    mob_request_sub = create_subscription<carma_v2x_msgs::msg::MobilityRequest>("incoming_mobility_request", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::mob_req_cb,  worker_.get(), std_ph::_1));

    mob_response_sub = create_subscription<carma_v2x_msgs::msg::MobilityResponse>("incoming_mobility_response", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::mob_resp_cb, worker_.get(), std_ph::_1));

    mob_operation_sub = create_subscription<carma_v2x_msgs::msg::MobilityOperation>("incoming_mobility_operation", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::mob_op_cb, worker_.get(), std_ph::_1));

    current_pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::pose_cb, worker_.get(), std_ph::_1));

    current_twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::twist_cb, worker_.get(), std_ph::_1));

    cmd_sub = create_subscription<geometry_msgs::msg::TwistStamped>("twist_raw", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::cmd_cb, worker_.get(), std_ph::_1));

    georeference_sub = create_subscription<std_msgs::msg::String>("georeference", 10,
                                                              std::bind(&PlatoonStrategicIHPPlugin::georeference_cb, worker_.get(), std_ph::_1));

    loop_timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(100), // 10 Hz frequency
        std::bind(&PlatoonStrategicIHPPlugin::onSpin, worker_.get()));

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn Node::on_cleanup_plugin()
  {
    // Ensure subscribers are disconnected incase cleanup is called, we don't want to keep driving the worker
    mob_response_sub.reset();
    mob_operation_sub.reset();
    current_pose_sub.reset();
    current_twist_sub.reset();
    cmd_sub.reset();
    georeference_sub.reset();
    worker_.reset();

    return CallbackReturn::SUCCESS;
  }


  void Node::plan_maneuvers_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
  {
    if (worker_)
      worker_->plan_maneuver_cb(*req, *resp);
  }

  bool Node::get_availability() {
    return true;
  }

  std::string Node::get_version_id() {
    return "v4.0";
  }

} // platoon_strategic_ihp

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(platoon_strategic_ihp::Node)
