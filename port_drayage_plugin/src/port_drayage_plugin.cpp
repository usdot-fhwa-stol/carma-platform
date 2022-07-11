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

<<<<<<< HEAD
} // port_drayage_plugin
=======
        if(wm_ == nullptr) {
            return false;
        }

        auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, current_loc, 1);

        if(current_lanelets.size() == 0) {
            ROS_WARN_STREAM("Cannot find any lanelet in map!");
            return false;
        }

        auto current_lanelet = current_lanelets[0];
        auto stop_rules = current_lanelet.second.regulatoryElementsAs<lanelet::StopRule>();
        double stop_rule_downtrack = 0;

        if(stop_rules.empty()) {
            return false;
        }
        else {
            auto stop_rule_elem = stop_rules.front();
            auto stop_line_vector = stop_rule_elem->stopAndWaitLine();
            auto stop_line = lanelet::traits::to2D(stop_line_vector.front());
            auto point = lanelet::traits::to2D(stop_line.front());
            auto pos = carma_wm::geometry::trackPos(current_lanelet.second, point);
            stop_rule_downtrack = pos.downtrack;
        }

        double current_loc_downtrack = wm_->routeTrackPos(current_loc).downtrack;

        double speed_progress = _cur_speed.linear.x;
        double current_progress = 0;
        
        if(current_loc_downtrack < stop_rule_downtrack)
        {

            ros::Time start_time;

            if(req.prior_plan.maneuvers.size() > 0) {
                switch(req.prior_plan.maneuvers.back().type) {
                    case cav_msgs::Maneuver::LANE_FOLLOWING : 
                        start_time = req.prior_plan.maneuvers.back().lane_following_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().lane_following_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::LANE_CHANGE : 
                        start_time = req.prior_plan.maneuvers.back().lane_change_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().lane_change_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_STRAIGHT : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_straight_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_straight_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_LEFT_TURN : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_left_turn_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_left_turn_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::INTERSECTION_TRANSIT_RIGHT_TURN : 
                        start_time = req.prior_plan.maneuvers.back().intersection_transit_right_turn_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().intersection_transit_right_turn_maneuver.end_dist;
                        break;
                    case cav_msgs::Maneuver::STOP_AND_WAIT : 
                        start_time = req.prior_plan.maneuvers.back().stop_and_wait_maneuver.end_time;
                        current_progress = req.prior_plan.maneuvers.back().stop_and_wait_maneuver.end_dist;
                        break;
                }
            } else {
                start_time = ros::Time::now();
                current_progress = current_loc_downtrack;
            }

            double end_dist = stop_rule_downtrack;

            double estimated_distance_to_stop = estimate_distance_to_stop(speed_progress,declaration);
            double estimated_time_to_stop = estimate_time_to_stop(estimated_distance_to_stop,speed_progress);

            double lane_following_distance = stop_rule_downtrack - estimated_distance_to_stop;
            double stop_and_wait_distance = estimated_distance_to_stop;

            resp.new_plan.maneuvers.push_back(
                compose_lane_following_maneuver_message(current_progress, 
                                       current_progress + lane_following_distance, 
                                       speed_progress, 
                                       speed_progress, 
                                       current_lanelet.second.id(), 
                                       start_time));

            resp.new_plan.maneuvers.push_back(
                compose_stop_and_wait_maneuver_message(current_progress + lane_following_distance, 
                                       end_dist, 
                                       speed_progress, 
                                       0.0, 
                                       current_lanelet.second.id(), 
                                       start_time,
                                       estimated_time_to_stop));
        }

        if(resp.new_plan.maneuvers.size() == 0)
        {
            ROS_WARN_STREAM("Cannot plan maneuver because no route is found");
        }

        return true;
    };

    cav_msgs::Maneuver PortDrayagePlugin::compose_stop_and_wait_maneuver_message(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time time, double time_to_stop)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::STOP_AND_WAIT;
        maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = "stop_and_wait_plugin";
        maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = "PortDrayageWorkerPlugin";
        maneuver_msg.stop_and_wait_maneuver.start_dist = current_dist;
        maneuver_msg.stop_and_wait_maneuver.start_speed = current_speed;
        maneuver_msg.stop_and_wait_maneuver.start_time = time;
        maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
        maneuver_msg.stop_and_wait_maneuver.end_time = time + ros::Duration(std::max(15.0,time_to_stop));
        maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(lane_id);
        maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(lane_id);
        return maneuver_msg;
    }

    cav_msgs::Maneuver PortDrayagePlugin::compose_lane_following_maneuver_message(double current_dist, double end_dist, double current_speed, double target_speed, int lane_id, ros::Time time)
    {
        cav_msgs::Maneuver maneuver_msg;
        maneuver_msg.type = cav_msgs::Maneuver::LANE_FOLLOWING;
        maneuver_msg.lane_following_maneuver.parameters.negotiation_type = cav_msgs::ManeuverParameters::NO_NEGOTIATION;
        maneuver_msg.lane_following_maneuver.parameters.presence_vector = cav_msgs::ManeuverParameters::HAS_TACTICAL_PLUGIN;
        maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = "InlaneCruisingPlugin";
        maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = "route_following_plugin";
        maneuver_msg.lane_following_maneuver.start_dist = current_dist;
        maneuver_msg.lane_following_maneuver.start_speed = current_speed;
        maneuver_msg.lane_following_maneuver.start_time = time;
        maneuver_msg.lane_following_maneuver.end_dist = end_dist;
        maneuver_msg.lane_following_maneuver.end_speed = target_speed;
        // because it is a rough plan, assume vehicle can always reach to the target speed in a lanelet
        maneuver_msg.lane_following_maneuver.end_time = time + ros::Duration((end_dist - current_dist) / (0.5 * (current_speed + target_speed)));
        maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lane_id) };
        return maneuver_msg;
    }
    // @SONAR_START@

    double estimate_distance_to_stop(double v, double a) {
        return (v*v)/2*a;
    }
>>>>>>> develop

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(port_drayage_plugin::PortDrayagePlugin)
