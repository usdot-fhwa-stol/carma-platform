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
#include "trajectory_executor/trajectory_executor_node.hpp"

namespace trajectory_executor
{
  namespace std_ph = std::placeholders;

  TrajectoryExecutor::TrajectoryExecutor(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.trajectory_publish_rate = declare_parameter<double>("trajectory_publish_rate", config_.trajectory_publish_rate);
    config_.default_control_plugin = declare_parameter<std::string>("default_control_plugin", config_.default_control_plugin);
    config_.default_control_plugin_topic = declare_parameter<std::string>("default_control_plugin_topic", config_.default_control_plugin_topic);
  }

  rcl_interfaces::msg::SetParametersResult TrajectoryExecutor::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({{"trajectory_publish_rate", config_.trajectory_publish_rate}}, parameters);
    auto error_2 = update_params<std::string>({
        {"default_control_plugin", config_.default_control_plugin},
        {"default_control_plugin_topic", config_.default_control_plugin_topic}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    return result;
  }

  std::map<std::string, std::string> TrajectoryExecutor::queryControlPlugins()
  {
      // Hard coded stub for MVP since plugin manager won't be developed yet
      // TODO: Query plugin manager to receive actual list of plugins and their corresponding topics
      RCLCPP_DEBUG_STREAM(get_logger(), "Executing stub behavior for plugin discovery MVP...");
      std::map<std::string, std::string> out;

      out[config_.default_control_plugin] = config_.default_control_plugin_topic;

      //Hardcoding platoon control plugins
      std::string control_plugin2 = "platoon_control";
      std::string control_plugin_topic2 = "/guidance/plugins/platoon_control/plan_trajectory";
      out[control_plugin2] = control_plugin_topic2;

      return out;
  }

  carma_ros2_utils::CallbackReturn TrajectoryExecutor::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "TrajectoryExecutor trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("trajectory_publish_rate", config_.trajectory_publish_rate);
    get_parameter<std::string>("default_control_plugin", config_.default_control_plugin);
    get_parameter<std::string>("default_control_plugin_topic", config_.default_control_plugin_topic);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&TrajectoryExecutor::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    plan_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("trajectory", 5,
                                                              std::bind(&TrajectoryExecutor::onNewTrajectoryPlan, this, std_ph::_1));
    state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("state", 5,
                                                              std::bind(&TrajectoryExecutor::guidanceStateMonitor, this, std_ph::_1));

    cur_traj_ = std::unique_ptr<carma_planning_msgs::msg::TrajectoryPlan>();

    RCLCPP_DEBUG_STREAM(get_logger(), "Setting up publishers for control plugin topics...");

    std::map<std::string, carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan>> control_plugin_topics;
    auto discovered_control_plugins = queryControlPlugins();

    for (auto it = discovered_control_plugins.begin(); it != discovered_control_plugins.end(); it++)
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "Trajectory executor discovered control plugin " << it->first.c_str() << " listening on topic " <<  it->second.c_str());
      carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan> control_plugin_pub = create_publisher<carma_planning_msgs::msg::TrajectoryPlan>(it->second, 1000);
      control_plugin_topics.insert(std::make_pair(it->first, control_plugin_pub));
    }

    traj_publisher_map_ = control_plugin_topics;
    RCLCPP_DEBUG_STREAM(get_logger(), "TrajectoryExecutor component initialized succesfully!");

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }
  carma_ros2_utils::CallbackReturn TrajectoryExecutor::handle_on_activate(const rclcpp_lifecycle::State &)
  {
    // Create timer for publishing outbound trajectories to the control plugins
    int timer_period_ms = (1 / config_.trajectory_publish_rate) * 1000; // Conversion from frequency (Hz) to milliseconds time period

    timer_ = create_timer(
        get_clock(),
        std::chrono::milliseconds(timer_period_ms), 
        std::bind(&TrajectoryExecutor::onTrajEmitTick, this));

    return CallbackReturn::SUCCESS;
  }

  void TrajectoryExecutor::onTrajEmitTick()
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "TrajectoryExecutor tick start!");

    if (cur_traj_ != nullptr) {
      // Determine the relevant control plugin for the current timestep
      std::string control_plugin = cur_traj_->trajectory_points[0].controller_plugin_name;
      // if it instructed to use default control_plugin
      if (control_plugin == "default" || control_plugin =="") {
        control_plugin = config_.default_control_plugin;
      }

      std::map<std::string, carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan>>::iterator it = traj_publisher_map_.find(control_plugin);
      if (it != traj_publisher_map_.end()) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Found match for control plugin " << control_plugin.c_str() << " at point " << timesteps_since_last_traj_ << " in current trajectory!");
        it->second->publish(*cur_traj_);
      } else {
        std::ostringstream description_builder;
              description_builder << "No match found for control plugin " 
              << control_plugin << " at point " 
              << timesteps_since_last_traj_ << " in current trajectory!";

        throw std::invalid_argument(description_builder.str());
      }
      timesteps_since_last_traj_++; 
    } else {
      RCLCPP_DEBUG_STREAM(get_logger(), "Awaiting initial trajectory publication...");
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "TrajectoryExecutor tick completed succesfully!");

  }

  void TrajectoryExecutor::onNewTrajectoryPlan(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "Received new trajectory plan!");
    RCLCPP_DEBUG_STREAM(get_logger(), "New Trajectory plan ID: " << msg->trajectory_id);
    RCLCPP_DEBUG_STREAM(get_logger(), "New plan contains " << msg->trajectory_points.size() << " points");

    cur_traj_ = std::unique_ptr<carma_planning_msgs::msg::TrajectoryPlan>(move(msg));
    timesteps_since_last_traj_ = 0;
    RCLCPP_INFO_STREAM(get_logger(), "Successfully swapped trajectories!");  
  }

  void TrajectoryExecutor::guidanceStateMonitor(carma_planning_msgs::msg::GuidanceState::UniquePtr msg)
  {
    // TODO need to handle control handover once alernative planner system is finished
    if(msg->state != carma_planning_msgs::msg::GuidanceState::ENGAGED)
    {
    	cur_traj_= nullptr;
    }
  }

} // trajectory_executor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_executor::TrajectoryExecutor)
