/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "stop_and_wait_plugin_node.hpp"

namespace stop_and_wait_plugin
{
  namespace std_ph = std::placeholders;

  StopandWaitNode::StopandWaitNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config = StopandWaitConfig();
 
    // Declare parameters
    config.config.minimal_trajectory_duration = declare_parameter<std::string>("minimal_trajectory_duration", config.minimal_trajectory_duration);
    config.stop_timestep = declare_parameter<std::string>("stop_timestep", config.stop_timestep);
    config.trajectory_step_size = declare_parameter<std::string>("trajectory_step_size", config.trajectory_step_size);
    config.accel_limit_multiplier = declare_parameter<std::string>("accel_limit_multiplier", config.accel_limit_multiplier);
    config.accel_limit = declare_parameter<std::string>("/vehicle_acceleration_limit", config.accel_limit);
    config.crawl_speed = declare_parameter<std::string>("crawl_speed", config.crawl_speed);
    config.cernterline_sampling_spacing = declare_parameter<std::string>("cernterline_sampling_spacing", config.cernterline_sampling_spacing);
    config.default_stopping_buffer = declare_parameter<std::string>("default_stopping_buffer", config.default_stopping_buffer);
  }

  carma_ros2_utils::CallbackReturn StopandWaitNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {

    // Setup subscribers
    pose_sub = create_subscription<geometry_msgs::msg::Pose>("pose_to_tf", 50,
                                                              std::bind(&pose_to_tf::PoseToTF2::poseCallback, pose_to_tf_worker_.get(), std_ph::_1));
  

   
    plugin_discovery_pub = create_publisher<carma_planning_msgs::msg::Plugin>("plugin_discovery", 1);



    StopandWait plugin(wm, config, [&plugin_discovery_pub](auto msg) { plugin_discovery_pub.publish(msg); });

    ros::ServiceServer trajectory_srv_ =
        nh.advertiseService("plan_trajectory", &StopandWait::plan_trajectory_cb, &plugin);


    trajectory_srv = create_service<carma_planning_msgs::srv::PlanManeuvers>("plugins/" + planning_strategic_plugin_ + "/plan_maneuvers",
                                                            std::bind(&RouteFollowingPlugin::plan_maneuvers_callback, this, std_ph::_1, std_ph::_2, std_ph::_3));
    

    ros::Timer discovery_pub_timer =
        pnh.createTimer(ros::Duration(ros::Rate(10.0)), [&plugin](const auto&) { plugin.spinCallback(); });                                                          
  
    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

} // stop_and_wait_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(stop_and_wait_plugin::StopandWaitNode)
