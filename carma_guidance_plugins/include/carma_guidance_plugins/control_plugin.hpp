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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_msgs/msg/control_command_stamped.hpp>

#include "carma_guidance_plugins/plugin_base_node.hpp"

namespace carma_guidance_plugins
{

  /**
   * \brief ControlPlugin base class which can be extended by user provided plugins which wish to implement the Control Plugin ROS API. 
   * 
   * A control plugin is responsible for generating high frequency vehicle speed and steering commands to execute the currently planned trajectory. 
   * This plugin provides default subscribers to track the pose, velocity, and current trajectory in the system. 
   * Extending classes must implement the generate_command() method to use that data and or additional data to plan commands at a 30Hz frequency. 
   * 
   */
  class ControlPlugin : public PluginBaseNode
  {

  private:
    // Subscriptions
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> current_pose_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> current_velocity_sub_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> trajectory_plan_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<autoware_msgs::msg::ControlCommandStamped> vehicle_cmd_pub_;

    // Timers
    rclcpp::TimerBase::SharedPtr command_timer_;

    // These callbacks do direct assignment into their respective member variables
    void current_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
    void current_twist_callback(geometry_msgs::msg::TwistStamped::UniquePtr msg);
    void current_trajectory_callback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);


  protected:

    //! The most recent pose message received by this node
    boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;

    //! The most recent velocity message received by this node
    //  NOTE: Only the twist.linear.x and header are guaranteed to be populated
    boost::optional<geometry_msgs::msg::TwistStamped> current_twist_;

    //! The most recent trajectory received by this plugin
    boost::optional<carma_planning_msgs::msg::TrajectoryPlan> current_trajectory_;


  public:
    /**
     * \brief ControlPlugin constructor 
     */
    explicit ControlPlugin(const rclcpp::NodeOptions &);

    //! Virtual destructor for safe deletion
    virtual ~ControlPlugin() = default;

    /**
     * \brief Extending class provided method which should generate a command message 
     *        which will be published to the required topic by the base class
     * 
     * NOTE:  Implementer can determine if trajectory has changed based on current_trajectory_->trajectory_id
     * 
     * \return The command message to publish
     */ 
    virtual autoware_msgs::msg::ControlCommandStamped generate_command() = 0;

    ////
    // Overrides
    ////

    // Non-Final to allow extending plugins to provide more detailed capabilities
    std::string get_capability() override;

    // Final
    uint8_t get_type() override final;

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override final; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &, const std::string &exception_string) override final;
  };

} // carma_guidance_plugins
