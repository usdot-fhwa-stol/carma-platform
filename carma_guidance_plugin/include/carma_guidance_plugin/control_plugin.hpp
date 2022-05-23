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
#include <functional>
#include <carma_planning_msgs/msg/plugin.hpp>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace carma_guidance_plugin
{

  /**
   * \brief ControlPlugin provides default functionality for all carma guidance plugins.
   *        This includes basic state machine management (largely delegated to lifecycle behavior), required interfaces, and plugin discovery
   * 
   */
  class ControlPlugin : public PluginBaseNode
  {

  private:
    // Subscriptions
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> current_pose_sub_;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::TwistStamped> current_velocity_sub_;
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> trajectory_plan_sub_;

    // Timers
    rclcpp::TimerBase::SharedPtr command_timer_;


  protected:

    //! The most recent pose message received by this node
    boost::optional<geometry_msgs::msg::PoseStamped> current_pose_;

    //! The most recent velocity message received by this node
    //  NOTE: Only the twist.linear.x and header are guaranteed to be populated
    boost::optional<geometry_msgs::msg::TwistStamped> current_twist_;

    // TODO comments, note user can determine if trajectory has changed based on trajectory_id
    boost::optional<carma_planning_msgs::msg::TrajectoryPlan> current_trajectory_;


  public:
    /**
     * \brief ControlPlugin constructor 
     */
    explicit ControlPlugin(const rclcpp::NodeOptions &);

    // TODO comments
    void current_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg);
    void current_twist_callback(geometry_msgs::msg::TwistStamped::UniquePtr msg);
    void current_trajectory_callback(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);
    
    
    virtual void generate_command() = 0;

    ////
    // Overrides
    ////

    // Non-Final
    std::string get_capability() override;

    // Final
    uint8_t get_type() override final;

    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_deactivate(const rclcpp_lifecycle::State &) override final;
    carma_ros2_utils::CallbackReturn handle_on_shutdown(const rclcpp_lifecycle::State &) override final; 
    carma_ros2_utils::CallbackReturn handle_on_error(const rclcpp_lifecycle::State &) override final;
  };

} // carma_guidance_plugin
