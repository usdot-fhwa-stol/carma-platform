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
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <gtest/gtest_prod.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "trajectory_executor/trajectory_executor_config.hpp"

namespace trajectory_executor
{

  /**
   * Trajectory Executor package primary worker class
   * 
   * Handles subscribing to inbound plans from Plan Delegator component.
   * control plugin registration querying, and then coordination of execution
   * of that plan amongst multiple control plugins.
   */
  class TrajectoryExecutor : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> plan_sub_; // Inbound trajectory plan subscriber
    carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> state_sub_; // Guidance State subscriber

    std::map<std::string, carma_ros2_utils::PubPtr<carma_planning_msgs::msg::TrajectoryPlan>> traj_publisher_map_; // Outbound trajectory plan publishers

    // Timers
    rclcpp::TimerBase::SharedPtr timer_; // Timer for publishing outbound trajectories to the control plugins

    // Node configuration
    Config config_;

    // Trajectory plan tracking data
    std::unique_ptr<carma_planning_msgs::msg::TrajectoryPlan> cur_traj_; 
    int timesteps_since_last_traj_ {0};

  protected:
    /*!
     * \brief Helper function to query control plugin registration system
     * 
     * \return A map of control plugin name -> control plugin input topics
     *  for all discovered control plugins 
     */
    std::map<std::string, std::string> queryControlPlugins();

    /*!
     * \brief Callback to be invoked when a new trajectory plan is
     * received on our inbound plan topic.
     * 
     * \param msg The new TrajectoryPlan message
     */
    void onNewTrajectoryPlan(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);

    /*!
     * \brief Monitor the guidance state and set the current trajector as null_ptr 
     */
    void guidanceStateMonitor(carma_planning_msgs::msg::GuidanceState::UniquePtr msg);

    /*!
     * \brief Timer callback to be invoked at our output tickrate.
     * Outputs current trajectory plan to the first control plugin in
     * it's point list. If this is our second or later timestep on the
     * same trajectory, consumes the first point in the point list before
     * transmission.
     */
    void onTrajEmitTick();

  public:

    /*!
     * \brief Constructor for TrajectoryExecutor
     */
    explicit TrajectoryExecutor(const rclcpp::NodeOptions &);

    /**
     * \brief Example callback for dynamic parameter updates
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
    carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);
  };

} // trajectory_executor
