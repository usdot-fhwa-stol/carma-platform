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

#ifndef __ARBITRATOR_INCLUDE_ARBITRATOR_HPP__
#define __ARBITRATOR_INCLUDE_ARBITRATOR_HPP__

#include <rclcpp/rclcpp.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "vehicle_state.hpp"
#include "arbitrator_state_machine.hpp"
#include "planning_strategy.hpp"
#include "capabilities_interface.hpp"

namespace arbitrator 
{
    /**
     * Primary work class for the Arbitrator package
     * 
     * Governs the interactions of plugins during the maneuver planning phase of
     * the CARMA planning process. Utilizes a generic planning interface to allow
     * for reconfiguration with other paradigms in the future.
     */
    class Arbitrator
    {
        public:
            /**
             * \brief Constructor for arbitrator class taking in dependencies via dependency injection
             * \param nh A CarmaLifecycleNode node pointer
             * \param sm An ArbitratorStateMachine instance for regulating the states of the Arbitrator
             * \param ci A CapabilitiesInterface for querying plugins
             * \param planning_strategy A planning strategy implementation for generating plans
             * \param min_plan_duration The minimum acceptable length of a plan
             * \param planning_frequency The frequency at which to generate high-level plans when engaged
             * \param wm pointer to an inialized world model.
             */ 
            Arbitrator(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh,
                std::shared_ptr<ArbitratorStateMachine> sm, 
                std::shared_ptr<CapabilitiesInterface> ci, 
                std::shared_ptr<PlanningStrategy> planning_strategy,
                rclcpp::Duration min_plan_duration,
                double planning_period,
                carma_wm::WorldModelConstPtr wm): 
                sm_(sm),
                nh_(nh),
                capabilities_interface_(ci),
                planning_strategy_(planning_strategy),
                initialized_(false),
                min_plan_duration_(min_plan_duration),
                time_between_plans_(planning_period),
                wm_(wm),
                tf2_buffer_(nh_->get_clock()) {};
            
            /**
             * \brief Begin the operation of the arbitrator.
             * 
             * Loops internally via rclcpp::Duration sleeps and spins
             */
            void run();

            /**
             * \brief Callback for the twist subscriber, which will store latest twist locally
             * \param msg Latest twist message
             */
            void twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg);

            /**
             * \brief Callback for the front bumper pose transform
             */
            void bumper_pose_cb();

            /**
             * \brief Initialize transform Lookup from front bumper to map
             */
            void initializeBumperTransformLookup();
            
        protected:
            /**
             * \brief Function to be executed during the initial state of the Arbitrator
             */
            void initial_state();

            /**
             * \brief Function to be called when the Arbitrator begins planning
             */
            void planning_state();

            /**
             * \brief Function to be executed when the Arbitrator has finished planning
             * and is awaiting another planning cycle
             */
            void waiting_state();

            /**
             * \brief Function to be executed when the Arbitrator is not planning but also
             * not awaiting a new plan cycle
             */
            void paused_state();

            /**
             * \brief Function to be executed when the Arbitrator is to clean up and shutdown
             */
            void shutdown_state();

            /**
             * \brief Callback for receiving Guidance state machine updates
             * \param msg The new GuidanceState message
             */
            void guidance_state_cb(carma_planning_msgs::msg::GuidanceState::UniquePtr msg);

        private:
            
            VehicleState vehicle_state_; // The current state of the vehicle for populating planning requests
            
            std::shared_ptr<ArbitratorStateMachine> sm_;
            carma_ros2_utils::PubPtr<carma_planning_msgs::msg::ManeuverPlan> final_plan_pub_;
            carma_ros2_utils::SubPtr<carma_planning_msgs::msg::GuidanceState> guidance_state_sub_;
            std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh_;
            rclcpp::Duration min_plan_duration_;
            rclcpp::Duration time_between_plans_;
            rclcpp::Time next_planning_process_start_;
            std::shared_ptr<CapabilitiesInterface> capabilities_interface_;
            std::shared_ptr<PlanningStrategy> planning_strategy_;
            bool initialized_;
            carma_wm::WorldModelConstPtr wm_;

            geometry_msgs::msg::TransformStamped tf_;
            // TF listenser
            tf2_ros::Buffer tf2_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
            // transform from front bumper to map
            tf2::Stamped<tf2::Transform> bumper_transform_;
            bool planning_in_progress_ = false;
    };
};

#endif
