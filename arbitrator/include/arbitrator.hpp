/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include "arbitrator_state_machine.hpp"
#include "planning_strategy.hpp"
#include "capabilities_interface.hpp"
#include <cav_msgs/GuidanceState.h>
#include "vehicle_state.hpp"
#include <carma_wm/WorldModel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

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
             * \param nh A CARMANodeHandle instance with a globally referenced ("/") path
             * \param pnh A CARMANodeHandle instance with a privately referenced ("~") path
             * \param sm An ArbitratorStateMachine instance for regulating the states of the Arbitrator
             * \param ci A CapabilitiesInterface for querying plugins
             * \param planning_strategy A planning strategy implementation for generating plans
             * \param min_plan_duration The minimum acceptable length of a plan
             * \param planning_frequency The frequency at which to generate high-level plans when engaged
             * \param wm pointer to an inialized world model.
             */ 
            Arbitrator(ros::CARMANodeHandle *nh, 
                ros::CARMANodeHandle *pnh, 
                ArbitratorStateMachine *sm, 
                CapabilitiesInterface *ci, 
                PlanningStrategy &planning_strategy,
                ros::Duration min_plan_duration,
                ros::Rate planning_frequency,
                carma_wm::WorldModelConstPtr wm):
                sm_(sm),
                nh_(nh),
                pnh_(pnh),
                capabilities_interface_(ci),
                planning_strategy_(planning_strategy),
                initialized_(false),
                min_plan_duration_(min_plan_duration),
                time_between_plans_(planning_frequency.expectedCycleTime()),
                wm_(wm) {};
            
            /**
             * \brief Begin the operation of the arbitrator.
             * 
             * Loops internally via ros::Duration sleeps and spins
             */
            void run();

            /**
             * \brief Callback for the twist subscriber, which will store latest twist locally
             * \param msg Latest twist message
             */
            void twist_cb(const geometry_msgs::TwistStampedConstPtr& msg);

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
            void guidance_state_cb(const cav_msgs::GuidanceState::ConstPtr& msg);

        private:
            
            VehicleState vehicle_state_; // The current state of the vehicle for populating planning requests

            ArbitratorStateMachine *sm_;
            ros::Publisher final_plan_pub_;
            ros::Subscriber guidance_state_sub_;
            ros::CARMANodeHandle *nh_;
            ros::CARMANodeHandle *pnh_;
            ros::Duration min_plan_duration_;
            ros::Duration time_between_plans_;
            ros::Time next_planning_process_start_;
            CapabilitiesInterface *capabilities_interface_;
            PlanningStrategy &planning_strategy_;
            bool initialized_;
            carma_wm::WorldModelConstPtr wm_;

            geometry_msgs::TransformStamped tf_;
            // TF listenser
            tf2_ros::Buffer tf2_buffer_;
            std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
            // transform from front bumper to map
            tf2::Stamped<tf2::Transform> bumper_transform_;

    };
};

#endif
