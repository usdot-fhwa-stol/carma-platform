/*
 * Copyright (C) 2023 LEIDOS.
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
#include <carma_planning_msgs/msg/guidance_state.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

#include <carma_msgs/srv/request_indicator_control.hpp>
#include <carma_msgs/srv/release_indicator_control.hpp>
#include <carma_driver_msgs/srv/set_light_bar_indicator.hpp>

namespace lightbar_manager
{

// Common LightBarManager enumerations and their meanings
enum LightBarIndicator  {GREEN_SOLID, GREEN_FLASH, YELLOW_SIDES, YELLOW_DIM, YELLOW_FLASH, YELLOW_ARROW_LEFT, YELLOW_ARROW_RIGHT, YELLOW_ARROW_OUT};
enum LightBarEvent  {GUIDANCE_ENGAGED, GUIDANCE_DISENGAGED, GUIDANCE_DISCONNECTED, GUIDANCE_CONNECTED};
enum LightBarState {DISENGAGED, ENGAGED, ACTIVE};
enum IndicatorStatus {OFF, ON};
enum IndicatorControlEvent  {CONTROL_LOST, CONTROL_GAINED};

// Cooperative Driving Automation message types: A, B, C, D. 
// These message types can be used to set the corresponding indicator specified in ROS param.
enum LightBarCDAType {TYPE_A, TYPE_B, TYPE_C, TYPE_D};

// Custom exceptions for lightbar manager
class LIGHTBAR_MANAGER_ERROR : public std::exception 
{
	std::string err_;
public:
	LIGHTBAR_MANAGER_ERROR(const std::string& err = "LIGHTBAR_MANAGER_ERROR"):err_(err){};
	char const* what() const throw() {return err_.c_str();};
};

struct INDICATOR_NOT_MAPPED : public LIGHTBAR_MANAGER_ERROR { using LIGHTBAR_MANAGER_ERROR::LIGHTBAR_MANAGER_ERROR;};
struct INVALID_LIGHTBAR_MANAGER_PRIORITY : public LIGHTBAR_MANAGER_ERROR { using LIGHTBAR_MANAGER_ERROR::LIGHTBAR_MANAGER_ERROR;};

class LightBarManagerStateMachine
{
    public:

    /*!
    * \brief Default constructor for LightBarManagerStateMachine
    */
    LightBarManagerStateMachine();

    /*!
    * \brief Transition to the next state of the LightBarStateMachine
    */
    void next(const LightBarEvent& event);

    /*!
    * \brief Get current state machine status.
    */
    LightBarState getCurrentState();
    
    /*!
    * \brief This function triggers the transitioning to the next state in LightBarStateMachine
    * based on the guidance state change
    */
    void handleStateChange(const carma_planning_msgs::msg::GuidanceState& msg);

    private:

    // Helper functions for interpreting guidance state change into lightbar manager event
    void onDisengage();
    void onActive();
    void onEngage();

    // Guidance state local copy for checking any change in state
    uint8_t guidance_state_ = carma_planning_msgs::msg::GuidanceState::SHUTDOWN;

    // ROS Service 
    carma_ros2_utils::ClientPtr<carma_msgs::srv::ReleaseIndicatorControl> request_control_client_;
    carma_ros2_utils::ClientPtr<carma_msgs::srv::RequestIndicatorControl> release_control_client_;
    carma_ros2_utils::ClientPtr<carma_driver_msgs::srv::SetLightBarIndicator> set_indicator_client_;
    
    // a local variable keeps the current state machine state
    LightBarState current_state_ = DISENGAGED;
    
};

} //namespace lightbar_manager
