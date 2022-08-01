/*
 * Copyright (C) 2018-2022 LEIDOS.
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

#include "guidance/guidance_state_machine.hpp"

namespace guidance
{
    GuidanceStateMachine::GuidanceStateMachine(rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
        : logger_(logger) {}

    void GuidanceStateMachine::onGuidanceSignal(Signal signal)
    {
        // Set to OFF state of SHUTDOWN signal received
        if(signal == Signal::SHUTDOWN) {
            current_guidance_state_ = State::OFF;
            return;
        }
        switch(current_guidance_state_)
        {
            case State::STARTUP:
                if(signal == Signal::INITIALIZED) {
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            case State::DRIVERS_READY:
                if(signal == Signal::ACTIVATED)
                {
                    current_guidance_state_ = State::ACTIVE;
                }
                break;
            case State::ACTIVE:
                if(signal == Signal::ENGAGE)
                {
                    current_guidance_state_ = State::ENGAGED;
                } else if(signal == Signal::DISENGAGED)
                {
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            case State::ENGAGED:
                if(signal == Signal::DISENGAGED)
                {
                    current_guidance_state_ = State::DRIVERS_READY;
                } else if(signal == Signal::OVERRIDE)
                {
                    current_guidance_state_ = State::INACTIVE;
                } else if(signal == Signal::PARK)
                {
                    current_guidance_state_ = State::ENTER_PARK;
                }
                break;
            case State::INACTIVE:
                if(signal == Signal::ACTIVATED)
                {
                    current_guidance_state_ = State::ACTIVE;
                }else if(signal == Signal::INITIALIZED){
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            case State::ENTER_PARK:
                if(signal == Signal::OVERRIDE)
                {
                    current_guidance_state_ = State::INACTIVE;
                } 
                else if(signal == Signal::DISENGAGED)
                {
                    current_guidance_state_ = State::DRIVERS_READY;
                }
                break;
            default:
                break;
        }
    }

    void GuidanceStateMachine::onVehicleStatus(autoware_msgs::msg::VehicleStatus::UniquePtr msg)
    {
        current_velocity_ = msg->speed * 0.277777; // Convert kilometers per hour to meters per second. Rounded down so that it comes under epsilon for parking check
        if (current_guidance_state_ == State::ENTER_PARK)
        {
            // '3' indicates vehicle gearshift is currently set to PARK
            if(msg->current_gear.gear== autoware_msgs::msg::Gear::PARK)
            {
                onGuidanceSignal(Signal::OVERRIDE); // Required for ENTER_PARK -> INACTIVE

                onGuidanceSignal(Signal::INITIALIZED); // Required for INACTIVE -> DRIVERS_READY
            }
        }
    }

    void GuidanceStateMachine::onGuidanceInitialized()
    {
        onGuidanceSignal(Signal::INITIALIZED);
    }

    void GuidanceStateMachine::onGuidanceShutdown()
    {
        onGuidanceSignal(Signal::SHUTDOWN);
    }

    void GuidanceStateMachine::onSetGuidanceActive(bool msg)
    {
        if(msg)
        {
            onGuidanceSignal(Signal::ACTIVATED);
        } else
        {
            onGuidanceSignal(Signal::DISENGAGED);
        }
        
    }

    void GuidanceStateMachine::onRoboticStatus(carma_driver_msgs::msg::RobotEnabled::UniquePtr msg)
    {
        // robotic status changes from false to true
        if(!robotic_active_status_ && msg->robot_active)
        {
            robotic_active_status_ = true;
            onGuidanceSignal(Signal::ENGAGE);
        }
        // robotic status changes from true to false
        else if(robotic_active_status_ && !msg->robot_active)
        {
            robotic_active_status_ = false;
            onGuidanceSignal(Signal::OVERRIDE);
        }
    }

    void GuidanceStateMachine::onRouteEvent(carma_planning_msgs::msg::RouteEvent::UniquePtr msg)
    {
        if(msg->event == carma_planning_msgs::msg::RouteEvent::ROUTE_DEPARTED ||
           msg->event == carma_planning_msgs::msg::RouteEvent::ROUTE_ABORTED) {
               onGuidanceSignal(Signal::DISENGAGED);
           }
        else if(msg->event == carma_planning_msgs::msg::RouteEvent::ROUTE_COMPLETED){
            if (fabs(current_velocity_) < 0.001) { // Check we have successfully stopped
                onGuidanceSignal(Signal::PARK); // ENGAGED -> ENTER_PARK this state restricts transitioning out of ENTER_PARK until vehicle is shifted to PARK
            } else { // Vehicle was not able to stop transition to inactive
                RCLCPP_WARN_STREAM(logger_->get_logger(), "Vehicle failed to park on route completion because current velocity was " << current_velocity_);
                onGuidanceSignal(Signal::OVERRIDE);
            }
        }
    }

    uint8_t GuidanceStateMachine::getCurrentState()
    {
        return static_cast<uint8_t>(current_guidance_state_);
    }

    bool GuidanceStateMachine::shouldCallSetEnableRobotic()
    {
        // call SetEnableRobotic service once when we enter ACTIVE state
        if(current_guidance_state_ == GuidanceStateMachine::ACTIVE)
        {
            if(!called_robotic_engage_in_active_)
            {
                called_robotic_engage_in_active_ = true;
                return true;
            }
        } else {
            // Reset when we leave ACTIVE state 
            called_robotic_engage_in_active_ = false;
        }
        return false;
    }

}