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

#include "arbitrator.hpp"
#include <carma_planning_msgs/msg/maneuver_plan.hpp>
#include <carma_planning_msgs/srv/plan_maneuvers.hpp>
#include "arbitrator_utils.hpp"
#include <rclcpp/rclcpp.hpp>
#include <exception>
#include <cstdlib>

namespace arbitrator
{
    void Arbitrator::run()
    {
        if (!planning_in_progress_ && nh_->get_current_state().id() != lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
        {
            planning_in_progress_ = true;
            
            switch (sm_->get_state()) 
            {
                case INITIAL:
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator spinning in INITIAL state.");
                    initial_state();
                    break;
                case PLANNING:
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator spinning in PLANNING state.");
                    planning_state();
                    break;
                case WAITING:
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator spinning in WAITING state.");
                    waiting_state();
                    break;
                case PAUSED:
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator spinning in PAUSED state.");
                    paused_state();
                    break;
                case SHUTDOWN:
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator shutting down after being commanded to shutdown!");
                    rclcpp::shutdown();
                    exit(0);
                    break;
                default:
                    throw std::invalid_argument("State machine attempting to process an illegal state value");
            }
        }
        else
        {
            return;    
        }
        planning_in_progress_ = false;
    }
    
    void Arbitrator::guidance_state_cb(carma_planning_msgs::msg::GuidanceState::UniquePtr msg) 
    {
        switch (msg->state)
        {
            case carma_planning_msgs::msg::GuidanceState::STARTUP:
                // NO-OP
                break;
            case carma_planning_msgs::msg::GuidanceState::DRIVERS_READY:
                if(sm_->get_state() == ArbitratorState::PLANNING || sm_->get_state() == ArbitratorState::WAITING)
                {
                    RCLCPP_INFO_STREAM(nh_->get_logger(), "Received notice that guidance has been restarted, pausing arbitrator.");
                    sm_->submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
                }
                break;
            case carma_planning_msgs::msg::GuidanceState::ACTIVE:
                // NO-OP
                break;
            case carma_planning_msgs::msg::GuidanceState::ENGAGED:
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Received notice that guidance has been engaged!");
                if (sm_->get_state() == ArbitratorState::INITIAL) {
                    sm_->submit_event(ArbitratorEvent::SYSTEM_STARTUP_COMPLETE);
                } else if (sm_->get_state() == ArbitratorState::PAUSED) {
                    sm_->submit_event(ArbitratorEvent::ARBITRATOR_RESUMED);
                }
                break;
            case carma_planning_msgs::msg::GuidanceState::INACTIVE:
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Received notice that guidance has been disengaged, pausing arbitrator.");
                sm_->submit_event(ArbitratorEvent::ARBITRATOR_PAUSED);
                break;
            case carma_planning_msgs::msg::GuidanceState::SHUTDOWN:
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Received notice that guidance has been shutdown, shutting down arbitrator.");
                sm_->submit_event(ArbitratorEvent::SYSTEM_SHUTDOWN_INITIATED);
                break;
            default:
                break;
        }
    }

    void Arbitrator::initial_state()
    {
        if(!initialized_)
        {   
            RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator initializing on first initial state spin...");
            final_plan_pub_ = nh_->create_publisher<carma_planning_msgs::msg::ManeuverPlan>("final_maneuver_plan", 5);
            guidance_state_sub_ = nh_->create_subscription<carma_planning_msgs::msg::GuidanceState>("guidance_state", 5, std::bind(&Arbitrator::guidance_state_cb, this, std::placeholders::_1));
            final_plan_pub_->on_activate();

            initialized_ = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void Arbitrator::planning_state()
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator beginning planning process!");
        rclcpp::Time planning_process_start = nh_->get_clock()->now();
        
        carma_planning_msgs::msg::ManeuverPlan plan = planning_strategy_->generate_plan(vehicle_state_);

        if (!plan.maneuvers.empty()) 
        {
            rclcpp::Time plan_end_time = arbitrator_utils::get_plan_end_time(plan);
            rclcpp::Time plan_start_time = arbitrator_utils::get_plan_start_time(plan);
            rclcpp::Duration plan_duration = plan_end_time - plan_start_time;

            if (plan_duration < min_plan_duration_) 
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Arbitrator is unable to generate a plan with minimum plan duration requirement!");
            } 
            else 
            {
                RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator is publishing plan " << std::string(plan.maneuver_plan_id) << " of duration " << plan_duration.seconds() << " as current maneuver plan");
            }
            final_plan_pub_->publish(plan);
        }
        else
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Arbitrator was unable to generate a plan!");
        }

        next_planning_process_start_ = planning_process_start + time_between_plans_;

        sm_->submit_event(ArbitratorEvent::PLANNING_COMPLETE);
    }

    void Arbitrator::waiting_state()
    {
        // Sleep in 100ms increments until our next planning cycle
        // This ensures we spin() at least a few times
        while (nh_->get_clock()->now() < next_planning_process_start_)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator transitioning from WAITING to PLANNING state.");
        sm_->submit_event(ArbitratorEvent::PLANNING_TIMER_TRIGGER);
    }

    void Arbitrator::paused_state()
    {
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    void Arbitrator::shutdown_state()
    {
        RCLCPP_INFO_STREAM(nh_->get_logger(), "Arbitrator shutting down...");
        rclcpp::shutdown(); // Will stop upper level spin and shutdown node
    }

    void Arbitrator::bumper_pose_cb()
    {
        try
        {
            tf_ = tf2_buffer_.lookupTransform("map", "vehicle_front", rclcpp::Time(0), rclcpp::Duration(1.0, 0)); //save to local copy of transform 1 sec timeout
            tf2::fromMsg(tf_, bumper_transform_);
            vehicle_state_.stamp = tf_.header.stamp;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), ex.what());
        }

        vehicle_state_.x = bumper_transform_.getOrigin().getX();
        vehicle_state_.y = bumper_transform_.getOrigin().getY();

        // If the route is available then set the downtrack and lane id
        if (wm_->getRoute()) {

            vehicle_state_.downtrack = wm_->routeTrackPos( { vehicle_state_.x, vehicle_state_.y } ).downtrack;

            auto lanelets = wm_->getLaneletsBetween(vehicle_state_.downtrack, vehicle_state_.downtrack, true);

            if (lanelets.empty()) {

                RCLCPP_WARN_STREAM(nh_->get_logger(), "Vehicle is not in a lanelet.");
                vehicle_state_.lane_id = lanelet::InvalId;

            } else {

                vehicle_state_.lane_id = lanelets[0].id();
            }
        }

    }

    void Arbitrator::twist_cb(geometry_msgs::msg::TwistStamped::UniquePtr msg) 
    {
        vehicle_state_.velocity = msg->twist.linear.x;
    }

    void Arbitrator::initializeBumperTransformLookup() 
    {
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
        tf2_buffer_.setUsingDedicatedThread(true);
    }
};
