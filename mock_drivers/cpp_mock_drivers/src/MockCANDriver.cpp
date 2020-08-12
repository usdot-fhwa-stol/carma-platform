/*
 * Copyright (C) 2019-2020 LEIDOS.
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

#include "cpp_mock_drivers/MockCANDriver.h"

namespace mock_drivers{

    void MockCANDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        
    }

    MockCANDriver::MockCANDriver(){

        acc_engaged_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "acc_engaged"));
        acceleration_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "acceleration"));
        antilock_brakes_active_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "antilock_brakes_active"));
        brake_lights_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "brake_lights"));
        brake_position_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "brake_position"));
        engine_speed_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "engine_speed"));
        fuel_flow_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "fuel_flow"));
        odometer_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "odometer"));
        parking_brake_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "parking_brake"));
        speed_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "speed"));
        stability_ctrl_active_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "stability_ctrl_active"));
        stability_ctrl_enabled_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "stability_ctrl_enabled"));
        steering_wheel_angle_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "steering_wheel_angle"));
        throttle_position_ptr_ = boost::make_shared<ROSComms<std_msgs::Float64>>(ROSComms<std_msgs::Float64>(CommTypes::pub, false, 10, "throttle_position"));
        traction_ctrl_active_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "traction_ctrl_active"));
        traction_ctrl_enabled_ptr_ = boost::make_shared<ROSComms<std_msgs::Bool>>(ROSComms<std_msgs::Bool>(CommTypes::pub, false, 10, "traction_ctrl_enabled"));
        transmission_state_ptr_ = boost::make_shared<ROSComms<j2735_msgs::TransmissionState>>(ROSComms<j2735_msgs::TransmissionState>(CommTypes::pub, false, 10, "transmission_state"));
        turn_signal_state_ptr_ = boost::make_shared<ROSComms<cav_msgs::TurnSignal>>(ROSComms<cav_msgs::TurnSignal>(CommTypes::pub, false, 10, "turn_signal_state"));
        vehicle_twist_ptr_ = boost::make_shared<ROSComms<geometry_msgs::TwistStamped>>(ROSComms<geometry_msgs::TwistStamped>(CommTypes::pub, false, 10, "vehicle/twist"));
        vehicle_status_ptr_ = boost::make_shared<ROSComms<autoware_msgs::VehicleStatus>>(ROSComms<autoware_msgs::VehicleStatus>(CommTypes::pub, false, 10, "vehicle_status"));
        velocity_accel_ptr_ = boost::make_shared<ROSComms<automotive_platform_msgs::VelocityAccel>>(ROSComms<automotive_platform_msgs::VelocityAccel>(CommTypes::pub, false, 10, "velocity_accel"));


    }

    int MockCANDriver::run(){

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(acc_engaged_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(acceleration_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(antilock_brakes_active_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(brake_lights_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(brake_position_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(engine_speed_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(fuel_flow_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(odometer_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(parking_brake_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(speed_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(stability_ctrl_active_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(stability_ctrl_enabled_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(steering_wheel_angle_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Float64>>>(throttle_position_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(traction_ctrl_active_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<std_msgs::Bool>>>(traction_ctrl_enabled_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<j2735_msgs::TransmissionState>>>(transmission_state_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::TurnSignal>>>(turn_signal_state_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<geometry_msgs::TwistStamped>>>(vehicle_twist_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<autoware_msgs::VehicleStatus>>>(vehicle_status_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<automotive_platform_msgs::VelocityAccel>>>(velocity_accel_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}