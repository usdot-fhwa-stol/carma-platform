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

    bool MockCANDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockCANDriver";
        discovery_msg.status = 1;

        discovery_msg.can = true;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = false;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("/hardware_interface/driver_discovery", discovery_msg);

        return true;
    }
    
    void MockCANDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        std_msgs::Bool acc_engaged = msg->acc_engaged;
        std_msgs::Float64 acceleration = msg->acceleration;
        std_msgs::Bool antilock_brakes_active = msg->antilock_brakes_active;
        std_msgs::Bool brake_lights = msg->brake_lights;
        std_msgs::Float64 brake_position = msg->brake_position;
        std_msgs::Float64 engine_speed = msg->engine_speed;
        std_msgs::Float64 fuel_flow = msg->fuel_flow;
        std_msgs::Float64 odometer = msg->odometer;
        std_msgs::Bool parking_brake = msg->parking_brake;
        std_msgs::Float64 speed = msg->speed;
        std_msgs::Bool stability_ctrl_active = msg->stability_ctrl_active;
        std_msgs::Bool stability_ctrl_enabled = msg->stability_ctrl_enabled;
        std_msgs::Float64 steering_wheel_angle = msg->steering_wheel_angle;
        std_msgs::Float64 throttle_position = msg->throttle_position;
        std_msgs::Bool traction_ctrl_active = msg->traction_ctrl_active;
        std_msgs::Bool traction_ctrl_enabled = msg->traction_ctrl_enabled;
        j2735_msgs::TransmissionState transmission_state = msg->transmission_state;
        cav_msgs::TurnSignal turn_signal_state = msg->turn_signal_state;
        geometry_msgs::TwistStamped vehicle_twist = msg->vehicle_twist;
        autoware_msgs::VehicleStatus vehicle_status = msg->vehicle_status;
        automotive_platform_msgs::VelocityAccel velocity_accel = msg->velocity_accel;

        ros::Time curr_time = ros::Time::now();
        
        // acc_engaged.header.stamp = curr_time;
        // acceleration.header.stamp = curr_time;
        // antilock_brakes_active.header.stamp = curr_time;
        // brake_lights.header.stamp = curr_time;
        // brake_position.header.stamp = curr_time;
        // engine_speed.header.stamp = curr_time;
        // fuel_flow.header.stamp = curr_time;
        // odometer.header.stamp = curr_time;
        // parking_brake.header.stamp = curr_time;
        // speed.header.stamp = curr_time;
        // stability_ctrl_active.header.stamp = curr_time;
        // stability_ctrl_enabled.header.stamp = curr_time;
        // steering_wheel_angle.header.stamp = curr_time;
        // throttle_position.header.stamp = curr_time;
        // traction_ctrl_active.header.stamp = curr_time;
        // traction_ctrl_enabled.header.stamp = curr_time;
        // transmission_state.header.stamp = curr_time;
        // turn_signal_state.header.stamp = curr_time;
        vehicle_twist.header.stamp = curr_time;
        vehicle_status.header.stamp = curr_time;
        velocity_accel.header.stamp = curr_time;


        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/acc_engaged", acc_engaged);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/acceleration", acceleration);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/antilock_brakes_active", antilock_brakes_active);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/brake_lights", brake_lights);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/brake_position", brake_position);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/engine_speed", engine_speed);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/fuel_flow", fuel_flow);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/odometer", odometer);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/parking_brake", parking_brake);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/speed", speed);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/stability_ctrl_active", stability_ctrl_active);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/stability_ctrl_enabled", stability_ctrl_enabled);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/steering_wheel_angle", steering_wheel_angle);
        mock_driver_node_.publishDataNoHeader<std_msgs::Float64>("/hardware_interface/can/throttle_position", throttle_position);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/traction_ctrl_active", traction_ctrl_active);
        mock_driver_node_.publishDataNoHeader<std_msgs::Bool>("/hardware_interface/can/traction_ctrl_enabled", traction_ctrl_enabled);
        mock_driver_node_.publishDataNoHeader<j2735_msgs::TransmissionState>("/hardware_interface/can/transmission_state", transmission_state);
        mock_driver_node_.publishDataNoHeader<cav_msgs::TurnSignal>("/hardware_interface/can/turn_signal_state", turn_signal_state);
        mock_driver_node_.publishData<geometry_msgs::TwistStamped>("/hardware_interface/can/vehicle/twist", vehicle_twist);
        mock_driver_node_.publishData<autoware_msgs::VehicleStatus>("/hardware_interface/can/vehicle_status", vehicle_status);
        mock_driver_node_.publishData<automotive_platform_msgs::VelocityAccel>("/hardware_interface/can/velocity_accel", velocity_accel);
    }

    MockCANDriver::MockCANDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

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

        mock_driver_node_.init();

        // mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

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

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockCANDriver::driverDiscovery, this));

        mock_driver_node_.spin(1);

        return 0;
    }

}