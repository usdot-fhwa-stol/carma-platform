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

#pragma once

#include "cpp_mock_drivers/MockDriver.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <j2735_msgs/TransmissionState.h>
#include <cav_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <automotive_platform_msgs/VelocityAccel.h>

namespace mock_drivers{

    class MockCANDriver : public MockDriver {

        private:

            boost::shared_ptr<ROSComms<std_msgs::Bool>> acc_engaged_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> acceleration_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> antilock_brakes_active_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> brake_lights_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> brake_position_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> engine_speed_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> fuel_flow_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> odometer_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> parking_brake_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> speed_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> stability_ctrl_active_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> stability_ctrl_enabled_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> steering_wheel_angle_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Float64>> throttle_position_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> traction_ctrl_active_ptr_;
            boost::shared_ptr<ROSComms<std_msgs::Bool>> traction_ctrl_enabled_ptr_;
            boost::shared_ptr<ROSComms<j2735_msgs::TransmissionState>> transmission_state_ptr_;
            boost::shared_ptr<ROSComms<cav_msgs::TurnSignal>> turn_signal_state_ptr_;
            boost::shared_ptr<ROSComms<geometry_msgs::TwistStamped>> vehicle_twist_ptr_;
            boost::shared_ptr<ROSComms<autoware_msgs::VehicleStatus>> vehicle_status_ptr_;
            boost::shared_ptr<ROSComms<automotive_platform_msgs::VelocityAccel>> velocity_accel_ptr_;

        public:

            MockCANDriver(bool dummy = false);
            int run();
            void parserCB(const cav_msgs::BagData::ConstPtr& msg);

    };

}