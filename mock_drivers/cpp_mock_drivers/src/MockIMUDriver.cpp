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

#include "cpp_mock_drivers/MockIMUDriver.h"

namespace mock_drivers{

    bool MockIMUDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockIMUDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = false;
        discovery_msg.imu = true;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("/hardware_interface/driver_discovery", discovery_msg);

        return true;
    }

    void MockIMUDriver::parserCB(const carma_simulation_msgs::BagData::ConstPtr& msg){
        sensor_msgs::Imu raw_data = msg->raw_data;

        ros::Time curr_time = ros::Time::now();
      
        raw_data.header.stamp = curr_time;

        mock_driver_node_.publishData<sensor_msgs::Imu>("/hardware_interface/imu/raw_data", raw_data);
    }
    
    MockIMUDriver::MockIMUDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        imu_pub_ptr_ = boost::make_shared<ROSComms<sensor_msgs::Imu>>(ROSComms<sensor_msgs::Imu>(CommTypes::pub, false, 10, "imu/raw_data"));
        
    }

    int MockIMUDriver::run(){

        mock_driver_node_.init();

        // mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<sensor_msgs::Imu>>>(imu_pub_ptr_);


        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockIMUDriver::driverDiscovery, this));

        mock_driver_node_.spin(1);

        return 0;
    }

}