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

#include "rosbag_mock_drivers/MockGNSSDriver.h"

namespace mock_drivers{

    bool MockGNSSDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockGNSSDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = false;
        discovery_msg.gnss = true;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = false;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = false;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

        return true;
    }

    void MockGNSSDriver::parserCB(const carma_simulation_msgs::BagData::ConstPtr& msg){
        // generate messages from bag data
        if(msg->gnss_fixed_fused_bool.data){
            gps_common::GPSFix gnss_fixed_fused = msg->gnss_fixed_fused;
            // update time stamps
            gnss_fixed_fused.header.stamp = ros::Time::now();
            // publish the data
            mock_driver_node_.publishData<gps_common::GPSFix>("gnss/gnss_fixed_fused", gnss_fixed_fused);
        }
    }

    MockGNSSDriver::MockGNSSDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        GPS_fix_ptr_ = boost::make_shared<ROSComms<gps_common::GPSFix>>(ROSComms<gps_common::GPSFix>(CommTypes::pub, false, 10, "gnss_fixed_fused"));
    
    }

    int MockGNSSDriver::run(){

        mock_driver_node_.init();

        // bag parser subscriber
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        // driver publisher and subscriber
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<gps_common::GPSFix>>>(GPS_fix_ptr_);

        // driver discovery publisher
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockGNSSDriver::driverDiscovery, this));

        mock_driver_node_.spin(1);

        return 0;
    }

}