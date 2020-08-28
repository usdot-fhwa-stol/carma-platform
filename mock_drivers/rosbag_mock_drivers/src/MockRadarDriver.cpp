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

#include "rosbag_mock_drivers/MockRadarDriver.h"

namespace mock_drivers{

    bool MockRadarDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockRadarDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = true;
        discovery_msg.gnss = false;
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

    void MockRadarDriver::parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg){
        ros::Time curr_time = ros::Time::now();
        
        // generate messages from bag data

        if(msg->status_flag){
            radar_msgs::RadarStatus status_msg = msg->status;
            status_msg.header.stamp = curr_time;
            mock_driver_node_.publishData<radar_msgs::RadarStatus>("radar/status", status_msg);
        }
        
        if(msg->tracks_raw_flag){
            radar_msgs::RadarTrackArray tracks_raw_msg = msg->tracks_raw;
            tracks_raw_msg.header.stamp = curr_time;
            mock_driver_node_.publishData<radar_msgs::RadarTrackArray>("radar/tracks_raw", tracks_raw_msg);
        }
    }

    MockRadarDriver::MockRadarDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

    }

    int MockRadarDriver::run(){

        mock_driver_node_.init();
        
        // driver publisher and subscriber
        addPassthroughPub<radar_msgs::RadarStatus>(bag_prefix_ + radar_status_topic_, radar_status_topic_, false, 10);
        addPassthroughPub<radar_msgs::RadarTrackArray>(bag_prefix_ + radar_tracks_raw_topic_, radar_tracks_raw_topic_, false, 10);

        // driver discovery publisher
        mock_driver_node_.addPub(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockRadarDriver::driverDiscovery, this));

        mock_driver_node_.spin(100);

        return 0;
    }

}