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

#include "cpp_mock_drivers/MockRadarDriver.h"

namespace mock_drivers{

    void MockRadarDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        radar_msgs::RadarStatus status_msg = msg->status;
        radar_msgs::RadarTrackArray tracks_raw_msg = msg->tracks_raw;
        ros::Time curr_time = ros::Time::now();
        
        status_msg.header.stamp = curr_time;
        tracks_raw_msg.header.stamp = curr_time;

        mock_driver_node_.publishData<radar_msgs::RadarStatus>("status", status_msg);
        mock_driver_node_.publishData<radar_msgs::RadarTrackArray>("tracks_raw", tracks_raw_msg);
    }

    MockRadarDriver::MockRadarDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        status_pub_ptr_ = boost::make_shared<ROSComms<radar_msgs::RadarStatus>>(ROSComms<radar_msgs::RadarStatus>(CommTypes::pub, false, 10, "radar/status"));
        tracks_raw_pub_ptr_ = boost::make_shared<ROSComms<radar_msgs::RadarTrackArray>>(ROSComms<radar_msgs::RadarTrackArray>(CommTypes::pub, false, 10, "radar/tracks_raw"));
    }

    int MockRadarDriver::run(){

        mock_driver_node_.init();
        
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<radar_msgs::RadarStatus>>>(status_pub_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<radar_msgs::RadarTrackArray>>>(tracks_raw_pub_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}