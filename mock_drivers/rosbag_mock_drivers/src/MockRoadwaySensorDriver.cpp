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

#include "rosbag_mock_drivers/MockRoadwaySensorDriver.h"

namespace mock_drivers{

    bool MockRoadwaySensorDriver::driverDiscovery(){
        cav_msgs::DriverStatus discovery_msg;
        
        discovery_msg.name = "MockRoadwaySensorDriver";
        discovery_msg.status = 1;

        discovery_msg.can = false;
        discovery_msg.radar = false;
        discovery_msg.gnss = false;
        discovery_msg.lidar = false;
        discovery_msg.roadway_sensor = true;
        discovery_msg.comms = false;
        discovery_msg.controller = false;
        discovery_msg.camera = false;
        discovery_msg.imu = false;
        discovery_msg.trailer_angle_sensor = false;
        discovery_msg.lightbar = false;

        mock_driver_node_.publishDataNoHeader<cav_msgs::DriverStatus>("driver_discovery", discovery_msg);

        return true;
    }

    void MockRoadwaySensorDriver::parserCB(const cav_simulation_msgs::BagData::ConstPtr& msg){
        // generate messages from bag data

        ros::Time curr_time = ros::Time::now();

        if(msg->detected_objects_flag){
            derived_object_msgs::ObjectWithCovariance detected_objects = msg->detected_objects;
            detected_objects.header.stamp = curr_time;
            mock_driver_node_.publishData<derived_object_msgs::ObjectWithCovariance>("roadway_sensor/detected_objects", detected_objects);
        }
        
        if(msg->lane_models_flag){
            derived_object_msgs::LaneModels lane_models = msg->lane_models;
            lane_models.header.stamp = curr_time;
            mock_driver_node_.publishData<derived_object_msgs::LaneModels>("roadway_sensor/lane_models", lane_models);  
        }
    }


    MockRoadwaySensorDriver::MockRoadwaySensorDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        object_with_covariance_ptr_ = boost::make_shared<ROSComms<derived_object_msgs::ObjectWithCovariance>>(ROSComms<derived_object_msgs::ObjectWithCovariance>(CommTypes::pub, false, 10, "detected_objects"));
        lane_models_ptr_ = boost::make_shared<ROSComms<derived_object_msgs::LaneModels>>(ROSComms<derived_object_msgs::LaneModels>(CommTypes::pub, false, 10, "lane_models"));
    }

    int MockRoadwaySensorDriver::run(){

        mock_driver_node_.init();

        // bag parser subscriber
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_simulation_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        // driver publishers
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<derived_object_msgs::ObjectWithCovariance>>>(object_with_covariance_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<derived_object_msgs::LaneModels>>>(lane_models_ptr_);

        // driver discovery publisher
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>>>(driver_discovery_pub_ptr_);
        mock_driver_node_.setSpinCallback(std::bind(&MockRoadwaySensorDriver::driverDiscovery, this));

        mock_driver_node_.spin(100);

        return 0;
    }

}