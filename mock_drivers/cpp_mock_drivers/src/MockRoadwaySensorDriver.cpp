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

#include "cpp_mock_drivers/MockRoadwaySensorDriver.h"

namespace mock_drivers{

    void MockRoadwaySensorDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        derived_object_msgs::ObjectWithCovariance detected_objects = msg->detected_objects;
        derived_object_msgs::LaneModels lane_models = msg->lane_models;

        ros::Time curr_time = ros::Time::now();
      
        detected_objects.header.stamp = curr_time;
        lane_models.header.stamp = curr_time;

        mock_driver_node_.publishData<derived_object_msgs::ObjectWithCovariance>("detected_objects", detected_objects);
        mock_driver_node_.publishData<derived_object_msgs::LaneModels>("lane_models", lane_models);  
    }


    MockRoadwaySensorDriver::MockRoadwaySensorDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        object_with_covariance_ptr_ = boost::make_shared<ROSComms<derived_object_msgs::ObjectWithCovariance>>(ROSComms<derived_object_msgs::ObjectWithCovariance>(CommTypes::pub, false, 10, "detected_objects"));
        lane_models_ptr_ = boost::make_shared<ROSComms<derived_object_msgs::LaneModels>>(ROSComms<derived_object_msgs::LaneModels>(CommTypes::pub, false, 10, "lane_models"));
    }

    int MockRoadwaySensorDriver::run(){

        mock_driver_node_.init();

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<derived_object_msgs::ObjectWithCovariance>>>(object_with_covariance_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<derived_object_msgs::LaneModels>>>(lane_models_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}