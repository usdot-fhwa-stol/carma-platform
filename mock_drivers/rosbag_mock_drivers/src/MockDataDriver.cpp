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

#include "rosbag_mock_drivers/MockDataDriver.h"

namespace mock_drivers{

    bool MockDataDriver::driverDiscovery(){
        return true;
    }

    void MockDataDriver::parserCB(const carma_simulation_msgs::BagData::ConstPtr& msg){
        if(msg->tf_bool.data){
            tf2_msgs::TFMessage tf = msg->tf;
            mock_driver_node_.publishDataNoHeader<tf2_msgs::TFMessage>("/hardware_interface/data/tf", tf);
        }

        if(msg->tf_static_bool.data){
            tf2_msgs::TFMessage tf_static = msg->tf_static;
            mock_driver_node_.publishDataNoHeader<tf2_msgs::TFMessage>("/hardware_interface/data/tf_static", tf_static);
        }
        
    }

    MockDataDriver::MockDataDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        tf_ptr_ = boost::make_shared<ROSComms<tf2_msgs::TFMessage>>(ROSComms<tf2_msgs::TFMessage>(CommTypes::pub, false, 10, "tf"));
        tf_static_ptr_ = boost::make_shared<ROSComms<tf2_msgs::TFMessage>>(ROSComms<tf2_msgs::TFMessage>(CommTypes::pub, false, 10, "tf_static"));

    }

    int MockDataDriver::run(){

        mock_driver_node_.init();

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<tf2_msgs::TFMessage>>>(tf_ptr_);
        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<tf2_msgs::TFMessage>>>(tf_static_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}