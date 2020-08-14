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

#include "cpp_mock_drivers/MockCommsDriver.h"

namespace mock_drivers{

    void MockCommsDriver::parserCB(const cav_msgs::BagData::ConstPtr& msg){
        cav_msgs::ByteArray inbound_binary_msg = msg->inbound_binary_msg;

        ros::Time curr_time = ros::Time::now();
      
        inbound_binary_msg.header.stamp = curr_time;

        mock_driver_node_.publishData<cav_msgs::ByteArray>("inbound_binary_msg", inbound_binary_msg);  
    }
    
    void MockCommsDriver::outboundCallback(const cav_msgs::ByteArray::ConstPtr& msg){

    };

    MockCommsDriver::MockCommsDriver(bool dummy){

        mock_driver_node_ = MockDriverNode(dummy);

        inbound_pub_ptr_ = boost::make_shared<ROSComms<cav_msgs::ByteArray>>(ROSComms<cav_msgs::ByteArray>(CommTypes::pub, false, 10, "inbound_binary_msg"));
        
        std::function<void(const cav_msgs::ByteArray::ConstPtr&)> outbound_ptr = std::bind(&MockCommsDriver::outboundCallback, this, std::placeholders::_1);
        outbound_sub_ptr_ = boost::make_shared<ROSComms<const cav_msgs::ByteArray::ConstPtr&>>(ROSComms<const cav_msgs::ByteArray::ConstPtr&>(outbound_ptr, CommTypes::sub, false, 10, "outbound_binary_msg"));
    }

    int MockCommsDriver::run(){

        mock_driver_node_.init();

        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>>>(bag_parser_sub_ptr_);

        mock_driver_node_.addPub<boost::shared_ptr<ROSComms<cav_msgs::ByteArray>>>(inbound_pub_ptr_);
        mock_driver_node_.addSub<boost::shared_ptr<ROSComms<const cav_msgs::ByteArray::ConstPtr&>>>(outbound_sub_ptr_);

        mock_driver_node_.spin(10);

        return 0;
    }

}