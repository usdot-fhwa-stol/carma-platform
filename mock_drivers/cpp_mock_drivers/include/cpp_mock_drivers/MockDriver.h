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

#include <ros/ros.h>
#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <cav_msgs/BagData.h>

#include "cpp_mock_drivers/ROSComms.h"
#include "cpp_mock_drivers/MockDriverNode.h"
#include "cpp_mock_drivers/comm_types.h"


namespace mock_drivers{

    class MockDriver{

        protected:

        MockDriverNode mock_driver_node_;
        
        std::function<void(const cav_msgs::BagData::ConstPtr&)> bag_parser_cb_ptr_ = std::bind(&MockDriver::parserCB, this, std::placeholders::_1);
        boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>> bag_parser_sub_ptr_ = boost::make_shared<ROSComms<const cav_msgs::BagData::ConstPtr&>>(ROSComms<const cav_msgs::BagData::ConstPtr&>(bag_parser_cb_ptr_, CommTypes::sub, false, 10, "bag_data"));

        public:

        virtual int run() = 0;
        virtual void parserCB(const cav_msgs::BagData::ConstPtr& msg) = 0;

        boost::shared_ptr<ROSComms<const cav_msgs::BagData::ConstPtr&>> getBagComms() {return bag_parser_sub_ptr_;};

        MockDriverNode getMockDriverNode() {return mock_driver_node_;}
        

    };
    
}