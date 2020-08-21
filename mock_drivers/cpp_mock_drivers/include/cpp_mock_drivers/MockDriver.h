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
#include <carma_simulation_msgs/BagData.h>
#include <cav_msgs/DriverStatus.h>

#include "cpp_mock_drivers/ROSComms.h"
#include "cpp_mock_drivers/MockDriverNode.h"
#include "cpp_mock_drivers/comm_types.h"

namespace mock_drivers{

    /*! \brief The template node for the mock drivers that will handle all of the driver logic
    *
    * This class will have virtual functions that define what to do when the mock driver is run,
    * as well as a callback function for when the driver gets a message from the bag parser node.
    * 
    * It will also have build in default mock driver publishers and subscribers baked in, such
    * as the driver discovery publisher.
    */

    class MockDriver{

        protected:

        MockDriverNode mock_driver_node_;
        
        std::function<void(const carma_simulation_msgs::BagData::ConstPtr&)> bag_parser_cb_ptr_ = std::bind(&MockDriver::parserCB, this, std::placeholders::_1);
        boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>> bag_parser_sub_ptr_ = boost::make_shared<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>>(ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>(bag_parser_cb_ptr_, CommTypes::sub, false, 10, "bag_data"));
        
        boost::shared_ptr<ROSComms<cav_msgs::DriverStatus>> driver_discovery_pub_ptr_ = boost::make_shared<ROSComms<cav_msgs::DriverStatus>>(ROSComms<cav_msgs::DriverStatus>(CommTypes::pub, false, 10, "driver_discovery"));

        public:

        /*! \brief A function to initialize the publishers and subsricers and start the node */        
        virtual int run() = 0;
        
        /*! \brief A function to support the bag_parser callback that all mock drivers support */      
        virtual void parserCB(const carma_simulation_msgs::BagData::ConstPtr& msg) = 0;
        
        /*! \brief A function to call at 1 Hz to publish to the driver discovery topic */                
        virtual bool driverDiscovery() = 0;

        boost::shared_ptr<ROSComms<const carma_simulation_msgs::BagData::ConstPtr&>> getBagComms() {return bag_parser_sub_ptr_;};

        /*! \brief Returns the mock driver node for the mock driver (used for testing) */        
        MockDriverNode getMockDriverNode() {return mock_driver_node_;}
        

    };
    
}