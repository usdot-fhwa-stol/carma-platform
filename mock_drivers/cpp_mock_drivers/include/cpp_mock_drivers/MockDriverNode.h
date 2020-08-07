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
#include <carma_utils/CARMAUtils.h>
#include "comm_types.h"
#include "ROSComms.h"
#include "iostream"
#include <std_msgs/String.h>

namespace mock_drivers{

    class MockDriverNode {

        private:
            // ros::CARMANodeHandle cnh_;
            ros::NodeHandle nh_;
            ros::Publisher publishers_;
            ros::Subscriber subscribers_;

        public:

// ros::Publisher chatter_pub = n.advertise<decltype(test_comms.getMessageType())>(test_comms.getTopic(), 1000);

            template<typename T>
            void addPub(T comm){
                std::cout << "test1" << std::endl;
                publishers_ = nh_.advertise<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize());
            }

            template<typename T>
            void addSub(T comm){
                std::cout << "test4" << std::endl;
                std::cout << comm->getTopic() << std::endl;
                // ros::Subscriber sub = n.subscribe("ooga_booga", 1000, &mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&>::callback, &test_comms);
                subscribers_ = nh_.subscribe<const std_msgs::String::ConstPtr&>(comm->getTopic(), comm->getQueueSize(), &ROSComms<decltype(comm->getTemplateType())>::callback, comm);
            }

            void spin(int rate);

            void publishData(int data);

    };
    
}