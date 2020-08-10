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
#include <vector>
#include <cav_srvs/SetEnableRobotic.h>

namespace mock_drivers{

    class MockDriverNode {

        private:
            ros::CARMANodeHandle cnh_;
            // ros::NodeHandle nh_;
            std::vector<ros::Publisher> publishers_;
            std::vector<ros::Subscriber> subscribers_;
            std::vector<ros::ServiceServer> services_;

        public:

            template<typename T>
            void addPub(T comm){
                publishers_.push_back(cnh_.advertise<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize()));
            }

            template<typename T>
            void addSub(T comm){
                subscribers_.push_back(cnh_.subscribe<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize(), &ROSComms<decltype(comm->getTemplateType())>::callback, comm));
            }

            template<typename T>
            void addSrv(T comm){
                services_.push_back(cnh_.advertiseService(comm->getTopic(), &ROSComms<decltype(comm->getReqType()), decltype(comm->getResType())>::callback, comm));
            }

            // TODO (needs at least C++ 17): use a constexpr if statement to an addComms function so you don't need different add functions
            // template<typename T>
            // void addComms(T comm) {
            //     if constexpr(comm->getCommType() == CommTypes::pub){
            //         std::cout << "Attaching publisher" << std::endl;
            //         publishers_ = cnh_.advertise<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize());
            //     } else if constexpr(comm->getCommType() == CommTypes::sub){
            //         std::cout << "Attaching subscriber" << std::endl;
            //         std::cout << comm->getTopic() << std::endl;
            //         // ros::Subscriber sub = n.subscribe("ooga_booga", 1000, &mock_drivers::ROSComms<std_msgs::String, const std_msgs::String::ConstPtr&>::callback, &test_comms);
            //         subscribers_ = cnh_.subscribe<const std_msgs::String::ConstPtr&>(comm->getTopic(), comm->getQueueSize(), &ROSComms<decltype(comm->getTemplateType())>::callback, comm);
            //     }
            // }            

            void spin(int rate);

            template<typename T>
            void publishData(std::string topic, T msg){
                std::vector<ros::Publisher>::iterator pub = std::find_if(publishers_.begin(), publishers_.end(), [&](ros::Publisher p){return (p.getTopic()) == topic;});
                pub->publish(msg);
            };

    };
    
}