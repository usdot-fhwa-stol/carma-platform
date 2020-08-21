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
#include <vector>

namespace mock_drivers{

    class MockDriverNode {

        private:
            boost::shared_ptr<ros::CARMANodeHandle> cnh_;
            std::vector<ros::Publisher> publishers_;
            std::vector<ros::Subscriber> subscribers_;
            std::vector<ros::ServiceServer> services_;
            
            // used for testing
            bool dummy_; // if the mock driver node is a dummy then it wont initialize a CARMANodeHandle and its functions wont do anything
            std::vector<std::string> topics_;
            std::vector<ros::Time> time_stamps_;

        public:

            /*! \brief Function to add a publisher from a ROSComms object */
            template<typename T>
            void addPub(T comm){
                if(!dummy_){
                    publishers_.push_back(cnh_->advertise<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize()));
                }
            }

            /*! \brief Function to add a subscriber from a ROSComms object */
            template<typename T>
            void addSub(T comm){
                if(!dummy_){
                    subscribers_.push_back(cnh_->subscribe<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize(), &ROSComms<decltype(comm->getTemplateType())>::callback, comm));
                }
            }

            /*! \brief Function to add a service from a ROSComms object */
            template<typename T>
            void addSrv(T comm){
                if(!dummy_){
                    services_.push_back(cnh_->advertiseService(comm->getTopic(), &ROSComms<decltype(comm->getReqType()), decltype(comm->getResType())>::callback, comm));
                }
            }

            // TODO (needs at least C++ 17): use a constexpr if statement to an addComms function so you don't need different add functions
            // template<typename T>
            // void addComms(T comm) {
            //     if constexpr(comm->getCommType() == CommTypes::pub){
            //         publishers_ = cnh_->advertise<decltype(comm->getTemplateType())>(comm->getTopic(), comm->getQueueSize());
            //     } else if constexpr(comm->getCommType() == CommTypes::sub){
            //         subscribers_ = cnh_->subscribe<const std_msgs::String::ConstPtr&>(comm->getTopic(), comm->getQueueSize(), &ROSComms<decltype(comm->getTemplateType())>::callback, comm);
            //     } else if constexpr(comm->getCommType() == CommTypes::srv){
            //         services_.push_back(cnh_->advertiseService(comm->getTopic(), &ROSComms<decltype(comm->getReqType()), decltype(comm->getResType())>::callback, comm));
            //     }
            // }            

            /*! \brief Begin the ros node*/
            void spin(double rate);

            /*! \brief Set the spin callback for the ros node*/
            void setSpinCallback(std::function<bool()> cb);

            /*! \brief Initialize the CARMA Node Handle pointer for the MockDriverNode (must be called before spin)*/
            void init();

            /*! \brief Publish data on a desired topic
            * 
            * This function must take in the full name of topic that will be published including the namespaces and leading /.
            * This can probably be made to take that information in on construction of the node but we can add that once it breaks :)
            */
            template<typename T>
            void publishData(std::string topic, T msg, bool header = true){
                if(!dummy_){
                    std::vector<ros::Publisher>::iterator pub = std::find_if(publishers_.begin(), publishers_.end(), [&](ros::Publisher p){return (p.getTopic()) == topic;});
                    pub->publish(msg);
                } else {
                    topics_.push_back(topic);
                    time_stamps_.push_back(msg.header.stamp);
                }
            };

            /*! \brief Publish data with no header on a desired topic
            * 
            * Same as the publishData function, except this is used when the data doesn't have a header. This exists to allow for testing of the code.
            * This can be combined with publishData once we are in c++ 17 and can use constexpr if statement.  
            */
            template<typename T>
            void publishDataNoHeader(std::string topic, T msg){
                if(!dummy_){
                    std::vector<ros::Publisher>::iterator pub = std::find_if(publishers_.begin(), publishers_.end(), [&](ros::Publisher p){return (p.getTopic()) == topic;});

                    pub->publish(msg);
                } else {
                    topics_.push_back(topic);
                }
            };

            MockDriverNode();
            MockDriverNode(bool dummy);

            /*! \brief Returns a vector of all the topics that the node would publish to (only when it is a dummy node). Used for testing*/
            std::vector<std::string> getTopics(){return topics_;}
            /*! \brief Returns a vector of all the time stamps of the data that would be published (only when it is a dummy node). Used for testing*/
            std::vector<ros::Time> getTimeStamps(){return time_stamps_;}
            /*! \brief Returns if the node is a dummy node. Used for testing*/
            bool isDummy(){return dummy_;}

    };
    
}