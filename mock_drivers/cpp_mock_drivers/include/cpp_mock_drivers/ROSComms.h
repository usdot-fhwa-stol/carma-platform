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

#include "comm_types.h"
#include "std_msgs/String.h"

namespace mock_drivers{

    template<typename...> class ROSComms;

    template <typename T>
    // T is message type
    class ROSComms<T>{

        private:

            // std::function<void(const std_msgs::String::ConstPtr&)> callback_function_;
            std::function<void(T)> callback_function_;
            CommTypes comm_type_;
            bool latch_;
            int queue_size_;
            std::string topic_;

        public:

            bool getLatch(){return latch_;}
            int getQueueSize(){return queue_size_;}
            std::string getTopic(){return topic_;}
            CommTypes getCommType(){return comm_type_;}

            void callback(T msg);
            T getTemplateType();
            ROSComms();
            ROSComms(CommTypes ct, bool latch, int qs, std::string t);
            ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, std::string t);

    };

    template <typename M, typename T>
    class ROSComms<M,T>{

        private:

            std::function<void(M, T)> callback_function_;
            CommTypes comm_type_;
            std::string topic_;
        
        public:

            std::string getTopic(){return topic_;}
            CommTypes getCommType(){return comm_type_;}

            bool callback(M req, T res);
            M getReqType();
            T getResType();
            ROSComms();
            ROSComms(std::function<bool(M, T)> cbf, CommTypes ct, std::string t);

    };
}

#include "ROSComms.ipp"