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

using namespace std;

namespace mock_drivers{

    class IComms
    {
        protected:
            CommTypes comm_type_;
            bool latch_;
            int queue_size_;
            string topic_;
        
        public:
            bool getLatch(){return latch_;}
            int getQueueSize(){return queue_size_;}
            string getTopic(){return topic_;}
            CommTypes getCommType(){return comm_type_;}
    };

    template<typename...> class ROSComms;

    template <typename T>
    // M is message type
    class ROSComms<T> : public IComms{

        private:

            // std::function<void(const std_msgs::String::ConstPtr&)> callback_function_;
            std::function<void(T)> callback_function_;

        public:

            void callback(T msg);
            T getTemplateType();
            ROSComms();
            ROSComms(CommTypes ct, bool latch, int qs, string t);
            ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, string t);

    };

    template <typename M, typename T>
    // M is message type, T is callback function parameter type
    class ROSComms<M,T> : public IComms{

        private:

            // std::function<void(const std_msgs::String::ConstPtr&)> callback_function_;
            std::function<void(T)> callback_function_;
        
        public:

            void callback(T msg);
            M getMessageType();
            T getParamType();
            ROSComms();
            ROSComms(std::function<void(T)> cbf, CommTypes ct, bool latch, int qs, string t);
            ROSComms(CommTypes ct, bool latch, int qs, string t);

    };
}

#include "ROSComms.ipp"