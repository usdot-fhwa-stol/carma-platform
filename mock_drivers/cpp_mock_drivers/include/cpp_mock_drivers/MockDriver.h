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

#include "ROSComms.h"
#include "MockDriverNode.h"
#include "comm_types.h"

using namespace std;

namespace mock_drivers{

    class MockDriver{

        protected:

        string name_;
        MockDriverNode mock_driver_node_;
        // vector<ROSComms<class message>> ros_api_;

        ROSComms<std_msgs::String> test_pub_;
        ROSComms<const std_msgs::String::ConstPtr&> test_sub_;


        public:

        // vector<ROSComms<class message>> getAPI() {return ros_api_;} 
        virtual int run() = 0;

    };
    
}