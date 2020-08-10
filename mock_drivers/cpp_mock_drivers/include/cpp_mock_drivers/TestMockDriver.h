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

#include "cpp_mock_drivers/MockDriver.h"
#include "std_msgs/String.h"


namespace mock_drivers{

    class TestMockDriver : public MockDriver {

        private:

            ROSComms<std_msgs::String> test_pub_;
            ROSComms<const std_msgs::String::ConstPtr&> test_sub_;

            void chatterCallback(const std_msgs::String::ConstPtr& msg);
            void parserCB(const cav_msgs::BagParserMsg::ConstPtr& msg);

        public:

            TestMockDriver();
            int run();

    };

}