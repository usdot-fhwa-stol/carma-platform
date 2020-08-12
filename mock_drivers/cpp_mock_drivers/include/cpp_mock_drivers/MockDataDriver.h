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
#include <tf2_msgs/TFMessage.h>

namespace mock_drivers{

    class MockDataDriver : public MockDriver {

        private:

            boost::shared_ptr<ROSComms<tf2_msgs::TFMessage>> tf_ptr_;
            boost::shared_ptr<ROSComms<tf2_msgs::TFMessage>> tf_static_ptr_;

        public:

            MockDataDriver();
            int run();
            void parserCB(const cav_msgs::BagData::ConstPtr& msg);

    };

}