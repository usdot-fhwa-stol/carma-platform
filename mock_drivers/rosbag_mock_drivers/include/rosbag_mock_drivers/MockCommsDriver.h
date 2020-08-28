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

#include "rosbag_mock_drivers/MockDriver.h"
#include <cav_msgs/ByteArray.h>


namespace mock_drivers{

    class MockCommsDriver : public MockDriver {

        private:

            ConstPtrRefROSCommsPtr<cav_msgs::ByteArray> outbound_sub_ptr_;

            const std::string inbound_binary_topic_ = "inbound_binary_msg";
            const std::string outbound_binary_topic_ = "outbound_binary_msg";

            void outboundCallback(const cav_msgs::ByteArray::ConstPtr& msg);

        public:

            MockCommsDriver(bool dummy = false);
            int run();
            bool driverDiscovery();

    };

}