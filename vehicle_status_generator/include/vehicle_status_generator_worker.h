#pragma once

/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include <vector>
#include <ros/ros.h>
#include <stdint.h>
#include <algorithm>

namespace vehicle_status_generator
{
    class VehicleStatusGeneratorWorker
    {
        public:
            
            VehicleStatusGeneratorWorker();
            uint8_t getNextMsgCount();
            std::vector<uint8_t> getMsgId(const ros::Time now);
            uint16_t getSecMark(const ros::Time now);

        private:

            // variables for BSM generation
            uint8_t msg_count_ {0};
            int random_id_ {0};
            ros::Time last_id_generation_time_;
    };
}
