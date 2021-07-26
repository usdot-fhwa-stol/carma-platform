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

#include "vehicle_status_generator_worker.h"
#include <stdlib.h>
#include <ros/ros.h>
#include <random>

namespace vehicle_status_generator
{
    VehicleStatusGeneratorWorker::VehicleStatusGeneratorWorker() {}
    
    uint8_t VehicleStatusGeneratorWorker::getNextMsgCount()
    {
        uint8_t old_msg_count = msg_count_;
        msg_count_++;
        if(msg_count_ == 128)
        {
            msg_count_ = 0;
        }
        return old_msg_count;
    }

    std::vector<uint8_t> VehicleStatusGeneratorWorker::getMsgId(const ros::Time now)
    {
        std::vector<uint8_t> id(4);
        // need to change ID every 5 mins
        ros::Duration id_timeout(60 * 5);

        std::default_random_engine generator;
        std::uniform_int_distribution<int> dis(0,INT_MAX);

        if(now - last_id_generation_time_ >= id_timeout)
        {
            random_id_ = dis(generator);
            last_id_generation_time_ = now;
        }
        for(int i = 0; i < id.size(); ++i)
        {
            id[i] = random_id_ >> (8 * i);
        }
        return id;
    }

}
