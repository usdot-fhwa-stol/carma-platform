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

#include "bsm_generator_worker.h"
#include <stdlib.h>
#include <ros/ros.h>

namespace bsm_generator
{
    BSMGeneratorWorker::BSMGeneratorWorker() : msg_count(0) {}
    
    uint8_t BSMGeneratorWorker::getNextMsgCount()
    {
        uint8_t old_msg_count = msg_count++;
        if(msg_count == 128)
        {
            msg_count = 0;
        }
        return old_msg_count;
    }

    std::vector<uint8_t> BSMGeneratorWorker::getMsgId(const ros::Time now)
    {
        std::vector<uint8_t> id(4);
        // need to change ID every 5 mins
        ros::Duration id_timeout(60 * 5);
        if(now - last_id_generation_time >= id_timeout)
        {
            random_id = rand();
            last_id_generation_time = now;
        }
        for(int i = 0; i < id.size(); ++i)
        {
            id[i] = random_id >> (8 * i);
        }
        return id;
    }

    uint16_t BSMGeneratorWorker::getSecMark(const ros::Time now)
    {
        return static_cast<uint16_t>((now.toNSec() / 1000000) % 60000);
    }

    float BSMGeneratorWorker::getSpeedInRange(const double speed)
    {
        return static_cast<float>(std::max(std::min(speed, 163.8), 0.0));
    }

    float BSMGeneratorWorker::getSteerWheelAngleInRnage(const double angle)
    {
        return static_cast<float>(std::max(std::min(angle * 57.2958, 189.0), -189.0));
    }

    float BSMGeneratorWorker::getLongAccelInRange(const float accel)
    {
        return std::max(std::min(accel, 20.0f), -20.0f);
    }

    float BSMGeneratorWorker::getYawRateInRange(const double yaw_rate)
    {
        return static_cast<float>(std::max(std::min(yaw_rate, 327.67), -327.67));
    }

    uint8_t BSMGeneratorWorker::getBrakeAppliedStatus(const double brake)
    {
        return brake >= 0.05 ? 0b1111 : 0;
    }

    float BSMGeneratorWorker::getHeadingInRange(const float heading)
    {
        return std::max(std::min(heading, 359.9875f), 0.0f);
    }
}
