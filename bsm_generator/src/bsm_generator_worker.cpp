/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include "bsm_generator/bsm_generator_worker.hpp"
#include <random>

namespace bsm_generator
{
    
    BSMGeneratorWorker::BSMGeneratorWorker() {}
    
    uint8_t BSMGeneratorWorker::getNextMsgCount()
    {
        uint8_t old_msg_count = msg_count_;
        msg_count_++;
        if(msg_count_ == 128)
        {
            msg_count_ = 0;
        }
        return old_msg_count;
    }

    std::vector<uint8_t> BSMGeneratorWorker::getMsgId(const rclcpp::Time now, double secs)
    {
        // need to change ID every designated period
        rclcpp::Duration id_timeout = rclcpp::Duration::from_nanoseconds(secs * 1e9);

        generator_.seed(std::random_device{}()); // guarantee randomness
        std::uniform_int_distribution<int> dis(0,INT_MAX);
        
        std::vector<uint8_t> id(4);

        if (first_msg_id_) {
            last_id_generation_time_ = now;
            first_msg_id_ = false;
            random_id_ = dis(generator_);
        }
        
        if(now - last_id_generation_time_ >= id_timeout)
        {
            random_id_ = dis(generator_);
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("bsm_generator"), "Newly generated random id: " << random_id_);
            last_id_generation_time_ = now;
        }

        for(int i = 0; i < id.size(); ++i)
        {
            id[i] = random_id_ >> (8 * i);
        }
        return id;
    }

    uint16_t BSMGeneratorWorker::getSecMark(const rclcpp::Time now)
    {
        return static_cast<uint16_t>((now.nanoseconds() / 1000000) % 60000);
    }

    float BSMGeneratorWorker::getSpeedInRange(const double speed)
    {
        return static_cast<float>(std::max(std::min(speed, 163.8), 0.0));
    }

    float BSMGeneratorWorker::getSteerWheelAngleInRange(const double angle)
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

    float BSMGeneratorWorker::getHeading(const geometry_msgs::msg::Quaternion & quaternion)
    {
        tf2::Quaternion orientation;
        orientation.setX(quaternion.x);
        orientation.setY(quaternion.y);
        orientation.setZ(quaternion.z);
        orientation.setW(quaternion.w);
      
        double roll;
        double pitch;
        double yaw;
        tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        // Convert yaw radians to degrees
        yaw = yaw * 180 / M_PI;
        // Convert to NED frame
        yaw = -yaw+90;
        // Convert to range [0, 360]
        yaw = std::fmod(yaw, 360.0);
        if ( yaw < 0  )
        {
          yaw = 360 + yaw;
        }
        return static_cast<float>(yaw);
    }
} // namespace bsm_generator