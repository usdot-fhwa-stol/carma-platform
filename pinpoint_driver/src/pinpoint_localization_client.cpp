/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <pinpoint_driver/pinpoint_localization_client.h>
#include <iomanip>
#include <pinpoint_driver/unpack_macros.h>


const long long two_to_thirty_one = 2147483648;
const long long two_to_fifteenth  = 32768;

torc::PinPointLocalizationClient::PinPointLocalizationClient() 
{
    onConnect.connect(boost::bind(&PinPointLocalizationClient::connectSignals,this));
}

void torc::PinPointLocalizationClient::processMessage(torc::MessageType msg_type, uint8_t msg_id, std::vector<uint8_t> &msg)
{
    switch (msg_type) 
    {
        case MessageType::ReturnValue: 
        {
            switch (msg_id) 
            {
                case 2: // get status return
                {
                    uint16_t number = UNPACK_UINT16(&msg[0]);
                    for (uint16_t i = 0; i < number; i++) 
                    {
                        struct PinPointLocalizationClient::PinPointStatusCode status;

                        status.condition = static_cast<StatusCondition>(UNPACK_UINT8(&msg[2 + i * 3]));
                        status.code = static_cast<StatusCode>(UNPACK_UINT16(&msg[3 + i * 3]));
                        onStatusConditionChanged(status);
                    }
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        case MessageType::SignalEmitted:
        {
            switch (msg_id) 
            {
                case 0: // status messages
                {
                    struct PinPointLocalizationClient::PinPointStatusCode status;

                    status.condition = static_cast<StatusCondition>(UNPACK_UINT8(&msg[0]));
                    status.code = static_cast<StatusCode>(UNPACK_UINT16(&msg[1]));

                    onStatusConditionChanged(status);
                    break;
                }
                case 6: // global pose signal emit
                {
                    struct PinPointGlobalPose pose;

                    pose.time = UNPACK_UINT64(&msg[0]);
                    pose.latitude = (180.0f / two_to_thirty_one) * (float) UNPACK_INT32(&msg[8]);
                    pose.longitude = (180.0f / two_to_thirty_one) * (float) UNPACK_INT32(&msg[12]);
                    pose.altitude = (float) UNPACK_INT32(&msg[16]) / (float) 1000.0;
                    pose.roll = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[20]);
                    pose.pitch = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[22]);
                    pose.yaw = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[24]);

                    onGlobalPoseChanged(pose);
                    break;
                }
                case 7:// local pose emit
                {
                    struct PinPointLocalPose pose;

                    pose.time = UNPACK_UINT64(&msg[0]);
                    pose.north = (float) UNPACK_INT32(&msg[8]) / (float) 1000.0;
                    pose.east = (float) UNPACK_INT32(&msg[12]) / (float) 1000.0;
                    pose.down = (float) UNPACK_INT32(&msg[16]) / (float) 1000.0;
                    pose.roll = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[20]);
                    pose.pitch = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[22]);
                    pose.yaw = (180.0f / two_to_fifteenth) * (float) UNPACK_INT16(&msg[24]);

                    onLocalPoseChanged(pose);
                    break;
                }
                case 8: // velocity emit
                {
                    struct PinPointVelocity velocity;

                    velocity.time = UNPACK_UINT64(&msg[0]);
                    velocity.forward_vel = (float) UNPACK_INT24(&msg[8]) / (float) 1000.0;
                    velocity.right_vel = (float) UNPACK_INT24(&msg[11]) / (float) 1000.0;
                    velocity.down_vel = (float) UNPACK_INT24(&msg[14]) / (float) 1000.0;
                    velocity.roll_rate = (float) UNPACK_INT24(&msg[17]) / (float) 1000.0;
                    velocity.pitch_rate = (float) UNPACK_INT24(&msg[20]) / (float) 1000.0;
                    velocity.yaw_rate = (float) UNPACK_INT24(&msg[23]) / (float) 1000.0;

                    onVelocityChanged(velocity);
                    break;
                }
                case 9: // quaternion_covariance emit
                {
                    struct PinPointQuaternionCovariance quat;

                    quat.time = UNPACK_UINT64(&msg[0]);
                    quat.quaternion[0] = UNPACK_FLOAT(&msg[8]);
                    quat.quaternion[1] = UNPACK_FLOAT(&msg[12]);
                    quat.quaternion[2] = UNPACK_FLOAT(&msg[16]);
                    quat.quaternion[3] = UNPACK_FLOAT(&msg[20]);

                    for (int i = 0; i < 3; i++) 
                    {
                        for (int j = 0; j < 3; j++) 
                        {
                            int idx = i * 3 + j;
                            quat.covariance[i][j] = UNPACK_FLOAT(&msg[24 + idx]);
                        }
                    }

                    onQuaternionCovarianceChanged(quat);
                    break;
                }
                case 10: // filter accuracy emmit
                {
                    struct PinPointFilterAccuracy acc;

                    acc.time = UNPACK_UINT64(&msg[0]);
                    acc.position.north = UNPACK_FLOAT(&msg[8]);
                    acc.position.east = UNPACK_FLOAT(&msg[12]);
                    acc.position.down = UNPACK_FLOAT(&msg[16]);

                    acc.velocity.north = UNPACK_FLOAT(&msg[20]);
                    acc.velocity.east = UNPACK_FLOAT(&msg[24]);
                    acc.velocity.down = UNPACK_FLOAT(&msg[28]);

                    acc.rotation.north = UNPACK_FLOAT(&msg[32]);
                    acc.rotation.east = UNPACK_FLOAT(&msg[36]);
                    acc.rotation.down = UNPACK_FLOAT(&msg[40]);

                    onFilterAccuracyChanged(acc);
                    break;
                }
                default:
                {
                    break;
                }
            }
            break;
        }
        default:
        {
            break;
        }
    }
}

void torc::PinPointLocalizationClient::connectSignals() 
{
    std::vector<uint8_t> signalIDs =
            {
                    0, // statusChanged
                    6, // globalPoseChanged
                    7, // localPoseChanged
                    8, // velocityStateChanged
                    9, // quaternionCovarianceChanged
                    10 // filterAccuracyChanged
            };

    std::for_each(signalIDs.begin(),signalIDs.end(),[this](uint8_t n){connectSignal(n);});

    //We need to get the status because if they aren't changing we wont get the initial state
    getStatus();
}