#pragma once
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

#include "pinpoint_driver.h"
#include <sstream>

namespace torc
{

/**
 * @brief PinPoint localpose structure
 *
 * @var time microseconds since 1970
 * @var north       north (m)
 * @var east        east (m)
 * @var down        down (m)
 * @var roll        Roll (deg)
 * @var pitch       Pitch (deg)
 * @var yaw         Yaw(deg)
 *
 */
struct PinPointLocalPose {
    uint64_t time;
    double north;
    double east;
    double down;
    float roll;
    float pitch;
    float yaw;

};


/**
 * @brief PinPoint quarternion covariance structure
 *
 * @var time microseconds since 1970
 * @var quaternion   quaternion rotation (w,x,y,z)
 * @var east        covariance of rotation (x.y.z) - row major order
 *
 */
struct PinPointQuaternionCovariance
{
    uint64_t time;
    float quaternion[4];
    float covariance[3][3];
};


/**
 * @brief PinPoint globalpose structure
 *
 * @var time microseconds since 1970
 * @var latitude    latitude (deg)
 * @var longitude   longitude (deg)
 * @var altitude    altitude (deg)
 * @var roll        Roll (deg)
 * @var pitch       Pitch (deg)
 * @var yaw         Yaw(deg)
 *
 */
struct PinPointGlobalPose {
    uint64_t time;
    float latitude;
    float longitude;
    float altitude;
    float roll;
    float pitch;
    float yaw;
};


/**
 * @brief PinPoint velocity structure
 *
 * @var time microseconds since 1970
 * @var forward_vel Forward (m/s)
 * @var right_vel   Right (m/s)
 * @var down_vel    Down (m/s)
 * @var roll_rate   Roll (rad/s)
 * @var pitch_rate  Pitch (rad/s)
 * @var yaw_rate    Yaw(rad/s)
 *
 */
struct PinPointVelocity {
    uint64_t time;
    float forward_vel;
    float right_vel;
    float down_vel;
    float roll_rate;
    float pitch_rate;
    float yaw_rate;

};

template <typename T>
struct VectorNED
{
    T north;
    T east;
    T down;
};

struct PinPointFilterAccuracy
{
    uint64_t time;
    VectorNED<float> position;
    VectorNED<float> velocity;
    VectorNED<float> rotation;
};


/**
 * @brief Implements the PinPoint driver connecting to the PinPoint localization server
 */
class PinPointLocalizationClient : public PinPoint
{
public:

    /**
     * @enum StatusCode
     * @brief Status Codes provided by the PinPoint localization server
     */
    enum class StatusCode : uint16_t
    {
        Aligning = 1,
        NoImuData = 2,
        NoGpsUpdates = 3,
        NoLeftWssUpdates = 4,
        NoRightWssUpdates = 5,
        BadGpsPosAgreement = 6,
        BadGpsVelAgreement = 7,
        BadWssVelAgreement = 8,
        BadGyroBiasEstimate = 9,
        BadAccelBiasEstimate = 10,
        PoseSteadying = 11,
        NoHeadingUpdates = 12,
        BadHeadingAgreement = 13,
        BadMeasurementTime = 14,
        IrregularTimeStep = 15
    };


    struct PinPointStatusCode
    {
        torc::StatusCondition condition;
        PinPointLocalizationClient::StatusCode code;
    };

    PinPointLocalizationClient();

    /**
     * @brief Signaled onGlobalPoseChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointGlobalPose const &)> onGlobalPoseChanged;


    /**
     * @brief Signaled onLocalPoseChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointLocalPose const &)> onLocalPoseChanged;


    /**
     * @brief Signaled onVelocityChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointVelocity const &)> onVelocityChanged;


    /**
     * @brief Signaled onFilterAccuracyChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointFilterAccuracy const &)> onFilterAccuracyChanged;


    /**
     * @brief Signaled onQuaternionCovarianceChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointQuaternionCovariance const &)> onQuaternionCovarianceChanged;


    /**
     * @brief Signaled onStatusConditionChanged message received from PinPoint device
     */
    boost::signals2::signal<void (struct PinPointLocalizationClient::PinPointStatusCode const &)> onStatusConditionChanged;


protected:

    /**
     * @brief Processes messages received from PinPoint device
     * @param msg_type
     * @param msg_id
     * @param msg
     */
    virtual void processMessage(MessageType msg_type, uint8_t msg_id, std::vector<uint8_t>& msg) override;

private:

    /**
     * @brief Connect the signals PinPointLocalizationClient is interested in
     */
    void connectSignals();

};

}
