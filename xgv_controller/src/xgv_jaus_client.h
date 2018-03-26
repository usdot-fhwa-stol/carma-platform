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
#include <boost/signals2/signal.hpp>

/**
 * @brief A client to connect to the XGV Jaus Components
 */
class XGVJausClient
{
public:

    /**
     * @brief Configuration for the XGVJausClient
     */
    struct XGVJausClientConfig
    {
        unsigned char subsystem_id;             /**< Jaus Subsystem ID corresponding to the XGV ByWire */
        unsigned char node_id;                  /**< Jaus Node ID corresponding to the XGV ByWire */
        unsigned char authority_level;          /**< Authority level the XGVJausClient will use to control PD/MPD on XGV ByWire */
        unsigned short motion_profile_duration; /**< Motion Profile duration to be used */
    };

    /**
     * @brief Used for velocity state callback
     */
    struct VelocityStateEventArgs
    {
        double velocity;
    };

    /**
     * @brief Used for WrenchEffortCallback
     */
    struct WrenchEffortSignalArgs
    {
        double throttle;
        double brakes;
        double steering;
    };

    enum class ControlMode
    {
        Unknown,
        SoftwarePause,
        WrenchEffort,
        MotionProfile,
        Reserved
    };

    enum class ReadyMode
    {
        Unknown,
        SoftwarePause,
        Ready,
        Reserved
    };

    enum class SafetyMode
    {
        Unknown,
        DriveByWire,
        SoftwarePause,
        EStopPause,
        ManualOverride,
        EmergencyManualOverride,
        EStopStop,
        Reserved
    };

    enum class PartialMode
    {
        Computer,
        Manual
    };


    enum class LinkStatus
    {
        NoLink,
        ByPass,
        Good,
        Reserved
    };


    struct VehicleModeEventArgs
    {
        ControlMode controlMode;
        ReadyMode readyMode;
        SafetyMode safetyMode;
        bool manualSteering;
        bool manualSpeed;
        bool manualOverride;
        bool safeStopPause;
        bool safeStopStop;
        LinkStatus safeStopLinkStatus;
        bool externalEstop;
        bool doorPause;
        bool errorPause;
        bool emergencyManualOverride;
        bool steeringRequiresInitialization;
        bool steeringInitAwaitingUser;
    };


private:

    class XGVJausClientPimpl;
    friend class XGVJausClientPimpl;
    std::unique_ptr<XGVJausClientPimpl> pimpl_;
    
public:

    /**
     * @brief Constructor
     * @param cfg The configuration this client will use
     */
    explicit XGVJausClient(XGVJausClientConfig cfg);

    virtual ~XGVJausClient();

    /**
     * @brief Signaled on receipt of a velocity state report from the XGV ByWire
     */
    boost::signals2::signal<void(const VelocityStateEventArgs& state)> velocityReceivedSignal;


    /**
     * @brief Signaled on receipt of a wrench effort report from the XGV ByWire
     */
    boost::signals2::signal<void(const WrenchEffortSignalArgs& state)> wrenchEffortReceivedSignal;


    /**
     * @brief Signaled on receipt of Vehicle Mode Reprot from XGV ByWire
     */
    boost::signals2::signal<void(const VehicleModeEventArgs& args)> vehicleModeReceivedSignal;

    /**
     * @brief Connects the XGV Client to the NodeManager and begins processing the Jaus State and Callbacks
     * @return true on successful connect
     */
    bool connect();

    /**
     * @brief Shutsdown the underlying component
     */
    void shutdown();

    /**
     * @brief Sends a disable robotic command to the SRX DBW module
     */
    void disableRoboticControl();

    /**
     * @brief Sends a wrench effort command to the XGV Primitive Driver
     * @param effort [-100,100] percent effort, values out side of range are bounded to extremes
     */
    void setWrenchEffort(double effort);

    /**
     * @brief Sends a closed loop control command to the XGV Motion Profile
     * @param speed - set speed m/s
     * @param max_accel - max accel to use ( bounds the PID controller ) m/s^2
     */
    void setSpeedAccel(double speed, double max_accel);


};

inline std::ostream& operator<<(std::ostream& o, const XGVJausClient::ControlMode& cm)
{
    switch(cm)
    {
    case XGVJausClient::ControlMode::Unknown:
        return o << "Unknown";
    case XGVJausClient::ControlMode::SoftwarePause:
        return o << "SoftwarePause";
    case XGVJausClient::ControlMode::WrenchEffort:
        return o << "WrenchEffort";
    case XGVJausClient::ControlMode::MotionProfile:
        return o << "MotionProfile";
    case XGVJausClient::ControlMode::Reserved:
        return o << "Reserved";
    }
}


inline std::ostream& operator<<(std::ostream& o,const XGVJausClient::ReadyMode& rm)
{
    switch(rm)
    {
    case XGVJausClient::ReadyMode::Unknown:
        return o << "Unknown";
    case XGVJausClient::ReadyMode::SoftwarePause: 
        return o << "SoftwarePause";
    case XGVJausClient::ReadyMode::Ready:
        return o << "Ready";
    case XGVJausClient::ReadyMode::Reserved:
        return o << "Reserved";
    }
}

inline std::ostream& operator<<(std::ostream&o, const XGVJausClient::SafetyMode& sm)
{
    switch(sm)
    {
    case XGVJausClient::SafetyMode::Unknown:
        return o << "Unknown";
    case XGVJausClient::SafetyMode::DriveByWire:
        return o << "DriveByWire";
    case XGVJausClient::SafetyMode::SoftwarePause:
        return o << "SoftwarePause";
    case XGVJausClient::SafetyMode::EStopPause:
        return o << "EStopPause";
    case XGVJausClient::SafetyMode::ManualOverride:
        return o << "ManualOverride";
    case XGVJausClient::SafetyMode::EmergencyManualOverride:
        return o << "EmergencyManualOverride";
    case XGVJausClient::SafetyMode::EStopStop:
        return o << "EStopStop";
    case XGVJausClient::SafetyMode::Reserved:
        return o << "Reserved";
    }
}

inline std::ostream& operator<<(std::ostream& o, const XGVJausClient::LinkStatus& ls)
{
    switch(ls)
    {
    case XGVJausClient::LinkStatus::NoLink:
        return o << "NoLink";
    case XGVJausClient::LinkStatus::ByPass:
        return o << "ByPass";
    case XGVJausClient::LinkStatus::Good:
        return o << "Good";
    case XGVJausClient::LinkStatus::Reserved:
        return o << "Reserved";
    }
}

inline std::ostream& operator<<(std::ostream &o,const XGVJausClient::VehicleModeEventArgs& args) 
{ 
    return o << "Control Mode: " << args.controlMode << std::endl
           <<  "Ready Mode: " << args.readyMode << std::endl
           <<  "SafetyMode: " << args.safetyMode << std::endl
           << "Manual Steering: " << args.manualSteering << std::endl
           << "Manual Speed: " << args.manualSpeed << std::endl
           << "Manual Override: " << args.manualOverride << std::endl
           << "SafeStop Pause: " << args.safeStopPause << std::endl
           << "safeStopStop: " << args.safeStopStop << std::endl
           << "LinkStatus: " << args.safeStopLinkStatus   << std::endl
           << "External Estop: " << args.externalEstop << std::endl
           << "DoorPause: " << args.doorPause << std::endl
           << "Error Pause: " << args.errorPause << std::endl
           << "EmergencyManualOverride: " << args.emergencyManualOverride << std::endl
           << "steeringRequiresInitialization: " << args.steeringRequiresInitialization << std::endl
           << "steeringInitAwaitingUser: " << args.steeringInitAwaitingUser << std::endl;
}