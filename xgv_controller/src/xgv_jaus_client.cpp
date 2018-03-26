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

#include "xgv_jaus_client.h"

#include <xgv_client/xgv_client.h>
#include <openjaus/openJaus.h>
#include <mutex>


/**
 * @brief translates a JausByte to the XGVClient::ControlMode
 * @param controlMode
 * @return
 */
XGVJausClient::ControlMode toControlMode(JausByte controlMode)
{
    switch(controlMode)
    {
    case 0:
        return XGVJausClient::ControlMode::Unknown;

    case 1:
        return XGVJausClient::ControlMode::SoftwarePause;

    case 2:
        return XGVJausClient::ControlMode::WrenchEffort;

    case 3:
        return XGVJausClient::ControlMode::MotionProfile;

    default:
        return XGVJausClient::ControlMode::Reserved;
    }
}


/**
 * @brief translates a JausByte to the XGVClient::ReadyMode
 * @param controlMode
 * @return
 */
XGVJausClient::ReadyMode toReadyMode(JausByte readyMode)
{
    switch(readyMode)
    {
        case 0:
            return XGVJausClient::ReadyMode::Unknown;
        case 1:
            return XGVJausClient::ReadyMode::SoftwarePause;
        case 2:
            return XGVJausClient::ReadyMode::Ready;
        default:
            return XGVJausClient::ReadyMode::Reserved;
    }
}


/**
 * @brief translates a JausByte to the XGVClient::SafetyMode
 * @param controlMode
 * @return
 */
XGVJausClient::SafetyMode toSafetyMode(JausByte safetyMode)
{
    switch(safetyMode)
    {
        case 0:
            return XGVJausClient::SafetyMode::Unknown;
        case 1:
            return XGVJausClient::SafetyMode ::DriveByWire;
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
            return XGVJausClient::SafetyMode::SoftwarePause;
        case 8:
            return XGVJausClient::SafetyMode::EStopPause;
        case 9:
            return XGVJausClient::SafetyMode::ManualOverride;
        case 10:
            return XGVJausClient::SafetyMode::EmergencyManualOverride;
        case 11:
            return XGVJausClient::SafetyMode::EStopStop;
        default:
            return XGVJausClient::SafetyMode::Reserved;
    }
}


/**
 * @brief translates a JausUnsignedShort to the XGVClient::LinkStatus
 * @param controlMode
 * @return
 */
XGVJausClient::LinkStatus toLinkStatus(JausUnsignedShort byte)
{
    JausUnsignedShort bits = static_cast<JausUnsignedShort>(byte & (0x3 << 3));
    bits >>= 3;
    switch(bits)
    {
        case 0:
            return XGVJausClient::LinkStatus::NoLink;
        case 1:
            return XGVJausClient::LinkStatus::ByPass;
        case 2:
            return XGVJausClient::LinkStatus::Good;
        default:
            return XGVJausClient::LinkStatus::NoLink;
    }
}

/**
 * @brief Private implementation of XGVJausClient
 */
class XGVJausClient::XGVJausClientPimpl
{
private:

    const JausByte JAUS_XGVCLIENT = 4;

    enum class ControlMode
    {
        WrenchEffort,
        MotionProfile,
        DisableRobotic,
        None
    };

    enum class Gear
    {
        Park,
        Low,
        Drive,
        Neutral,
        Reverse,
        Unknown
    };

    bool control_pd_, control_mp_;
    bool pd_ready_{}, mp_ready_{};
    std::unique_ptr<JausAddressStruct, decltype(&jausAddressDestroy)> primitive_driver_addr_;
    std::unique_ptr<JausAddressStruct, decltype(&jausAddressDestroy)> motion_profile_driver_addr_;

    ControlMode control_mode_;
    double wrench_effort_{};

    std::mutex speed_ctrl_mutex_;
    double set_speed_{};
    double max_accel_{};

    Gear xgv_current_gear_;

    std::unique_ptr<OpenJausComponent> cmpt_;
    std::vector<int> service_handles_;
    XGVJausClientConfig cfg_;

    XGVJausClient* parent_;

public:
    XGVJausClientPimpl(XGVJausClientConfig cfg, XGVJausClient* parent) :
        cmpt_(nullptr), 
        cfg_(cfg), 
        control_mode_(ControlMode::None),
        primitive_driver_addr_(jausAddressCreate(),jausAddressDestroy),
        motion_profile_driver_addr_(jausAddressCreate(),jausAddressDestroy),
        control_pd_(false),
        control_mp_(false),
        parent_(parent)
    {
        primitive_driver_addr_->subsystem   = cfg_.subsystem_id;
        primitive_driver_addr_->node        = cfg_.node_id;
        primitive_driver_addr_->component   = JAUS_PRIMITIVE_DRIVER;
        primitive_driver_addr_->instance    = 1;

        motion_profile_driver_addr_->subsystem   = cfg_.subsystem_id;
        motion_profile_driver_addr_->node        = cfg_.node_id;
        motion_profile_driver_addr_->component   = JAUS_MOTION_PROFILE_DRIVER;
        motion_profile_driver_addr_->instance    = 1;

    }

    /**
     * @brief Connects the XGV Client to the NodeManager and begins processing the Jaus State and Callbacks
     * @return true on successful connect
     */
    bool connect()
    {
        cmpt_.reset(new OpenJausComponent("XGV Longitudinal Client", JAUS_XGVCLIENT, 50.0));
        if(!cmpt_->valid())
            return false;

        cmpt_->addService(JAUS_XGVCLIENT);
        cmpt_->addServiceInputMessage(JAUS_XGVCLIENT, JAUS_REPORT_WRENCH_EFFORT,0xFF);

        cmpt_->addServiceOutputMessage(JAUS_XGVCLIENT, JAUS_SET_WRENCH_EFFORT, 0xFF);
        cmpt_->addServiceOutputMessage(JAUS_XGVCLIENT, JAUS_SET_MOTION_PROFILE, 0xFF);
        cmpt_->addServiceOutputMessage(JAUS_XGVCLIENT, JAUS_REQUEST_COMPONENT_CONTROL, 0xFF);
        cmpt_->addServiceOutputMessage(JAUS_XGVCLIENT, JAUS_RESUME, 0xFF);
        cmpt_->addServiceOutputMessage(JAUS_XGVCLIENT, JAUS_SET_DISCRETE_DEVICES, 0xFF);
        cmpt_->setUserData(this);

        cmpt_->setAuthority(cfg_.authority_level);
        cmpt_->setStateCallback(JAUS_READY_STATE, &XGVJausClientPimpl::setReadyState);
        cmpt_->setState(JAUS_READY_STATE);
        cmpt_->run();

        register_msg_handlers();
        create_svc_connections();

        return cmpt_->valid();
    }


    /**
     * @brief Shutsdown the underlying component
     */
    void shutdown()
    {
        control_mode_ = ControlMode::None;

        if(cmpt_)
        {
            cmpt_.reset(nullptr);
        }
    }

    /**
     * @brief Sends a disable robotic command to the SRX DBW module
     */
    void disableRoboticControl()
    {
        control_mode_ = ControlMode::DisableRobotic;
    }

    /**
     * @brief Sends a wrench effort command to the XGV Primitive Driver
     * @param effort [-100,100] percent effort, values out side of range are bounded to extremes
     */
    void setWrenchEffort(double effort)
    {
        wrench_effort_ = effort;
        control_mode_ = ControlMode::WrenchEffort;
    }

    /**
     * @brief Sends a closed loop control command to the XGV Motion Profile
     * @param speed - set speed m/s
     * @param max_accel - max accel to use ( bounds the PID controller ) m/s^2
     */
    void setSpeedAccel(double speed, double max_accel)
    {
        std::lock_guard<std::mutex> lock(speed_ctrl_mutex_);
        set_speed_ = speed;
        max_accel_ = max_accel;
        control_mode_ = ControlMode::MotionProfile;
    }

private:


    /**
     * @brief This is the callback assigned to the components set state handler
     *
     * This gets called at approximate the rate set when creating the component.
     * @param cmpt
     */
    static void setReadyState(OjCmpt cmpt)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));
        switch(self->control_mode_)
        {
        case ControlMode::WrenchEffort:
            {
                if(!self->control_pd_)  //Do we not control primitive driver?
                {
                    self->send_req_ctrl_msg(JAUS_PRIMITIVE_DRIVER);
                }

                if(!self->pd_ready_)    //Is primitive driver not ready state?
                {
                    self->send_resume_msg(JAUS_PRIMITIVE_DRIVER);
                }

                if(!(self->xgv_current_gear_ == Gear::Drive)) //Is the gear not set to Drive
                {
                    self->send_set_drive();
                }

                self->send_set_wrench_effort_message();
            }
            break;
        case ControlMode::MotionProfile:
            {
                if(self->control_pd_) //Do wecontrol primitive driver?, if So we need to release control
                {
                    self->send_rel_ctrl_msg(JAUS_PRIMITIVE_DRIVER);
                }

                if(!self->pd_ready_)    //Is primitive driver not ready state?
                {
                    self->send_resume_msg(JAUS_PRIMITIVE_DRIVER);
                }

                if(!self->control_mp_) // Do we not control the motion profile driver
                {
                    self->send_req_ctrl_msg(JAUS_MOTION_PROFILE_DRIVER);
                }

                if(!self->mp_ready_)    //Is motion profile driver not ready state?
                {
                    self->send_resume_msg(JAUS_MOTION_PROFILE_DRIVER);
                }

                self->send_set_motion_profile_message();
            }
            break;
        case ControlMode::DisableRobotic:
            {
                //Rlease control of both drivers this will put the XGV in software pause.
                if(self->control_pd_)
                    self->send_rel_ctrl_msg(JAUS_PRIMITIVE_DRIVER);
                
                if(self->control_mp_)
                    self->send_rel_ctrl_msg(JAUS_MOTION_PROFILE_DRIVER);
            }
            break;
        case ControlMode::None:
            break;
        }

        self->control_mode_ = ControlMode::None;
    }

    /**
     * @brief Handles a Report Velocity StateMessage. Packs the data in an XGVCLient type and calls corresponding signal
     * @param cmpt
     * @param msg
     */
    static void onReportVelocityStateMessageHandler(OjCmpt cmpt, JausMessage msg)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));
        std::unique_ptr<ReportVelocityStateMessageStruct, decltype(&reportVelocityStateMessageDestroy)>
            reportVelocityState{reportVelocityStateMessageFromJausMessage(msg),reportVelocityStateMessageDestroy};

        if (reportVelocityState)
        {
            XGVJausClient::VelocityStateEventArgs args{};
            args.velocity = reportVelocityState->velocityXMps;

            self->parent_->velocityReceivedSignal(args);
        }

    }

    /**
     * @brief Handles a Report Wrench EffortMessage. Packs the data in an XGVCLient type and calls corresponding signal
     * @param cmpt
     * @param msg
     */
    static void onReportWrenchEffortMessageHandler(OjCmpt cmpt, JausMessage msg)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));
        std::unique_ptr<ReportWrenchEffortMessageStruct, decltype(&reportWrenchEffortMessageDestroy)>
            reportWrenchEffort{reportWrenchEffortMessageFromJausMessage(msg),reportWrenchEffortMessageDestroy};

        if(reportWrenchEffort)
        {
            XGVJausClient::WrenchEffortSignalArgs args{};
            args.throttle = reportWrenchEffort->propulsiveLinearEffortXPercent;
            args.brakes = reportWrenchEffort->resistiveLinearEffortXPercent;
            args.steering = reportWrenchEffort->propulsiveRotationalEffortZPercent;

            self->parent_->wrenchEffortReceivedSignal(args);
        }
    }

    /**
     * @brief Handles a Report Component Control message. sets the member variables corresponding to the source
     * @param cmpt
     * @param msg
     */
    static void onReportComponentControlHandler(OjCmpt cmpt, JausMessage msg)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));
        std::unique_ptr<ReportComponentControlMessageStruct, decltype(&reportComponentControlMessageDestroy)>
            reportComponentControl{reportComponentControlMessageFromJausMessage(msg),reportComponentControlMessageDestroy};

        std::unique_ptr<JausAddressStruct, decltype(&jausAddressDestroy)> controller{jausAddressCreate(), jausAddressDestroy};
        controller->subsystem = reportComponentControl->subsystemId;
        controller->node = reportComponentControl->nodeId;
        controller->component = reportComponentControl->componentId;
        controller->instance = reportComponentControl->instanceId;

        //is this the PD component sending it?
        bool we_control = jausAddressEqual(self->cmpt_->getAddress(), controller.get());

        if(jausAddressEqual(reportComponentControl->source, self->primitive_driver_addr_.get()))
        {
            self->control_pd_ = we_control;
        }
        else if(jausAddressEqual(reportComponentControl->source, self->motion_profile_driver_addr_.get()))
        {
            self->control_mp_ = we_control;
        }

    }

    /**
     * @brief Handles a Report Component Status Message. Sets local variables that are used to determine if components are ready
     * @param cmpt
     * @param msg
     */
    static void onReportComponentStatusHandler(OjCmpt cmpt, JausMessage msg)
    {

        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));
        std::unique_ptr<ReportComponentStatusMessageStruct, decltype(&reportComponentStatusMessageDestroy)>
            reportComponentStatus{reportComponentStatusMessageFromJausMessage(msg),reportComponentStatusMessageDestroy};

        if(jausAddressEqual(reportComponentStatus->source, self->primitive_driver_addr_.get()))
        {
            self->pd_ready_ = reportComponentStatus->primaryStatusCode & 0x1;
        }
        else if(jausAddressEqual(reportComponentStatus->source,self->motion_profile_driver_addr_.get()))
        {
            self->mp_ready_ = reportComponentStatus->primaryStatusCode & 0x1;
        }

    }

    /**
     * @brief Handles Report Discrete Devices Message. The field we are most interested in is what Gear the XGV is
     * in which will be used to send wrench efforts
     * @param cmpt
     * @param msg
     */
    static void onReportDiscreteDevicesMessageHandler(OjCmpt cmpt, JausMessage msg)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));

        std::unique_ptr<ReportDiscreteDevicesMessageStruct, decltype(&reportDiscreteDevicesMessageDestroy)>
            reportDiscreteDevices{reportDiscreteDevicesMessageFromJausMessage(msg),reportDiscreteDevicesMessageDestroy};

        if(jausAddressEqual(reportDiscreteDevices->source, self->primitive_driver_addr_.get()))
        {
            if(reportDiscreteDevices->gear == (JausByte)0)
                self->xgv_current_gear_ = Gear::Park;
            else if(reportDiscreteDevices->gear == (JausByte)1)
                self->xgv_current_gear_ = Gear::Low;
            else if(reportDiscreteDevices->gear <= (JausByte)127)
                self->xgv_current_gear_ = Gear::Drive;
            else if(reportDiscreteDevices->gear == (JausByte)128)
                self->xgv_current_gear_ = Gear::Neutral;
            else if(reportDiscreteDevices->gear <= (JausByte)255)
                self->xgv_current_gear_ = Gear::Reverse;
            else
                self->xgv_current_gear_ = Gear::Unknown;
        }

    }

    /**
     * @brief Handles a Report Vehicle Mode Message. Packs the data in an XGVCLient type and calls corresponding signal
     * @param cmpt
     * @param msg
     */
    static void onReportVehicleModeMessageHandler(OjCmpt cmpt, JausMessage msg)
    {
        auto * self = static_cast<XGVJausClientPimpl*>(ojCmptGetUserData(cmpt));

        std::unique_ptr<ReportVehicleModeMessageStruct, decltype(&reportVehicleModeMessageDestroy)>
            reportVehicleMode{reportVehicleModeMessageFromJausMessage(msg),reportVehicleModeMessageDestroy};

        XGVJausClient::VehicleModeEventArgs args{};
        args.controlMode = toControlMode(reportVehicleMode->controlMode);
        args.readyMode = toReadyMode(reportVehicleMode->readyMode);
        args.safetyMode = toSafetyMode(reportVehicleMode->safetyMode);
        args.manualSteering = (reportVehicleMode->partialModeOverrides & 1) != 0;
        args.manualSpeed = (reportVehicleMode->partialModeOverrides & (1 << 1)) != 0;
        args.manualOverride = (reportVehicleMode->safetyDevices & 1) != 0;
        args.safeStopPause = (reportVehicleMode->safetyDevices & (1 << 1)) != 0;
        args.safeStopStop = (reportVehicleMode->safetyDevices & (1 << 2)) != 0;
        args.safeStopLinkStatus =  toLinkStatus(reportVehicleMode->safetyDevices);
        args.externalEstop = (reportVehicleMode->safetyDevices & (1 << 5)) != 0;
        args.doorPause = (reportVehicleMode->safetyDevices & (1 << 6)) != 0;
        args.errorPause = (reportVehicleMode->safetyDevices & (1 << 7)) != 0;
        args.emergencyManualOverride = (reportVehicleMode->safetyDevices & (1 << 8)) != 0;
        args.steeringRequiresInitialization = (reportVehicleMode->safetyDevices & (1 << 9)) != 0;
        args.steeringInitAwaitingUser = (reportVehicleMode->safetyDevices & (1 << 10)) != 0;

        self->parent_->vehicleModeReceivedSignal(args);

    }

    /**
     * @brief Sets up the Jaus Service components for status information needed from the XGV
     */
    void create_svc_connections()
    {
        if(!cmpt_->valid()) return;

        std::unique_ptr<JausAddressStruct, decltype(&jausAddressDestroy)> addr{jausAddressCreate(),jausAddressDestroy};
        
        addr->subsystem = cfg_.subsystem_id;
        addr->node = cfg_.node_id;
        addr->component = JAUS_VELOCITY_STATE_SENSOR;
        addr->instance = 1;
        cmpt_->establishSc(JAUS_REPORT_VELOCITY_STATE, 0x01, addr.get(), 50.0, 3.0, 1);

        addr->component = JAUS_NODE_MANAGER_COMPONENT;
        cmpt_->establishSc(JAUS_REPORT_VEHICLE_MODE, 0xFF, addr.get(), 5.0, 3.0, 1);

        cmpt_->establishSc(JAUS_REPORT_WRENCH_EFFORT, 0x01 | 0x20 | 0x40, primitive_driver_addr_.get(), 50.0, 3.0, 1);
        cmpt_->establishSc(JAUS_REPORT_COMPONENT_CONTROL, 0xFF, primitive_driver_addr_.get(), 1.0, 3.0, 1);
        cmpt_->establishSc(JAUS_REPORT_DISCRETE_DEVICES, 0xFF, primitive_driver_addr_.get(), 10.0, 3.0, 1);
        cmpt_->establishSc(JAUS_REPORT_COMPONENT_STATUS, 0xFF, primitive_driver_addr_.get(), 5.0, 3.0, 1);

        cmpt_->establishSc(JAUS_REPORT_COMPONENT_CONTROL, 0xFF, motion_profile_driver_addr_.get(), 1.0, 3.0, 1);
        cmpt_->establishSc(JAUS_REPORT_COMPONENT_STATUS, 0xFF, motion_profile_driver_addr_.get(), 5.0, 3.0, 1);
        
    }

    /**
     * @brief Registers the message handlers we are interested in
     */
    void register_msg_handlers()
    {
        cmpt_->setMessageCallback(JAUS_REPORT_VELOCITY_STATE, &XGVJausClientPimpl::onReportVelocityStateMessageHandler);
        cmpt_->setMessageCallback(JAUS_REPORT_WRENCH_EFFORT,&XGVJausClientPimpl::onReportWrenchEffortMessageHandler);
        cmpt_->setMessageCallback(JAUS_REPORT_COMPONENT_CONTROL, &XGVJausClientPimpl::onReportComponentControlHandler);
        cmpt_->setMessageCallback(JAUS_REPORT_COMPONENT_STATUS, &XGVJausClientPimpl::onReportComponentStatusHandler);
        cmpt_->setMessageCallback(JAUS_REPORT_DISCRETE_DEVICES, &XGVJausClientPimpl::onReportDiscreteDevicesMessageHandler);
        cmpt_->setMessageCallback(JAUS_REPORT_VEHICLE_MODE, &XGVJausClientPimpl::onReportVehicleModeMessageHandler);
    }

    /**
     * @brief Constructs and sends a SetDiscreteDevices message with the Gear byte set to Drive.
     *
     * This Driver was written to mimic the SRX Behaviour. So we support forward motion in the Drive gear only
     */
    void send_set_drive()
    {
        std::unique_ptr<SetDiscreteDevicesMessageStruct, decltype(&setDiscreteDevicesMessageDestroy)>
            setDiscreteDevices(setDiscreteDevicesMessageCreate(), setDiscreteDevicesMessageDestroy);

        setDiscreteDevices->destination = jausAddressClone(primitive_driver_addr_.get());
        setDiscreteDevices->presenceVector = (JausByte)1 << 2;
        setDiscreteDevices->gear = (JausByte)2;

        std::shared_ptr<JausMessageStruct> msg(setDiscreteDevicesMessageToJausMessage(setDiscreteDevices.get()), jausMessageDestroy);
            cmpt_->sendMessage(msg);
    }

    /**
     * @brief Constructs and sends a Request Control Message to the component identified by Component ID to the XGV
     * subsystem/node set in the config. Uses authority level configured
     * @param component_id
     */
    void send_req_ctrl_msg(JausByte component_id)
    {
        std::unique_ptr<RequestComponentControlMessageStruct, decltype(&requestComponentControlMessageDestroy)>
            reqControlMessage(requestComponentControlMessageCreate(),requestComponentControlMessageDestroy);

      
        reqControlMessage->destination = jausAddressCreate();
        reqControlMessage->destination->subsystem   = cfg_.subsystem_id;
        reqControlMessage->destination->node        = cfg_.node_id;
        reqControlMessage->destination->component   = component_id;
        reqControlMessage->destination->instance    = 1;

        reqControlMessage->authorityCode = cmpt_->getAuthority();

        std::shared_ptr<JausMessageStruct> msg(requestComponentControlMessageToJausMessage(reqControlMessage.get()),jausMessageDestroy);

        cmpt_->sendMessage(msg);
    }

    /**
    * @brief Constructs and sends a Release Control Message to the component identified by Component ID to the XGV
    * subsystem/node set in the config.
     *
    * @param component_id
    */
    void send_rel_ctrl_msg(JausByte component_id)
    {
        std::unique_ptr<ReleaseComponentControlMessageStruct, decltype(&releaseComponentControlMessageDestroy)>
            relControlMessage(releaseComponentControlMessageCreate(),releaseComponentControlMessageDestroy);

        relControlMessage->destination = jausAddressCreate();
        relControlMessage->destination->node        = cfg_.node_id;
        relControlMessage->destination->subsystem   = cfg_.subsystem_id;
        relControlMessage->destination->component   = component_id;
        relControlMessage->destination->instance    = 1;
      
        std::shared_ptr<JausMessageStruct> msg(releaseComponentControlMessageToJausMessage(relControlMessage.get()),
                                        jausMessageDestroy);

        cmpt_->sendMessage(msg);      
    }

    /**
    * @brief Constructs and sends a Resume Messaage to the component identified by Component ID to the XGV
    * subsystem/node set in the config.
     *
    * @param component_id
    */
    void send_resume_msg(JausByte component_id)
    {
        std::unique_ptr<ResumeMessageStruct, decltype(&resumeMessageDestroy)>
            resumeMessage(resumeMessageCreate(), resumeMessageDestroy);

        resumeMessage->destination = jausAddressCreate();
        resumeMessage->destination->subsystem = cfg_.subsystem_id;
        resumeMessage->destination->node = cfg_.node_id;
        resumeMessage->destination->component = component_id;
        resumeMessage->destination->instance = 1;

        std::shared_ptr<JausMessageStruct> msg(resumeMessageToJausMessage(resumeMessage.get()), jausMessageDestroy);

        cmpt_->sendMessage(msg);
    }

    /**
    * @brief Constructs and sends a Set Motion Profile Message to the Motion Profile Driver component on the xgv.
    *
    * Uses the current set_speed_ and max_accel_ values
    */
    void send_set_motion_profile_message()
    {
        std::unique_ptr<SetMotionProfileMessageStruct, decltype(&setMotionProfileMessageDestroy)>
            setMotionProfileMessage(setMotionProfileMessageCreate(),setMotionProfileMessageDestroy);

        setMotionProfileMessage->destination = jausAddressCreate();
        setMotionProfileMessage->destination->node = cfg_.node_id;
        setMotionProfileMessage->destination->subsystem = cfg_.subsystem_id;
        setMotionProfileMessage->destination->component = JAUS_MOTION_PROFILE_DRIVER;
        setMotionProfileMessage->destination->instance = 1;

        setMotionProfileMessage->version = 1;
        setMotionProfileMessage->numberOfMotions = 1;
        {
            std::lock_guard<std::mutex> lock(speed_ctrl_mutex_);
            setMotionProfileMessage->desiredVelocity = set_speed_;
            setMotionProfileMessage->maximumAcceleration = max_accel_;

        }

        setMotionProfileMessage->atanOfDesiredCurvature = 0.0;
        setMotionProfileMessage->rateOfChangeOfCurvature = 0.0;
        setMotionProfileMessage->timeDuration = cfg_.motion_profile_duration;

        std::shared_ptr<JausMessageStruct> msg(setMotionProfileMessageToJausMessage(setMotionProfileMessage.get()),jausMessageDestroy);

        cmpt_->sendMessage(msg);
    }

    /**
    * @brief Constructs and sends a Set Wrench Effort Message to the component to the Primitive Driver Component
    * on the XGV. Uses the current wrench_effort value
    */
    void send_set_wrench_effort_message()
    {
        //todo: finish this message
        std::unique_ptr<SetWrenchEffortMessageStruct, decltype(&setWrenchEffortMessageDestroy)>
            setWrenchEffortMessage(setWrenchEffortMessageCreate(),setWrenchEffortMessageDestroy);

        setWrenchEffortMessage->destination = jausAddressCreate();
        setWrenchEffortMessage->destination->node = cfg_.node_id;
        setWrenchEffortMessage->destination->subsystem = cfg_.subsystem_id;
        setWrenchEffortMessage->destination->component = JAUS_PRIMITIVE_DRIVER;
        setWrenchEffortMessage->destination->instance = 1;

        setWrenchEffortMessage->presenceVector = 0b1;
        setWrenchEffortMessage->propulsiveLinearEffortXPercent = wrench_effort_;


        std::shared_ptr<JausMessageStruct> msg(setWrenchEffortMessageToJausMessage(setWrenchEffortMessage.get()),jausMessageDestroy);

        cmpt_->sendMessage(msg);
    }


};

//Public Interface

XGVJausClient::XGVJausClient(XGVJausClient::XGVJausClientConfig cfg)
{
    pimpl_.reset(new XGVJausClientPimpl(cfg,this));
}

XGVJausClient::~XGVJausClient()
{
    shutdown();
}

void XGVJausClient::setSpeedAccel(double speed, double max_accel)
{
    pimpl_->setSpeedAccel(speed,max_accel);
}
void XGVJausClient::setWrenchEffort(double effort)
{
    pimpl_->setWrenchEffort(effort);
}
void XGVJausClient::disableRoboticControl()
{
    pimpl_->disableRoboticControl();
}
void XGVJausClient::shutdown()
{
    pimpl_->shutdown();
}
bool XGVJausClient::connect()
{
    return pimpl_->connect();
}