/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Torc Robotics, LLC
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

#include "truck_application.h"

namespace carma
{
    const double max_commanded_speed = 31.25; // (m/s)
    const double max_commanded_accel = 3.5; // (m/s)/s
}

TruckApplication::TruckApplication(int argc, char **argv, const std::string &name) : DriverApplication(argc, argv, name)
{

}

TruckApplication::~TruckApplication() 
{
    truck_dbw_ctrl_.reset();
}

void TruckApplication::initialize() 
{
    control_message_nh_.reset(new ros::NodeHandle("~control"));
    updater_.reset(new diagnostic_updater::Updater());
    updater_->setHardwareID("Truck DBW");

    pnh_->param<bool>("enabled_at_start", robotic_enabled_, false); // Set (enable_robotic_ = false) as default
    pnh_->param<bool>("clear_faults_at_start", clear_faults_enabled_, false); // Set (clear_faults_enabled_ = false) as default

    // Initialize and Setup Pid Param
    pnh_->param<double>("k_p", k_p_, 7.5);
    pnh_->param<double>("k_i", k_i_, 3.0);
    pnh_->param<double>("k_d", k_d_, 0.0);

    set_PidParam_.setF(k_p_,k_i_,k_d_);

    set_LightControl_.Flashing     = TruckDBWController::GenericStatusEnum::On;
    set_LightControl_.LeftBlinker  = TruckDBWController::GenericStatusEnum::On;
    set_LightControl_.RightBlinker = TruckDBWController::GenericStatusEnum::On;
    set_LightControl_.TakeDown     = TruckDBWController::GenericStatusEnum::On;
    set_LightControl_.GreenFlash   = TruckDBWController::GenericStatusEnum::On;
    set_LightControl_.GreenSolid   = TruckDBWController::GenericStatusEnum::On;

    // Initialize Send Timers
    auto time_now = ros::Time::now();
    last_sent_control_mode_message_     = time_now ;
    last_sent_engine_braking_message_   = time_now ;
    last_sent_pid_gains_message_        = time_now ;
    last_sent_led_status_message_       = time_now ;
    last_sent_front_lights_msg_         = time_now ;
    last_sent_rear_lights_msg_          = time_now ;

    // Initialize robotic control info stream print flag
    disable_robotic_printed_flag_ = false;
    robotic_wrench_control_active_ = false;
    robotic_speed_control_active_ = false;

    //Initialize the dbw controller
    initializeDBWController();

    //DBW Module feedback status handlers
    setupRecvItems();

    // Connect and bind recv signals to update functions of dbw controller
    registerEventHandlers();

    //Setup the pub/subs
    setupSubscribersPublishers();

    updater_->add(*recv_RoboticMode_PedalPosition_);
    updater_->add(*recv_ControlMessageEcho_);
    updater_->add(*recv_PrimaryBrakeAxiomatic_);
    updater_->add(*recv_SecondaryBrakAxiomatic_);
    updater_->add(*recv_EngineBrakeControlFb_);
    updater_->add(*recv_SpeedControlPidParams_);
    updater_->add(*recv_SpeedControlPidEcho_);
    updater_->add(*recv_GeneralEcuStatus_);
    updater_->add(*recv_RawInputChannel_);
    updater_->add(*recv_RawOutputChannel_);
    updater_->add(*recv_ManualConditionChecks_);
    updater_->add(*recv_BrakeFaultConditionChecks_);
    updater_->add(*recv_EmergencyFaultConditionChecks_);
    updater_->add(*recv_LedStatusEcho_);
    updater_->add(*recv_SettingsCrc_);

    // Create a new driver status and setup to default values
    cav_msgs::DriverStatus status;
    status.lon_controller = static_cast<unsigned char>(true);
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);

    // Set SpinRate
    spin_rate = 50;
}

void TruckApplication::pre_spin()
{
    // Update last sent messages with previously set messages
    last_EngineBrakeCommand_ = set_EngineBrakeCommand_;
    last_PidParam_ = set_PidParam_;
    last_LedStatus_ = set_LedStatus_;
    last_FrontLightStatus_ = set_LightControl_;
    last_RearLightStatus_ = set_LightControl_;
    updater_->update();
}

void TruckApplication::post_spin()
{
    sendCommands();
    checkLastReceivedMsgs();
}

void TruckApplication::sendCommands()
{
    auto time_now = ros::Time::now();
    if((time_now - last_sent_control_mode_message_) > ros::Duration(truck_dbw_send_rates_.send_control_mode_message))
    {
        // Initialize control to false - willl overwrite if active
        robotic_wrench_control_active_ = false;
        robotic_speed_control_active_ = false;

        if (!robotic_enabled_)
        {
            // Reset All Other values since we are disabling ACC
            set_ContolMessage_.wrench_effort = 0.0;
            set_ContolMessage_.speed_control = 0.0;
            set_ContolMessage_.max_accel = 0.0;
            set_ContolMessage_.mode = TruckDBWController::CommandMode::DisableACCSystem;

            truck_dbw_ctrl_->sendDisableRoboticControl();
            disable_robotic_printed_flag_ = false;
        }
        else if((time_now - control_message_recv_time) > ros::Duration(0.3) && set_ContolMessage_.mode != TruckDBWController::CommandMode::DisableACCSystem)
        {
            ROS_WARN_STREAM("Command message timeout on both control topics - Disabling Robotic");
            set_ContolMessage_.mode = TruckDBWController::CommandMode::DisableACCSystem;
            truck_dbw_ctrl_->sendDisableRoboticControl();
            disable_robotic_printed_flag_ = false;
        }
        else if (set_ContolMessage_.mode == TruckDBWController::CommandMode::RoboticWrenchEffortControl)
        {
            ROS_INFO_STREAM("Sending wrench " << set_ContolMessage_.wrench_effort);
            truck_dbw_ctrl_->sendWrenchEffortCommand(set_ContolMessage_.wrench_effort);
            disable_robotic_printed_flag_ = false;

            robotic_wrench_control_active_ = true;
        }
        else if (set_ContolMessage_.mode == TruckDBWController::CommandMode::RoboticSpeedControl)
        {
            ROS_INFO_STREAM("Sending speed " << set_ContolMessage_.speed_control << " " << set_ContolMessage_.max_accel);
            truck_dbw_ctrl_->sendSpeedAccelCommand(set_ContolMessage_.speed_control, set_ContolMessage_.max_accel);
            disable_robotic_printed_flag_ = false;

            robotic_speed_control_active_ = true;
        }
        else
        {
            if(!disable_robotic_printed_flag_)
            {
                ROS_INFO_STREAM("No control command - Disabling Robotic");
                disable_robotic_printed_flag_ = true;
            }
            truck_dbw_ctrl_->sendDisableRoboticControl();
        }

        last_sent_control_mode_message_ = time_now;
    }

    if((time_now - last_sent_engine_braking_message_) > ros::Duration(truck_dbw_send_rates_.send_engine_brake_message))
    {
        truck_dbw_ctrl_->sendEngineBrakeCommand(set_EngineBrakeCommand_);
        last_sent_engine_braking_message_ = time_now;
    }

    if((time_now - last_sent_pid_gains_message_) > ros::Duration(truck_dbw_send_rates_.send_pid_gains_message))
    {
        truck_dbw_ctrl_->sendPidParam(set_PidParam_);
        last_sent_pid_gains_message_ = time_now;
    }

    if((time_now - last_sent_led_status_message_) > ros::Duration(truck_dbw_send_rates_.send_led_status_message))
    {
        truck_dbw_ctrl_->sendLEDStatus(set_LedStatus_);
        last_sent_led_status_message_ = time_now;
    }

    if((time_now - last_sent_front_lights_msg_) > ros::Duration(truck_dbw_send_rates_.send_front_light_status_message))
    {

        truck_dbw_ctrl_->sendLightStatus(TruckDBWController::LightId::Front, set_LightControl_);
        last_sent_front_lights_msg_ = time_now;
    }

    if((time_now - last_sent_rear_lights_msg_) > ros::Duration(truck_dbw_send_rates_.send_rear_light_status_message))
    {
        truck_dbw_ctrl_->sendLightStatus(TruckDBWController::LightId::Rear, set_LightControl_);
        last_sent_rear_lights_msg_ = time_now;
    }

    if(clear_faults_enabled_)
    {
        truck_dbw_ctrl_->sendClearFaults(); // Clear Faults
        clear_faults_enabled_ = false;  // Reset Clearing (only send once)
    }
}

/** ---------- Update functions for recv signals ---------- **/
void TruckApplication::updateRoboticModePedalPercent(TruckDBWController::RoboticMode_PedalPosition signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_RoboticMode_PedalPosition_->setData(signal_data);
}

void TruckApplication::updateControlMessageEcho(TruckDBWController::ControlMessageEcho signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_ControlMessageEcho_->setData(signal_data);
}

void TruckApplication::updatePrimaryBrakeAxiomatic(TruckDBWController::PrimaryBrakeAxiomatic signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_PrimaryBrakeAxiomatic_->setData(signal_data);
}

void TruckApplication::updateSecondaryBrakeAxiomatic(TruckDBWController::SecondaryBrakeAxiomatic signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
    recv_SecondaryBrakAxiomatic_ ->setData(signal_data);
}

void TruckApplication::updateEngineBrakeControlFb(TruckDBWController::EngineBrakeControl signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_EngineBrakeControlFb_->setData(signal_data);
}

void TruckApplication::updateSpeedControllerPidError(TruckDBWController::SpeedControllerPIDParams signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_SpeedControlPidParams_->setData(signal_data);
}

void TruckApplication::updateSpeedControllerPidEcho(TruckDBWController::SpeedControllerPIDEcho signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_SpeedControlPidEcho_->setData(signal_data);
}

void TruckApplication::updateGeneralEcuStatus(TruckDBWController::GeneralECUStatus signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_GeneralEcuStatus_->setData(signal_data);
}

void TruckApplication::updateRawInputChannel(TruckDBWController::RawInputChannel signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_RawInputChannel_->setData(signal_data);
}

void TruckApplication::updateRawOutputChannel(TruckDBWController::RawOutputChannel signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_RawOutputChannel_->setData(signal_data);
}

void TruckApplication::updateManualConditionChecks(TruckDBWController::ManualConditionChecks signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_ManualConditionChecks_->setData(signal_data);
}

void TruckApplication::updateBrakeFaultConditionChecks(TruckDBWController::BrakeFaultConditionChecks signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_BrakeFaultConditionChecks_->setData(signal_data);
}

void TruckApplication::updateEmergencyFaultConditionChecks(TruckDBWController::EmergencyFaultConditionChecks signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_EmergencyFaultConditionChecks_->setData(signal_data);
}

void TruckApplication::updateLedStatusEcho(TruckDBWController::LEDStatusEcho signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_LedStatusEcho_->setData(signal_data);
}

void TruckApplication::updateSettingsCrc(TruckDBWController::SettingsCrc signal_data)
{
	std::unique_lock<std::recursive_mutex> lock(status_mutex_);
	recv_SettingsCrc_->setData(signal_data);
}


void TruckApplication::checkLastReceivedMsgs()
{

    // Create an instance of all messages in Truck Controller
    TruckDBWController::ControlMessage controlMessage_temp{};
    TruckDBWController::EngineBrakeCommand engineBrakeCommand_temp{};
    TruckDBWController::PidParams pidParam_temp{};
    TruckDBWController::LEDStatus ledStatus_temp{};
    TruckDBWController::LightStatus frontLightStatus_temp{};
    TruckDBWController::LightStatus rearLightStatus_temp{};

    // Copy over local data into temp message structures
    {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        controlMessage_temp     = last_ControlMessage_;
        engineBrakeCommand_temp = last_EngineBrakeCommand_;
        pidParam_temp           = last_PidParam_;
        ledStatus_temp          = last_LedStatus_;
        frontLightStatus_temp   = last_FrontLightStatus_;
        rearLightStatus_temp    = last_RearLightStatus_;
    }
}

/** ---------- ROS Message Handling ---------- **/
bool TruckApplication::enableRobotic(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &res)
{
    bool tmp = robotic_enabled_;
    robotic_enabled_ = (req.set == cav_srvs::SetEnableRoboticRequest::ENABLE);

    if(tmp != robotic_enabled_)
    {
        ROS_INFO_STREAM("Robotic enabled set to: " << robotic_enabled_);
    }
    return true;
}

void TruckApplication::updateControlStatus(const ros::WallTimerEvent &) 
{
    cav_msgs::RobotEnabled msg;
    msg.robot_active = static_cast<cav_msgs::RobotEnabled::_robot_enabled_type>(robotic_wrench_control_active_ || robotic_speed_control_active_);
    msg.robot_enabled   = robotic_enabled_;

    // Not populating any of the fields below
    msg.torque = 0;
    msg.torque_validity = false;

    msg.brake_decel = 0;
    msg.brake_decel_validity = false;

    msg.throttle_effort = 0;
    msg.throttle_effort_validity = false;

    msg.braking_effort = 0;
    msg.braking_effort_validity = false;

    robotic_status_pub_.publish(msg);
}

/**
 * @brief Helper function to convert the ros message type EngineBrakeMode to the equivalent TruckDBWController::EngineBrakeMsgModeEnum
 * type
 * @param mode
 * @return
 */
TruckDBWController::EngineBrakeMsgModeEnum toEngineBrakeMsg(cav_msgs::EngineBrakeMode::_mode_type mode)
{
    switch(mode)
    {
        case cav_msgs::EngineBrakeMode::PASS_THRU :
            return TruckDBWController::EngineBrakeMsgModeEnum::PassThrough;
        case cav_msgs::EngineBrakeMode::BYPASS :
            return TruckDBWController::EngineBrakeMsgModeEnum::TorcBypass;
        case cav_msgs::EngineBrakeMode::NO_MODE_CHANGE:
            return TruckDBWController::EngineBrakeMsgModeEnum::NoChange;
        case cav_msgs::EngineBrakeMode::MODE_ERROR :
        default:
            return TruckDBWController::EngineBrakeMsgModeEnum::Error;
    }
}


/**
 * @brief Helper function to convert the ros message type EngineBrakeCommand to the equivalent TruckDBWController::EngineBrakeCommandEnum
 * type
 * @param mode
 * @return
 */
TruckDBWController::EngineBrakeCommandEnum toEngineBrakeCommand(cav_msgs::EngineBrakeCommand::_command_type cmd)
{
    switch(cmd)
    {
        case cav_msgs::EngineBrakeCommand::DISABLE:
            return TruckDBWController::EngineBrakeCommandEnum::DisableEngineBraking;
        case cav_msgs::EngineBrakeCommand::ENABLE:
            return TruckDBWController::EngineBrakeCommandEnum::EnableEngineBraking;
        case cav_msgs::EngineBrakeCommand::NO_CMD_CHANGE:
            return TruckDBWController::EngineBrakeCommandEnum::NoChange;
        case cav_msgs::EngineBrakeCommand::CMD_ERROR:
        default:
            return TruckDBWController::EngineBrakeCommandEnum::Error;
    }
}


/**
 * @brief Helper function to convert the ros message type EngineBrakeLevelCommandEnum to the equivalent TruckDBWController::EngineBrakeLevelCommandEnum
 * type
 * @param mode
 * @return
 */
TruckDBWController::EngineBrakeLevelCommandEnum toEngineBrakeLevelCommand(cav_msgs::EngineBrakeLevel::_level_type lvl)
{
    switch(lvl)
    {
        case cav_msgs::EngineBrakeLevel::LOW_LEVEL:
            return TruckDBWController::EngineBrakeLevelCommandEnum::Low;
        case cav_msgs::EngineBrakeLevel::MEDIUM_LEVEL:
            return TruckDBWController::EngineBrakeLevelCommandEnum::Medium;
        case cav_msgs::EngineBrakeLevel::HIGH_LEVEL:
            return TruckDBWController::EngineBrakeLevelCommandEnum::High;
        case cav_msgs::EngineBrakeLevel::NO_LEVEL_CHANGE:
        default:
            return TruckDBWController::EngineBrakeLevelCommandEnum::NoChange;
    }
}


bool TruckApplication::engineBrakeControl(cav_srvs::EngineBrakeControlRequest &req, cav_srvs::EngineBrakeControlResponse &res)
{
    set_EngineBrakeCommand_.mode = toEngineBrakeMsg(req.set_engine_brake_mode.mode);
    set_EngineBrakeCommand_.command = toEngineBrakeCommand(req.set_engine_brake_cmd.command);
    set_EngineBrakeCommand_.level = toEngineBrakeLevelCommand(req.set_engine_brake_level.level);

    return true;
}

bool TruckApplication::getLightStatus(cav_srvs::GetLightsRequest &req, cav_srvs::GetLightsResponse &res)
{
    static auto ON = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::ON);
    static auto OFF = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::OFF);

    res.status.flash        = (set_LightControl_.Flashing     == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;
    res.status.left_arrow   = (set_LightControl_.LeftBlinker  == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;
    res.status.right_arrow  = (set_LightControl_.RightBlinker == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;
    res.status.takedown     = (set_LightControl_.TakeDown     == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;
    res.status.green_flash  = (set_LightControl_.GreenFlash   == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;
    res.status.green_solid  = (set_LightControl_.GreenSolid   == TruckDBWController::GenericStatusEnum::On)   ? ON : OFF;

    return true;
}

bool TruckApplication::setLightStatus(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse &res)
{
    using cav_msgs::LightBarStatus;
    static auto ON = static_cast<TruckDBWController::GenericStatusEnum>(TruckDBWController::GenericStatusEnum::On);
    static auto OFF = static_cast<TruckDBWController::GenericStatusEnum>(TruckDBWController::GenericStatusEnum::Off);

    set_LightControl_.Flashing      = (req.set_state.flash       == LightBarStatus::ON) ? ON : OFF;
    set_LightControl_.LeftBlinker   = (req.set_state.left_arrow  == LightBarStatus::ON) ? ON : OFF;
    set_LightControl_.RightBlinker  = (req.set_state.right_arrow == LightBarStatus::ON) ? ON : OFF;
    set_LightControl_.TakeDown      = (req.set_state.takedown    == LightBarStatus::ON) ? ON : OFF;
    set_LightControl_.GreenFlash    = (req.set_state.green_flash == LightBarStatus::ON) ? ON : OFF;
    set_LightControl_.GreenSolid    = (req.set_state.green_solid == LightBarStatus::ON) ? ON : OFF;

    return true;
}

bool TruckApplication::clearFaults(cav_srvs::ClearFaultsRequest &req, cav_srvs::ClearFaultsResponse &res)
{
    bool tmp = clear_faults_enabled_;
    clear_faults_enabled_ = (req.clear_faults == cav_srvs::ClearFaultsRequest::ENABLE);

    if(tmp != clear_faults_enabled_)
    {
        bool on_off = (clear_faults_enabled_ ? "true" : "false");
        ROS_WARN_STREAM("Clearing Faults: " << on_off);
    }

    return true;
}

void TruckApplication::shutdown()
{
    ROS_WARN_STREAM("Shutting down");
    truck_dbw_ctrl_.reset();
}

void TruckApplication::registerEventHandlers()
{
    if(!truck_dbw_ctrl_) return;
    truck_dbw_ctrl_->onRoboticMode_PedalPercentageRecv.connect(std::bind(&TruckApplication::updateRoboticModePedalPercent, this, std::placeholders::_1));
    truck_dbw_ctrl_->onControlMessageEchoRecv.connect(std::bind(&TruckApplication::updateControlMessageEcho, this, std::placeholders::_1));
    truck_dbw_ctrl_->onPrimaryBrakeAxiomaticRecv.connect(std::bind(&TruckApplication::updatePrimaryBrakeAxiomatic, this, std::placeholders::_1));
    truck_dbw_ctrl_->onSecondaryBrakeAxiomaticRecv.connect(std::bind(&TruckApplication::updateSecondaryBrakeAxiomatic, this, std::placeholders::_1));
    truck_dbw_ctrl_->onEngineBrakeControlFeedbackRecv.connect(std::bind(&TruckApplication::updateEngineBrakeControlFb, this, std::placeholders::_1));
    truck_dbw_ctrl_->onSpeedControllerPidRecv.connect(std::bind(&TruckApplication::updateSpeedControllerPidError, this, std::placeholders::_1));
    truck_dbw_ctrl_->onSpeedConrollerPidEchoRecv.connect(std::bind(&TruckApplication::updateSpeedControllerPidEcho, this, std::placeholders::_1));
    truck_dbw_ctrl_->onGeneralEcuStatusRecv.connect(std::bind(&TruckApplication::updateGeneralEcuStatus, this, std::placeholders::_1));
    truck_dbw_ctrl_->onRawInputChannelRecv.connect(std::bind(&TruckApplication::updateRawInputChannel, this, std::placeholders::_1));
    truck_dbw_ctrl_->onRawOutputChannelRecv.connect(std::bind(&TruckApplication::updateRawOutputChannel, this, std::placeholders::_1));
    truck_dbw_ctrl_->onManualConditionChecksRecv.connect(std::bind(&TruckApplication::updateManualConditionChecks, this, std::placeholders::_1));
    truck_dbw_ctrl_->onBrakeFaultChecksRecv.connect(std::bind(&TruckApplication::updateBrakeFaultConditionChecks, this, std::placeholders::_1));
    truck_dbw_ctrl_->onEmergencyFaultChecksRecv.connect(std::bind(&TruckApplication::updateEmergencyFaultConditionChecks, this, std::placeholders::_1));
    truck_dbw_ctrl_->onLedStatusEchoRecv.connect(std::bind(&TruckApplication::updateLedStatusEcho, this, std::placeholders::_1));
    truck_dbw_ctrl_->onSettingsCrcRecv.connect(std::bind(&TruckApplication::updateSettingsCrc, this, std::placeholders::_1));
}


void TruckApplication::initializeDBWController() {

    int can_select;
    pnh_->param<int32_t>("socketcan_selection", can_select, 0);
    truck_cfg_.socketcan_selection = static_cast<truck::SocketCanSelect>(can_select);

    // Use CAV SocketCAN or ROS SocketCAN depending on configuration
    std::unique_ptr<cav::CANInterface> can_device;
    switch(truck_cfg_.socketcan_selection)
    {
        case truck::SocketCanSelect::CavSocketCan:
        {
            // Get 'Device Name' from config settings
            pnh_->param<std::string>("socketcan_device", truck_cfg_.socketcan_device,"dbw_ifc");
            can_device.reset(new cav::SocketCANInterface(truck_cfg_.socketcan_device));
        }
            break;
        case truck::SocketCanSelect::RosSocketCan:
        {
            pnh_->param<std::string>("ros_socketcan_topic_recv",truck_cfg_.ros_socketcan_topic_recv,"received_messages");
            pnh_->param<std::string>("ros_socketcan_topic_send",truck_cfg_.ros_socketcan_topic_send,"sent_messages");
            can_device.reset(new cav::ROSSocketCANBridge(truck_cfg_.ros_socketcan_topic_recv,
                                                         truck_cfg_.ros_socketcan_topic_send));
        }
            break;
    }

    if(can_device)
    {
        truck_dbw_ctrl_.reset(new TruckDBWController(std::move(can_device)));
        truck_dbw_ctrl_->initialize();
    }
    else
    {
        ROS_WARN("No CAN Device configured");
    }
}


void TruckApplication::setupSubscribersPublishers()
{
    // Subscribe to Incoming messages
    wrench_effort_subscriber_ = control_message_nh_->subscribe<std_msgs::Float32>("cmd_wrench_effort", 1,
                                                                                  [this](const std_msgs::Float32ConstPtr &msg)
                                                                                  {
                                                                                    control_message_recv_time = ros::Time::now();
                                                                                    set_ContolMessage_.wrench_effort = msg->data;
                                                                                    set_ContolMessage_.mode = TruckDBWController::CommandMode::RoboticWrenchEffortControl;
                                                                                  });
    api_.push_back(wrench_effort_subscriber_.getTopic());

    speed_accel_subscriber_ = control_message_nh_->subscribe<cav_msgs::SpeedAccel>("cmd_speed_accel", 1,
                                                                                   [this](const cav_msgs::SpeedAccelConstPtr &msg)
                                                                                   {
                                                                                     if(msg->speed >= 0 && msg->max_accel >= 0)
                                                                                     {
                                                                                         double recv_speed = msg->speed;
                                                                                         double recv_accel = msg->max_accel;

                                                                                         if(recv_speed > carma::max_commanded_speed)
                                                                                         {
                                                                                             recv_speed  = carma::max_commanded_speed;
                                                                                         }
                                                                                         if(recv_accel > carma::max_commanded_accel)
                                                                                         {
                                                                                             recv_accel = carma::max_commanded_accel;
                                                                                         }

                                                                                         set_ContolMessage_.speed_control =  recv_speed;
                                                                                         set_ContolMessage_.max_accel = recv_accel;
                                                                                         set_ContolMessage_.mode = TruckDBWController::CommandMode::RoboticSpeedControl;
                                                                                         control_message_recv_time = ros::Time::now();

                                                                                     }
                                                                                   });
    api_.push_back(speed_accel_subscriber_.getTopic());

    // Advertise Publisher
    robotic_status_pub_ = control_message_nh_->advertise<cav_msgs::RobotEnabled>("robotic_status", 1);
    api_.push_back(robotic_status_pub_.getTopic());

    // Advertise Services
    enable_robotic_service_ = control_message_nh_->advertiseService("enable_robotic", &TruckApplication::enableRobotic, this);
    api_.push_back(enable_robotic_service_.getService());

    engine_brake_service_ = control_message_nh_->advertiseService("engine_brake_ctrl", &TruckApplication::engineBrakeControl, this);
    api_.push_back(engine_brake_service_.getService());

    get_lights_service_ = control_message_nh_->advertiseService("get_lights", &TruckApplication::getLightStatus, this);
    api_.push_back(get_lights_service_.getService());

    set_lights_service_ = control_message_nh_->advertiseService("set_lights", &TruckApplication::setLightStatus, this);
    api_.push_back(set_lights_service_.getService());

    clear_faults_service_ = control_message_nh_->advertiseService("clear_faults", &TruckApplication::clearFaults, this);
    api_.push_back(clear_faults_service_.getService());

    // Creat Publisher Timer
    status_publisher_timer_ = nh_->createWallTimer(ros::WallDuration(ros::Rate(2)),&TruckApplication::updateControlStatus, this);
    status_publisher_timer_.start();

}
void TruckApplication::setupRecvItems()
{
    using tc = TruckDBWController;

    //todo should we change the summary to Error if any of these fields are certain values

    recv_RoboticMode_PedalPosition_.reset(new StatusUpdater<tc::RoboticMode_PedalPosition>("Robotic Mode Position",
                                                                                ros::Duration(truck_ros_recv_rates_.recv_roboticmode_pedalposition),
                                                                                [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                                {
                                                                                    stat.add("Expected Accel Pedal Percent",recv_RoboticMode_PedalPosition_->getData().exp_accel_pedal_percent);
                                                                                    stat.add("Generated Accel Pedal Percent",recv_RoboticMode_PedalPosition_->getData().gen_accel_pedal_percent);
                                                                                    stat.add("Physical Accel Pedal Percent",recv_RoboticMode_PedalPosition_->getData().phy_accel_pedal_percent);
                                                                                    stat.add("Robotic Mode", static_cast<int>(recv_RoboticMode_PedalPosition_->getData().status));
                                                                                }));

    recv_ControlMessageEcho_.reset(new StatusUpdater<tc::ControlMessageEcho>("Control Message Echo",
                                                                               ros::Duration(truck_ros_recv_rates_.recv_control_message_echo),
                                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                               {
                                                                                   stat.add("Wrench Effort",recv_ControlMessageEcho_->getData().wrench_effort);
                                                                                   stat.add("Speed Control",recv_ControlMessageEcho_->getData().speed_ctrl);
                                                                                   stat.add("Max Accel",recv_ControlMessageEcho_->getData().max_accel);
                                                                                   stat.add("Mode", static_cast<int>(recv_ControlMessageEcho_->getData().mode));
                                                                               }));

    recv_PrimaryBrakeAxiomatic_.reset(new StatusUpdater<tc::PrimaryBrakeAxiomatic>("Primary Brake Axiomatic Module",
                                                                               ros::Duration(truck_ros_recv_rates_.recv_prim_brake_axiomatic),
                                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                               {
                                                                                   stat.add("System Air Pressure", recv_PrimaryBrakeAxiomatic_->getData().sys_air_pres);
                                                                                   stat.add("Robotic Truck Applied Brake Pressure", recv_PrimaryBrakeAxiomatic_->getData().robotic_truck_brake_pres);
                                                                                   stat.add("Robotic Trailer Applied Brake Pressure", recv_PrimaryBrakeAxiomatic_->getData().robotic_trailer_brake_pres);
                                                                                   stat.add("Command Brake Application Level", recv_PrimaryBrakeAxiomatic_->getData().cmd_brake_appy_level);
                                                                                   stat.add("Health Status", static_cast<int>(recv_PrimaryBrakeAxiomatic_->getData().status));
                                                                               }));

    recv_SecondaryBrakAxiomatic_.reset(new StatusUpdater<tc::SecondaryBrakeAxiomatic>("Secondary Brake Axiomatic Module",
                                                                              ros::Duration(truck_ros_recv_rates_.recv_sec_brake_axiomatic),
                                                                              [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                              {
                                                                                  stat.add("System Air Pressure", recv_SecondaryBrakAxiomatic_->getData().sys_air_pres);
                                                                                  stat.add("Robotic Truck Applied Brake Pressure", recv_SecondaryBrakAxiomatic_->getData().robotic_truck_brake_pres);
                                                                                  stat.add("Robotic Trailer Applied Brake Pressure", recv_SecondaryBrakAxiomatic_->getData().robotic_trailer_brake_pres);
                                                                                  stat.add("Command Brake Application Level", recv_SecondaryBrakAxiomatic_->getData().cmd_brake_appy_level);
                                                                                  stat.add("Health Status", static_cast<int>(recv_SecondaryBrakAxiomatic_->getData().status));
                                                                              }));

    recv_EngineBrakeControlFb_.reset(new StatusUpdater<tc::EngineBrakeControl>("Engine Brake Control Feedback",
                                                                               ros::Duration(truck_ros_recv_rates_.recv_engine_brake_control),
                                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                               {
                                                                                   stat.add("Engine Brake Message Mode", static_cast<int>(recv_EngineBrakeControlFb_->getData().eng_brake_msg_mode));
                                                                                   stat.add("Generated Engine Brake Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().gen_eng_brake_cmd));
                                                                                   stat.add("Generated Engine Brake Level Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().gen_eng_brake_level));

                                                                                   stat.add("Expected Engine Brake Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().exp_eng_brake_cmd));
                                                                                   stat.add("Expected Engine Brake Level Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().exp_eng_brake_level));

                                                                                   stat.add("Physical Engine Brake Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().phy_eng_brake_cmd));
                                                                                   stat.add("Physical Engine Brake Level Command", static_cast<int>(recv_EngineBrakeControlFb_->getData().phy_eng_brake_level));

                                                                               }));


    recv_SpeedControlPidParams_.reset(new StatusUpdater<tc::SpeedControllerPIDParams>("Speed Control PID Error",
                                                                      ros::Duration(truck_ros_recv_rates_.recv_speed_control_pid_params),
                                                                      [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                      {
                                                                          stat.add("Proportinal Error", recv_SpeedControlPidParams_->getData().p);
                                                                          stat.add("Integral Error", recv_SpeedControlPidParams_->getData().i);
                                                                          stat.add("Derivative Error", recv_SpeedControlPidParams_->getData().d);
                                                                          stat.add("Divisor", recv_SpeedControlPidParams_->getData().divisor);
                                                                      }));



    recv_SpeedControlPidEcho_.reset(new StatusUpdater<tc::SpeedControllerPIDEcho>("Speed Control PID Params Echo",
                                                                      ros::Duration(truck_ros_recv_rates_.recv_speed_control_pid_echo),
                                                                      [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                      {
                                                                          stat.add("Proportional Gain", recv_SpeedControlPidEcho_->getData().p);
                                                                          stat.add("Integral Gain", recv_SpeedControlPidEcho_->getData().i);
                                                                          stat.add("Derivative Gain", recv_SpeedControlPidEcho_->getData().d);
                                                                          stat.add("Divisor", recv_SpeedControlPidEcho_->getData().divisor);
                                                                      }));

    recv_GeneralEcuStatus_.reset(new StatusUpdater<tc::GeneralECUStatus>("General ECU Status",
                                                                     ros::Duration(truck_ros_recv_rates_.recv_general_ecu_status),
                                                                     [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                     {
                                                                         stat.add("Module Temperature", recv_GeneralEcuStatus_->getData().module_temp);
                                                                         stat.add("Module Input Voltage", recv_GeneralEcuStatus_->getData().module_in_voltage);
                                                                         stat.add("Module Max Loop Time", recv_GeneralEcuStatus_->getData().module_max_loop_time);
                                                                         stat.add("Module Avg Loop Time", recv_GeneralEcuStatus_->getData().module_avg_loop_time);

                                                                     }));


    recv_RawInputChannel_.reset(new StatusUpdater<tc::RawInputChannel>("Raw Input Channels",
                                                               ros::Duration(truck_ros_recv_rates_.recv_raw_input_channel),
                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                               {
                                                                   stat.add("Analog In 1", recv_RawInputChannel_->getData().ain_1);
                                                                   stat.add("Analog In 2", recv_RawInputChannel_->getData().ain_2);
                                                                   stat.add("Analog In 3", recv_RawInputChannel_->getData().ain_3);
                                                                   stat.add("Analog In 4", recv_RawInputChannel_->getData().ain_4);
                                                               }));


    recv_RawOutputChannel_.reset(new StatusUpdater<tc::RawOutputChannel>("Raw Output Channels",
                                                                   ros::Duration(truck_ros_recv_rates_.recv_raw_input_channel),
                                                                   [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                   {
                                                                       stat.add("Analog Out 1", recv_RawOutputChannel_->getData().aout_1);
                                                                       stat.add("Analog Out 2", recv_RawOutputChannel_->getData().aout_2);
                                                                       stat.add("Analog Out 3", recv_RawOutputChannel_->getData().aout_3);
                                                                       stat.add("Analog Out 4", recv_RawOutputChannel_->getData().aout_4);
                                                                   }));

    recv_ManualConditionChecks_.reset(new StatusUpdater<tc::ManualConditionChecks>("Manual Check Messages",
                                                                 ros::Duration(truck_ros_recv_rates_.recv_manual_check_conditions),
                                                                 [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                 {
                                                                     stat.add("Service Brake Pressed", static_cast<int>(recv_ManualConditionChecks_->getData().service_brake));
                                                                     stat.add("Door ajar", static_cast<int>(recv_ManualConditionChecks_->getData().door_ajar));
                                                                     stat.add("Parking Brake Sensor", static_cast<int>(recv_ManualConditionChecks_->getData().parking_brake));
                                                                     stat.add("Gear not manual", static_cast<int>(recv_ManualConditionChecks_->getData().gear_not_manual));
                                                                 }));

    recv_BrakeFaultConditionChecks_.reset(new StatusUpdater<tc::BrakeFaultConditionChecks>("Brake Fault Messages",
                                                                               ros::Duration(truck_ros_recv_rates_.recv_brake_fault_checks),
                                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                               {
                                                                                   stat.add("Loss of air in primary circuit", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().air_loss_primary));
                                                                                   stat.add("Loss of air in secondary circuit", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().air_loss_primary));
                                                                                   stat.add("Primary robotic valve mechanical failure", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().valve_failure_primary));
                                                                                   stat.add("Secondary robotic valve mechanical failure", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().valve_failure_secondary));
                                                                                   stat.add("Primary valve power loss", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().controller_power_primary));
                                                                                   stat.add("Secondary valve power loss", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().controller_power_secondary));
                                                                                   stat.add("Primary valve loss comms", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().controller_comms_primary));
                                                                                   stat.add("Secondary valve loss comms", static_cast<int>(recv_BrakeFaultConditionChecks_->getData().controller_comms_secondary));
                                                                               }));


    recv_EmergencyFaultConditionChecks_.reset(new StatusUpdater<tc::EmergencyFaultConditionChecks>("Emergency Fault Messages",
                                                                               ros::Duration(truck_ros_recv_rates_.recv_emergency_fault_checks),
                                                                               [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                               {
                                                                                   stat.add("firmware_load",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().firmware_load));
                                                                                   stat.add("settings_invalid",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().settings_invalid));
                                                                                   stat.add("vehicle_can2_error_passive",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vehicle_can2_error_passive));
                                                                                   stat.add("vehicle_can2_bus_off",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vehicle_can2_bus_off));
                                                                                   stat.add("dbw_can1_error_passive",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().dbw_can1_error_passive));
                                                                                   stat.add("dbw_can1_bus_off",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().dbw_can1_bus_off));
                                                                                   stat.add("vbus_low_voltage",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vbus_low_voltage));
                                                                                   stat.add("system_reset_watchdog",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().system_reset_watchdog));
                                                                                   stat.add("uncalibrated_settings",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().uncalibrated_settings));
                                                                                   stat.add("vehicle_speed_msg_expirerd",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vehicle_speed_msg_expirerd));
                                                                                   stat.add("brake_pressure_error_threshold",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().brake_pressure_error_threshold));
                                                                                   stat.add("brake_emergency_fault",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().brake_emergency_fault));
                                                                                   stat.add("aux_throt_module_timeout",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().aux_throt_module_timeout));
                                                                                   stat.add("accel_sync_signal_lost",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().accel_sync_signal_lost));
                                                                                   stat.add("accel_in_sensor_threshold",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().accel_in_sensor_threshold));
                                                                                   stat.add("accel_out_sensor_threshold",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().accel_out_sensor_threshold));
                                                                                   stat.add("accel_in_out_mismatch",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().accel_in_out_mismatch));
                                                                                   stat.add("test_mode_timeout",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().test_mode_timeout));
                                                                                   stat.add("axiom_can4_error_passive",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().axiom_can4_error_passive));
                                                                                   stat.add("axiom_can4_bus_off",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().axiom_can4_bus_off));
                                                                                   stat.add("vehicle_can3_error_passive",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vehicle_can3_error_passive));
                                                                                   stat.add("vehicle_can3_bus_off",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().vehicle_can3_bus_off));
                                                                                   stat.add("command_msg_timeout",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().command_msg_timeout));
                                                                                   stat.add("axiomatic_msg_timeout",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().axiomatic_msg_timeout));
                                                                                   stat.add("truck_feedback_msg_timeout",static_cast<int>(recv_EmergencyFaultConditionChecks_->getData().truck_feedback_msg_timeout));
                                                                               }));

    recv_LedStatusEcho_.reset(new StatusUpdater<tc::LEDStatusEcho>("LED Status Echo",
                                                                   ros::Duration(truck_ros_recv_rates_.recv_brake_fault_checks),
                                                                   [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                   {
                                                                       stat.add("fault_led",static_cast<int>(recv_LedStatusEcho_->getData().fault_led));
                                                                       stat.add("robotic_led",static_cast<int>(recv_LedStatusEcho_->getData().robotic_led));
                                                                       stat.add("emo_led",static_cast<int>(recv_LedStatusEcho_->getData().emo_led));
                                                                       stat.add("audible_buzzer",static_cast<int>(recv_LedStatusEcho_->getData().audible_buzzer));
                                                                   }));

    recv_SettingsCrc_.reset(new StatusUpdater<tc::SettingsCrc>("Settings CRC",
                                                                   ros::Duration(truck_ros_recv_rates_.recv_settings_crc),
                                                                   [this](diagnostic_updater::DiagnosticStatusWrapper& stat)
                                                                   {
                                                                       stat.add("nvram_settings_crc",recv_SettingsCrc_->getData().nvram_settings_crc);
                                                                       stat.add("ram_settings_crc",recv_SettingsCrc_->getData().ram_settings_crc);
                                                                   }));
}
