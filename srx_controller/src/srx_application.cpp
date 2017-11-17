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


#include <srx_controller/srx_application.h>

#include <cav_msgs/SpeedAccel.h>
#include <cav_msgs/RobotEnabled.h>

#include <std_msgs/Float32.h>

#include <boost/tokenizer.hpp>

namespace carma
{
    const double max_commanded_speed = 35.76; //m/s
    const double max_commanded_accel = 2.5; //m/s/s
}


SRXApplication::SRXApplication(int argc, char **argv) : cav::DriverApplication(argc, argv, "srx_controller") {}

void SRXApplication::initialize() {

    control_nh_.reset(new ros::NodeHandle("~control"));

    pnh_->param<double>("k_p", k_p_, 0.1);
    pnh_->param<double>("k_i", k_i_, 0.01);
    pnh_->param<double>("k_d", k_d_, 0.005);
    pnh_->param<bool>("enabled_at_start", robotic_enabled_, false);

    PID_params_set_.setF(k_p_, k_i_, k_d_);

    //default all lights off;
    light_set_.FlashOn = false;
    light_set_.LeftArrowOn = false;
    light_set_.RightArrowOn = false;
    light_set_.TakeDownOn = false;

    //we assume these are correct until we get an update from the controller
    front_lights_status_ = light_set_;
    rear_light_status_ = light_set_;


    dbw_ctrl_.reset(new torc::SRXDBWController(*nh_, *pnh_));

    dbw_ctrl_->connect("received_messages", "sent_messages");

    dbw_ctrl_->onPIDEchoRecv.connect([this](const torc::SRXDBWController::PIDParams_t &params) {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        PID_params_status_ = params;
        ROS_DEBUG_STREAM("PID params: " << std::endl << params);
    });

    dbw_ctrl_->onLightStatusRecv.connect(
            [this](const torc::SRXDBWController::LightID_t &id, const torc::SRXDBWController::LightParams_t params) {
                std::unique_lock<std::recursive_mutex> lock(status_mutex_);
                switch (id) {
                    case torc::SRXDBWController::LightID_t::Front: {
                        front_lights_status_ = params;
                        ROS_DEBUG_STREAM("Front Lights: " << std::endl << params);
                        break;
                    }
                    case torc::SRXDBWController::LightID_t::Rear: {
                        rear_light_status_ = params;
                        ROS_DEBUG_STREAM("Rear Lights: " << std::endl << params);
                        break;
                    }
                }

            });

    dbw_ctrl_->onThrottleFeedbackRecv.connect(
            [this](const torc::SRXDBWController::ThrottleOutputFeedback_t &feedback) {
                std::unique_lock<std::recursive_mutex> lock(status_mutex_);
                throttle_feedback_ = feedback;
                last_status_update_ = ros::Time::now();
                ROS_DEBUG_STREAM(throttle_feedback_);
            });

    dbw_ctrl_->onBrakeFeedbackRecv.connect([this](const torc::SRXDBWController::BrakeOutputFeedback_t &feedback) {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        brake_feedback_ = feedback;
        last_status_update_ = ros::Time::now();
        ROS_DEBUG_STREAM(brake_feedback_);
    });

    dbw_ctrl_->onErrorFrame.connect([this](const torc::SRXDBWController::ErrorCode_t &ec) {
        socket_CAN_updater_.setErrorCode(ec);
        ROS_ERROR_STREAM("Socket ERROR: " << ec.what());
    });

    dbw_ctrl_->onSentCommand.connect([this](const torc::SRXDBWController::ControlMessage_t& msg)
                                     {
                                         std::unique_lock<std::recursive_mutex> lock(status_mutex_);
                                         last_sent_cmd_time_ = ros::Time::now();
                                         last_sent_cmd_message_ = msg;
                                         ROS_DEBUG_STREAM(msg);
                                     });

    effort_sub_ = control_nh_->subscribe<std_msgs::Float32>("cmd_longitudinal_effort",
                                                            1,
                                                            [this](const std_msgs::Float32ConstPtr &msg) {
                                                                wrench_effort_ = msg->data;
                                                                cmd_mode_ = CommandMode_t::Wrench;
                                                            });
    api_list_.push_back(effort_sub_.getTopic());

    speed_sub_ = control_nh_->subscribe<cav_msgs::SpeedAccel>("cmd_speed",
                                                              1,
                                                              [this](const cav_msgs::SpeedAccelConstPtr &msg) {

                                                                if(msg->speed < 0 || msg->max_accel < 0)
                                                                {
                                                                    ROS_ERROR_STREAM("Invalid command received: speed: " << msg->speed << ", max_accel: " << msg->max_accel);
                                                                    cmd_mode_ = CommandMode_t::None;
                                                                }
                                                                else
                                                                {
                                                                    ROS_DEBUG_STREAM("Received command: " << msg);
                                                                    double speed = msg->speed;
                                                                    double accel = msg->accel;

                                                                    if(speed > carma::max_commanded_speed)
                                                                    {
                                                                        speed  = carma::max_commanded_speed
                                                                        ROS_WARN_STEAM("Speed Command exceeds max allowed, capping to: " << speed);
                                                                    }

                                                                    if(accel > carma::max_commanded_accel)
                                                                    {
                                                                        accel = carma::max_commanded_accel;
                                                                        ROS_WARN_STEAM("Max Accel Command exceeds max allowed, capping to: " << accel);
                                                                    }
                                                                    
                                                                    set_speed_ =  speed;
                                                                    set_accel_ = accel;
                                                                    cmd_mode_ = CommandMode_t::ClosedLoop;
                                                                }
                                                              });

    api_list_.push_back(speed_sub_.getTopic());

    robotic_status_pub_ = control_nh_->advertise<cav_msgs::RobotEnabled>("robot_status", 1);
    api_list_.push_back(robotic_status_pub_.getTopic());

    get_lights_srv_ = control_nh_->advertiseService("get_lights", &SRXApplication::get_lights_cb, this);
    api_list_.push_back(get_lights_srv_.getService());

    set_lights_srv_ = control_nh_->advertiseService("set_lights", &SRXApplication::set_lights_cb, this);
    api_list_.push_back(set_lights_srv_.getService());

    enable_robotic_srv_ = control_nh_->advertiseService("enable_robotic", &SRXApplication::enable_robotic_cb, this);
    api_list_.push_back(enable_robotic_srv_.getService());


    status_publisher_timer_ = nh_->createWallTimer(ros::WallDuration(ros::Rate(2)),&SRXApplication::statusUpdateTimerCB, this);

    status_publisher_timer_.start();

    cav_msgs::DriverStatus status;
    status.controller = static_cast<unsigned char>(true);
    status.status = cav_msgs::DriverStatus::OPERATIONAL;

    setStatus(status);

    diag_updater_.setHardwareID("SRXDBWController");
    diag_updater_.add(socket_CAN_updater_);
    diag_updater_.add("PID Status", boost::bind(&SRXApplication::PIDUpdaterTask, this, _1));
    diag_updater_.add("DBW Module Feedback",boost::bind(&SRXApplication::feedbackUpdaterTask, this, _1));
    diag_updater_.add(fault_updater_task_);
}

void SRXApplication::PIDUpdaterTask(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    torc::SRXDBWController::PIDParams_t status;
    {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        status = PID_params_status_;
    }

    if(PID_params_set_ != status)
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::WARN, "PID not set at controller");
    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "PID set");
    }

    stat.addf("PID Set", "%f %f %f", PID_params_set_.p(),PID_params_set_.i(),PID_params_set_.d());
    stat.addf("PID Status", "%f %f %f", status.p(),status.i(),status.d());
}

void SRXApplication::feedbackUpdaterTask(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    static auto OK   = static_cast<diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::OK);
    static auto WARN = static_cast<diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::WARN);

    torc::SRXDBWController::ThrottleOutputFeedback_t throttleOutputFeedback;
    torc::SRXDBWController::BrakeOutputFeedback_t brakeOutputFeedback;
    {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        throttleOutputFeedback = throttle_feedback_;
        brakeOutputFeedback = brake_feedback_;
    }

    stat.add("Primed", throttleOutputFeedback.ACCPrimed);
    stat.add("Throttle DBWEnabled", throttleOutputFeedback.AccelDBWEnabled);
    stat.add("Brake DBWEnabled", brakeOutputFeedback.BrakeDBWEnabled);
    stat.add("ThrottleReengageReq", throttleOutputFeedback.ThrottleReengageReq );
    stat.add("DriverOverride", throttleOutputFeedback.DriverOverride );
    stat.add("ThrottleCommandEcho", (int)throttleOutputFeedback.AccelCmdModeEcho);
    stat.add("BrakeCommandEcho", (int)brakeOutputFeedback.BrakeCmdModeEcho);

    if(throttleOutputFeedback.AccelCmdActive || brakeOutputFeedback.BrakeCmdInjection)
    {
        stat.summaryf(OK, "Controller is active robotic");
    }
    else if(throttleOutputFeedback.DriverOverride
            || !throttleOutputFeedback.ACCPrimed
            || !throttleOutputFeedback.AccelDBWEnabled
            || !brakeOutputFeedback.BrakeDBWEnabled
            || throttleOutputFeedback.ThrottleReengageReq)
    {
        stat.summaryf(WARN, "Controller unable to command robotic");
    }
    else
    {
        stat.summaryf(OK, "Controller awaiting robotic commands");
    }

}

void SRXApplication::pre_spin() {
    //run diagnostic updater
    diag_updater_.update();

    //set command mode to none so that we only send if we received a command to send this iteration
    cmd_mode_ = CommandMode_t::None;

    //reset the fault updater that is keeping track of faults per iteration
    fault_updater_task_.reset();
}

void SRXApplication::post_spin() {

    torc::SRXDBWController::PIDParams_t pidParams{};
    torc::SRXDBWController::LightParams_t rearLightParams{};
    torc::SRXDBWController::LightParams_t frontLightParams{};
    torc::SRXDBWController::ThrottleOutputFeedback_t throttleOutputFeedback{};
    torc::SRXDBWController::ControlMessage_t lastCmdMessage{};
    ros::Duration time_since_cmd;
    ros::Duration time_since_update;

    //Get copies of data
    {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        pidParams              = PID_params_status_;
        rearLightParams        = rear_light_status_;
        frontLightParams       = front_lights_status_;
        time_since_update      = ros::Time::now() - last_status_update_;
        throttleOutputFeedback = throttle_feedback_;
        lastCmdMessage         = last_sent_cmd_message_;
        time_since_cmd         = ros::Time::now() - last_sent_cmd_time_;
    }


    //If out param status do not match the set state we attempt to set them
    if (pidParams != PID_params_set_) {
        ROS_DEBUG_STREAM_THROTTLE(1, "Updating PID");
        dbw_ctrl_->setPID(PID_params_set_);
    }

    if (frontLightParams != light_set_) {
        ROS_DEBUG_STREAM_THROTTLE(1, "Updating front light state");
        dbw_ctrl_->setLights(torc::SRXDBWController::LightID_t::Front, light_set_);
    }

    if (rearLightParams != light_set_) {
        ROS_DEBUG_STREAM_THROTTLE(1, "Updating rear light state");
        dbw_ctrl_->setLights(torc::SRXDBWController::LightID_t::Rear, light_set_);
    }


    if (time_since_update > ros::Duration(0.2))
    {
        setFault(fault_updater_task_.NoUpdatesReceived,"Not receiving updates from controller");
        return;
    }


    //Check to see if the controller hardware is still communicating with the Vehicle Bus
    {
        static int counter = 0;
        if(!throttleOutputFeedback.VehicleBusTimeout)
        {
            counter = 0;
        }
        else if(++counter > spin_rate / 2)
        {
            setFault(fault_updater_task_.VehicleBusTimeout, "Vehicle Bus Timeout");
            return;
        }
    }


    //Check the command timeout, if we have sent a command recently we should expect the timeout flag to be unset
    {
        static int counter = 0;
        if(!throttleOutputFeedback.AccelCmdTimeout)
        {
            counter = 0;
        }
        else if(time_since_cmd < ros::Duration(0.2) && ++counter > spin_rate / 2)
        {
            setFault(fault_updater_task_.ControllerIsNotReceivingUpdates, "Controller is not receiving commands");
            return;
        }
    }


    //this is as intended, if the controller is reporting disabled
    //but when we send disable robotic we are no longer mismatched with the commands
    //so we then set the status to Operational, if the hardware is latched to disabled then
    //the status of this node will alternate between operational and fault
    {
        static int counter = 0;
        if(lastCmdMessage.CommandMode != throttleOutputFeedback.AccelCmdModeEcho && ++counter > spin_rate/2)
        {
            setFault(fault_updater_task_.CommandMismatch, "Controller command mismatch");
        }
        else
        {
            counter = 0;
        }
    }

    auto status = getStatus();
    status.status = cav_msgs::DriverStatus::OPERATIONAL;
    setStatus(status);

    if (!robotic_enabled_)
    {
        ROS_DEBUG_STREAM_THROTTLE(2, "Commanding disable robotic control");
        dbw_ctrl_->disableRoboticControl();
    }
    else if (cmd_mode_ == CommandMode_t::Wrench)
    {
        ROS_DEBUG_STREAM_THROTTLE(2, "Commanding wrench effort: " << wrench_effort_);
        dbw_ctrl_->setWrenchEffort(wrench_effort_);
    }
    else if (cmd_mode_ == CommandMode_t::ClosedLoop)
    {
        ROS_DEBUG_STREAM_THROTTLE(2, "Commanding speed: " << set_speed_ << ", " << set_accel_);
        dbw_ctrl_->setSpeedAccel(set_speed_, set_accel_);
    }

}

std::vector<std::string> &SRXApplication::get_api() {
    return api_list_;
}

bool SRXApplication::get_lights_cb(cav_srvs::GetLightsRequest&, cav_srvs::GetLightsResponse &resp) {
    static auto ON = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::ON);
    static auto OFF = static_cast<cav_msgs::LightBarStatus::_flash_type>(cav_msgs::LightBarStatus::OFF);

    resp.status.flash       = light_set_.FlashOn        ? ON : OFF;
    resp.status.takedown    = light_set_.TakeDownOn     ? ON : OFF;
    resp.status.left_arrow  = light_set_.LeftArrowOn    ? ON : OFF;
    resp.status.right_arrow = light_set_.RightArrowOn   ? ON : OFF;

    return true;
}

bool SRXApplication::set_lights_cb(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse&) {
    using cav_msgs::LightBarStatus;

    light_set_.RightArrowOn = req.set_state.right_arrow == LightBarStatus::ON;
    light_set_.LeftArrowOn  = req.set_state.left_arrow  == LightBarStatus::ON;
    light_set_.FlashOn      = req.set_state.flash       == LightBarStatus::ON;
    light_set_.TakeDownOn   = req.set_state.takedown    == LightBarStatus::ON;

    return true;
}

bool SRXApplication::enable_robotic_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticRequest&) {
    bool tmp = robotic_enabled_;
    robotic_enabled_ = req.set == cav_srvs::SetEnableRoboticRequest::ENABLE;
    ROS_INFO_STREAM_COND(tmp != robotic_enabled_,
                         "Robotic_enabled set to: " << robotic_enabled_ ? "true" : "false");
    return true;
}

void SRXApplication::statusUpdateTimerCB(const ros::WallTimerEvent &) {

    torc::SRXDBWController::ThrottleOutputFeedback_t throttleOutputFeedback{};
    torc::SRXDBWController::BrakeOutputFeedback_t brakeOutputFeedback{};

    {
        std::unique_lock<std::recursive_mutex> lock(status_mutex_);
        throttleOutputFeedback  = throttle_feedback_;
        brakeOutputFeedback     = brake_feedback_;
    }

    cav_msgs::RobotEnabled msg;
    msg.robot_active = static_cast<cav_msgs::RobotEnabled::_robot_enabled_type>(throttleOutputFeedback.AccelCmdActive
                                                                                || brakeOutputFeedback.BrakeCmdInjection);
    msg.robot_enabled   = robotic_enabled_;
    msg.brake_decel     = brakeOutputFeedback.InjectedBrakeForce;
    msg.torque          = throttleOutputFeedback.InjectedTorque;

    robotic_status_pub_.publish(msg);
}

void SRXApplication::setFault(bool &set, const std::string &msg) {

    ROS_WARN_STREAM_THROTTLE(1, msg);
    set = true;
    auto status = getStatus();
    status.status = cav_msgs::DriverStatus::FAULT;
    setStatus(status);

    dbw_ctrl_->disableRoboticControl();
}

void SRXApplication::SRXControllerFaultTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    static auto OK   = static_cast<diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::OK);
    static auto ERROR = static_cast<diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::ERROR);
    stat.add("NoUpdatesReceived", NoUpdatesReceived);
    stat.add("ControllerIsNotReceivingUpdates",ControllerIsNotReceivingUpdates);
    stat.add("CommandMismatch",CommandMismatch);
    stat.add("VehicleBusTimeout", VehicleBusTimeout);
    if(NoUpdatesReceived)
    {
        stat.summaryf(ERROR,"Not receiving updates from controller");

    }
    else if(ControllerIsNotReceivingUpdates)
    {

        stat.summaryf(ERROR, "Controller is not receiving commands");
    }
    else if(VehicleBusTimeout)
    {
        stat.summaryf(ERROR, "Controller is unable to communicate with Vehicle");
    }
    else if(CommandMismatch)
    {
        stat.summaryf(ERROR, "Command mistmatch");
    }
    else
    {
        stat.summaryf(OK, "Controller operational");
    }
}

void SRXApplication::SRXControllerFaultTask::reset() {
    NoUpdatesReceived = ControllerIsNotReceivingUpdates = CommandMismatch = VehicleBusTimeout = false;
}

void SRXApplication::SocketCANDiagnosticUpdater::run(diagnostic_updater::DiagnosticStatusWrapper &stat) {
    ros::Duration elapsed = ros::Time::now() - last_update;

    static auto ERROR = static_cast<const diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::ERROR);
    static auto STALE = static_cast<const diagnostic_msgs::DiagnosticStatus::_level_type>(diagnostic_msgs::DiagnosticStatus::STALE);
    if (error_code_.code != 0) {
        stat.summaryf(elapsed < ros::Duration(5.0) ? ERROR : STALE, "Socket CAN Error");
        typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
        boost::char_separator<char> sep("|");
        std::string str(error_code_.what());
        tokenizer tokens(str, sep);

        for (tokenizer::iterator it = tokens.begin(); it != tokens.end(); it++) {
            std::string s(*it);
            stat.add(s, "set");
        }

    }
    else
    {
        stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "Socket CAN OK");
    }
}

void SRXApplication::SocketCANDiagnosticUpdater::setErrorCode(const torc::SRXDBWController::ErrorCode_t &code) {
    error_code_ = code;
    last_update = ros::Time::now();
}
