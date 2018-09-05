#pragma once
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
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THEp
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "truck_dbw_controller.h"

#include <ros/ros.h>

// #include <cav_driver_utils/can/socketcan_interface/socketcan_interface.h>
// #include <cav_driver_utils/can/ros_socketcan_bridge/ros_socketcan_bridge.h>

#include <driver_application/driver_application.h>

#include <cav_srvs/SetEnableRobotic.h>
#include <cav_srvs/EngineBrakeControl.h>
#include <cav_srvs/GetLights.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/ClearFaults.h>

#include <cav_msgs/RobotEnabled.h>
#include <cav_msgs/SpeedAccel.h>
#include <std_msgs/Float32.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <utility>

// #include <vector>
// #include <memory>

/**
 * @brief Class encapsulates the ROS behaviour for the TruckApplication
 */
class TruckApplication : public cav::DriverApplication
{
public:
    enum class lastCommandMode : uint8_t
    {
        ROBOTIC_DISABLE = 0,
        WRENCH_EFFORT_COMAND = 1,
        SPEED_ACCEL_COMMAND = 2
    };

    TruckApplication(int argc, char **argv, const std::string &name = "truck");
    virtual ~TruckApplication();

    virtual std::vector<std::string> &get_api() override { return api_; }

private:
    // cav::DriverApplication Members
    virtual void initialize() override;  // Setup specified client interface
    void pre_spin();    // Unused
    void post_spin();   // Unused
    void sendCommands();
    void checkLastReceivedMsgs();
    void initializeDBWController();
    void setupSubscribersPublishers();
    void registerEventHandlers();

    void updateRoboticModePedalPercent(TruckDBWController::RoboticMode_PedalPosition signal_data);
    void updateControlMessageEcho(TruckDBWController::ControlMessageEcho signal_data);
    void updatePrimaryBrakeAxiomatic(TruckDBWController::PrimaryBrakeAxiomatic signal_data);
    void updateSecondaryBrakeAxiomatic(TruckDBWController::SecondaryBrakeAxiomatic signal_data);
    void updateEngineBrakeControlFb(TruckDBWController::EngineBrakeControl signal_data);
    void updateSpeedControllerPidError(TruckDBWController::SpeedControllerPIDParams signal_data);
    void updateSpeedControllerPidEcho(TruckDBWController::SpeedControllerPIDEcho signal_data);
    void updateGeneralEcuStatus(TruckDBWController::GeneralECUStatus signal_data);
    void updateRawInputChannel(TruckDBWController::RawInputChannel signal_data);
    void updateRawOutputChannel(TruckDBWController::RawOutputChannel signal_data);
    void updateManualConditionChecks(TruckDBWController::ManualConditionChecks signal_data);
    void updateBrakeFaultConditionChecks(TruckDBWController::BrakeFaultConditionChecks signal_data);
    void updateEmergencyFaultConditionChecks(TruckDBWController::EmergencyFaultConditionChecks signal_data);
    void updateLedStatusEcho(TruckDBWController::LEDStatusEcho signal_data);
    void updateSettingsCrc(TruckDBWController::SettingsCrc signal_data);

    bool enableRobotic(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticResponse &res);
    void updateControlStatus(const ros::WallTimerEvent &);  // Publishes a topic containing the robotic status
    bool engineBrakeControl(cav_srvs::EngineBrakeControlRequest &req, cav_srvs::EngineBrakeControlResponse &res);
    bool getLightStatus(cav_srvs::GetLightsRequest &req, cav_srvs::GetLightsResponse &res);
    bool setLightStatus(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse &res);
    bool clearFaults(cav_srvs::ClearFaultsRequest &req, cav_srvs::ClearFaultsResponse &res);

    virtual void shutdown() override ;    // Properly shutdown client interface

    // ROS Members
    std::shared_ptr<ros::NodeHandle> control_message_nh_;
    ros::Subscriber wrench_effort_subscriber_;
    ros::Subscriber speed_accel_subscriber_;
    ros::Publisher robotic_status_pub_;
    ros::ServiceServer enable_robotic_service_;
    ros::ServiceServer engine_brake_service_;
    ros::ServiceServer get_lights_service_;
    ros::ServiceServer set_lights_service_;
    ros::ServiceServer clear_faults_service_;

    ros::WallTimer status_publisher_timer_;

    std::unique_ptr<diagnostic_updater::Updater> updater_;

    double k_p_;
    double k_i_;
    double k_d_;
    bool robotic_enabled_; // Disables or Enables Robotic Command
    bool clear_faults_enabled_; // Enables Clear Faults Message to be sent

    bool disable_robotic_printed_flag_; // Tracks robotic stream info print
    bool robotic_wrench_control_active_;    // Tracks if robotic wrench is currently being commanded
    bool robotic_speed_control_active_;     // Tracks if robotic speed is currently being commanded

    // stores the api that is returned to the base DriverApplication class
    std::vector<std::string> api_;
    // Unique Pointer for Cav SocketCan Type Client
    std::unique_ptr<TruckDBWController> truck_dbw_ctrl_;
    // Configuration Settings 
    truck::TruckSettings truck_cfg_;
    // Truck DBW Can Send Settings
    truck::TruckDbwSendRates truck_dbw_send_rates_;
    // Truck Ros Recv Settings
    truck::TruckRosRecvRates truck_ros_recv_rates_;
    // Mutex for locking 
    std::recursive_mutex status_mutex_;

    // Last known (sent) command mode
    lastCommandMode last_cmd_mode_;

    // Structs to track variables of last iterations
    TruckDBWController::ControlMessage last_ControlMessage_;
    TruckDBWController::EngineBrakeCommand last_EngineBrakeCommand_;
    TruckDBWController::PidParams last_PidParam_;
    TruckDBWController::LEDStatus last_LedStatus_;
    TruckDBWController::LightStatus last_FrontLightStatus_;
    TruckDBWController::LightStatus last_RearLightStatus_;

    ros::Time last_sent_control_mode_message_;
    ros::Time last_sent_engine_braking_message_;
    ros::Time last_sent_pid_gains_message_;
    ros::Time last_sent_led_status_message_;
    ros::Time last_sent_front_lights_msg_;
    ros::Time last_sent_rear_lights_msg_;

    // Varibles used in the latest spin iteration
    TruckDBWController::ControlMessage set_ContolMessage_;
    ros::Time control_message_recv_time;

    TruckDBWController::EngineBrakeCommand set_EngineBrakeCommand_;
    TruckDBWController::PidParams set_PidParam_;
    TruckDBWController::LEDStatus set_LedStatus_;
    TruckDBWController::LightStatus set_LightControl_;

    ros::Time last_recv_roboticmode_pedalposition_;
    ros::Time last_recv_control_message_echo_;
    ros::Time last_recv_prim_brake_axiomatic_;
    ros::Time last_recv_sec_brake_axiomatic_;
    ros::Time last_recv_engine_brake_control_;
    ros::Time last_recv_speed_control_pid_params_;
    ros::Time last_recv_speed_control_pid_echo_;
    ros::Time last_recv_general_ecu_status_;
    ros::Time last_recv_raw_input_channel_;
    ros::Time last_rev_raw_output_channel_;
    ros::Time last_recv_manual_check_conditions_;
    ros::Time last_recv_brake_fault_checks_;
    ros::Time last_recv_emergency_fault_checks_;
    ros::Time last_recv_led_status_echo_;
    ros::Time last_recv_settings_crc_;


    template<class T>
    class StatusUpdater : public diagnostic_updater::DiagnosticTask
    {
        ros::Time last_updated_;
        const ros::Duration update_period_;
        bool has_timeout_;
        std::function< void(diagnostic_updater::DiagnosticStatusWrapper&)> f_;
    public:

        T data_;

        StatusUpdater(const std::string &name,
                      ros::Duration update_period,
                      std::function< void(diagnostic_updater::DiagnosticStatusWrapper&)> status_loader)
            : DiagnosticTask(name),
              f_(status_loader),
              has_timeout_(true),
              update_period_(update_period),
              last_updated_(ros::Time::now())
        {
        }

        StatusUpdater(const std::string &name,
                      std::function< void(diagnostic_updater::DiagnosticStatusWrapper&)> status_loader)
            : DiagnosticTask(name),
              f_(status_loader),
              has_timeout_(false),
              last_updated_(ros::Time::now())
        {
        }


        inline void setData(const T& data)
        {
            last_updated_ = ros::Time::now();
            data_ = data;
        }

        inline T getData() { return data_; }

        inline void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override
        {
            auto dur = ros::Time::now() - last_updated_;
            if(has_timeout_ && dur > update_period_)
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::STALE, "Have not received data within update period");
            }
            else
            {
                stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Data received");
            }

            f_(stat);
        }

        inline ros::Duration timeSinceLast()
        {
            return ros::Time::now() - last_updated_;
        }
    };

    void setupRecvItems();

    // Structs to track recieved dbw messages
    std::unique_ptr<StatusUpdater<TruckDBWController::RoboticMode_PedalPosition>>       recv_RoboticMode_PedalPosition_;
    std::unique_ptr<StatusUpdater<TruckDBWController::ControlMessageEcho>>              recv_ControlMessageEcho_;
    std::unique_ptr<StatusUpdater<TruckDBWController::PrimaryBrakeAxiomatic>>           recv_PrimaryBrakeAxiomatic_;
    std::unique_ptr<StatusUpdater<TruckDBWController::SecondaryBrakeAxiomatic>>         recv_SecondaryBrakAxiomatic_;
    std::unique_ptr<StatusUpdater<TruckDBWController::EngineBrakeControl>>              recv_EngineBrakeControlFb_;
    std::unique_ptr<StatusUpdater<TruckDBWController::SpeedControllerPIDParams>>        recv_SpeedControlPidParams_;
    std::unique_ptr<StatusUpdater<TruckDBWController::SpeedControllerPIDEcho>>          recv_SpeedControlPidEcho_;
    std::unique_ptr<StatusUpdater<TruckDBWController::GeneralECUStatus>>                recv_GeneralEcuStatus_;
    std::unique_ptr<StatusUpdater<TruckDBWController::RawInputChannel>>                 recv_RawInputChannel_;
    std::unique_ptr<StatusUpdater<TruckDBWController::RawOutputChannel>>                recv_RawOutputChannel_;
    std::unique_ptr<StatusUpdater<TruckDBWController::ManualConditionChecks>>           recv_ManualConditionChecks_;
    std::unique_ptr<StatusUpdater<TruckDBWController::BrakeFaultConditionChecks>>       recv_BrakeFaultConditionChecks_;
    std::unique_ptr<StatusUpdater<TruckDBWController::EmergencyFaultConditionChecks>>   recv_EmergencyFaultConditionChecks_;
    std::unique_ptr<StatusUpdater<TruckDBWController::LEDStatusEcho>>                   recv_LedStatusEcho_;
    std::unique_ptr<StatusUpdater<TruckDBWController::SettingsCrc>>                     recv_SettingsCrc_;

};