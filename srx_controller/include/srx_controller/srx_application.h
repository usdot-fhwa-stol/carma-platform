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


#include <driver_application/driver_application.h>
#include <srx_controller/srx_dbw_controller.h>

#include <cav_srvs/GetLights.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetEnableRobotic.h>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <ros/ros.h>

#include <memory>
#include <vector>


/**
 * @brief SRXApplication
 *
 * This class is the ROS Node providing access to the SRX DBW Module
 */
class SRXApplication : public cav::DriverApplication {
private:

    //Private types

    /**
     * @brief This class is used to report the socketCAN error frames to the diagnostic updater
     */
    class SocketCANDiagnosticUpdater : public diagnostic_updater::DiagnosticTask {
        ros::Time last_update;
        torc::SRXDBWController::ErrorCode_t error_code_;
    public:

        SocketCANDiagnosticUpdater() : DiagnosticTask("Socket CAN Diagnostic") { error_code_.code = 0; }

        /**
         * @brief Sets the error code of this class
         * @param code The error code of this class
         */
        inline void setErrorCode(const torc::SRXDBWController::ErrorCode_t &code);

        /**
         * @brief Constructs the diagnostic updater structure
         * @param stat
         */
        virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;
    };


    /**
     * @brief This class is used to report faults with communicating with the hardware device
     */
    class SRXControllerFaultTask : public diagnostic_updater::DiagnosticTask
    {
    public:
        SRXControllerFaultTask() : DiagnosticTask("SRXContollerFaultTask") {}

        bool NoUpdatesReceived, ControllerIsNotReceivingUpdates, CommandMismatch, VehicleBusTimeout;

        void reset();

    private:
        virtual void run(diagnostic_updater::DiagnosticStatusWrapper &stat) override;

    };


    enum class CommandMode_t {
        None,
        Wrench,
        ClosedLoop
    };

public:
    SRXApplication(int argc, char **argv);


private:


    // ROS members
    ros::Subscriber effort_sub_, speed_sub_;
    ros::ServiceServer get_lights_srv_, set_lights_srv_, enable_robotic_srv_;
    ros::Publisher robotic_status_pub_;
    std::shared_ptr<ros::NodeHandle> control_nh_;
    ros::WallTimer status_publisher_timer_;

    // stores the api that is returned to the base DriverApplication class
    std::vector<std::string> api_list_;

    //the SRXDBWController this node communicates with
    std::shared_ptr<torc::SRXDBWController> dbw_ctrl_;


    //Set Parameters
    torc::SRXDBWController::PIDParams_t PID_params_set_{};
    double k_p_, k_i_, k_d_;

    torc::SRXDBWController::LightParams_t light_set_{};


    //Feedback Paramaters
    torc::SRXDBWController::LightParams_t front_lights_status_{}, rear_light_status_{};
    torc::SRXDBWController::ThrottleOutputFeedback_t throttle_feedback_{};
    torc::SRXDBWController::BrakeOutputFeedback_t brake_feedback_{};
    torc::SRXDBWController::PIDParams_t PID_params_status_{};
    ros::Time last_status_update_;
    std::recursive_mutex status_mutex_;


    //Command variables

    bool robotic_enabled_; //if false we command disableRoboticControl

    // varibles used in the latest spin iteration
    CommandMode_t cmd_mode_;
    double set_speed_;
    double set_accel_;
    float wrench_effort_;

    ros::Time last_sent_cmd_time_;
    torc::SRXDBWController::ControlMessage_t last_sent_cmd_message_;

    //Diagnostic updater members
    diagnostic_updater::Updater diag_updater_;
    SocketCANDiagnosticUpdater socket_CAN_updater_;
    SRXControllerFaultTask fault_updater_task_;

    //DriverApplication implementation members

    /**
     * @brief Implements the DriverApplication initialize()
     *
     * This function sets up the SRXDBWController. Establishes ROS services and subscribes/adverties
     * necessary topics
     */
    void initialize() override;

    /**
     * @brief IMplements the DriverApplication pre_spin() method
     *
     * Prepare state variables for current iteration, Primarily resets fault_updater_task_ variables, and resets
     * the cmd_mod_ to None
     */
    void pre_spin() override;


    /**
     * @brief Implements the DriverApplication post_spin() method
     *
     * After collecting updates from this iteration spin callbacks we send out commands to the SRXDBWController.
     * Check for faults.
     */
    void post_spin() override;


    /**
     * @brief Returns the api implemented by this interface to the DriverApplication
     * @return
     */
    std::vector<std::string> &get_api() override;


    // Diagnostic updater tasks

    /**
     * @brief Updates the PID diagnostic report
     * @param stat
     */
    void PIDUpdaterTask(diagnostic_updater::DiagnosticStatusWrapper &stat);


    /**
     * @brief Updates the SRX DBW feedback diagnostic report
     * @param stat
     */
    void feedbackUpdaterTask(diagnostic_updater::DiagnosticStatusWrapper& stat);


    /**
     * @brief Helper function that performs common functionality for setting the fault state of the driver
     *
     * This function sets driver status to Fault, and calls disableRoboticControl on the dbw controller
     * It also sets the variable that will be used by the diagnostic updaters aswell as publishing a message
     * to ROS_DEBUG
     *
     * @param set - target variable that will be set
     * @param msg - message that is sent to ROS_DEBUG_STREAM
     */
    void setFault(bool & set, const std::string& msg);


    //ROS service callbacks

    /**
     * @brief Returns the set state of the lights
     *
     * This may not match up with the hardware, we return the current set state that was last sent
     * by the set_lights service not what the hardware is doing. If this does not match the hardware status
     * this controller will continue to attempt to set the lights.
     * @param req
     * @param resp
     * @return true always
     */
    bool get_lights_cb(cav_srvs::GetLightsRequest &req, cav_srvs::GetLightsResponse &resp);


    /**
     * @brief Sets the light set state
     *
     * This will set the internal variable for the desired set state of the lights. The controller will attempt
     * to command the hardware to set the lights to match this state
     * @param req
     * @param resp
     * @return true always
     */
    bool set_lights_cb(cav_srvs::SetLightsRequest &req, cav_srvs::SetLightsResponse &resp);


    /**
     * @brief Sets the internal robotic_enabled flag
     *
     * If the robotic_enabled flag is set ( true )this controller will forward messages received on the input topics.
     * If this flag is unset ( false ) this controll will send robotic disable to the hardware every iteration
     * @param req req.set is used to set robotic_enabled
     * @param resp
     * @return true always
     */
    bool enable_robotic_cb(cav_srvs::SetEnableRoboticRequest &req, cav_srvs::SetEnableRoboticRequest &resp);

    /**
     * @brief Timer Callback that updates the robot_status topic
     */
    void statusUpdateTimerCB(const ros::WallTimerEvent &);





};

