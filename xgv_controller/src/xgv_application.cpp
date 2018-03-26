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

#include <std_msgs/Float32.h>
#include "xgv_application.h"

XGVApplication::XGVApplication(int argc, char **argv) :
    DriverApplication(argc,argv,"xgv_controller"), 
    cmd_mode_(cav::CommandMode_t::None)
{

}


void XGVApplication::initialize()
{

    XGVJausClient::XGVJausClientConfig cfg{};
    int temp;
    pnh_->param<int>("authority_level", temp, 6);
    cfg.authority_level = static_cast<unsigned char>(temp);

    pnh_->param<int>("node_id",temp, 1);
    cfg.node_id = static_cast<unsigned char>(temp);

    pnh_->param<int>("subsystem_id", temp, 1);
    cfg.subsystem_id= static_cast<unsigned char>(temp);

    pnh_->param<int>("motion_profile_duration_ms", temp,400);
    cfg.motion_profile_duration = static_cast<unsigned short>(temp);


    //create and connect the xgv client
    xgv_client_.reset(new XGVJausClient(cfg));

    xgv_client_->connect();

    {
        //Set up the effort controller. This manages the cmd_longitudinal_effort topic
        effort_controller_.reset(new cav::LongitudinalEffortController());
        auto& v = effort_controller_->get_api();
        api_.insert(api_.end(),v.begin(),v.end());
        effort_controller_->onNewCommand.connect([this](const cav::LongitudinalEffortController::Command&)
                                                 {
                                                     cmd_mode_ = cav::CommandMode_t::Wrench;
                                                 });

    }

    {
        //Set up the speed controller. This manages the cmd_speed topic
        speed_controller_.reset(new cav::LongitudinalSpeedController());
        auto& v = speed_controller_->get_api();
        api_.insert(api_.end(),v.begin(),v.end());
        speed_controller_->onNewCommand.connect([this](const cav::LongitudinalSpeedController::Command& cmd)
                                                {
                                                    if(cmd.speed < 0 || cmd.max_accel < 0)
                                                    {
                                                        ROS_ERROR_STREAM("Invalid command received: speed: " << cmd.speed << ", max_accel: " << cmd.max_accel);
                                                        cmd_mode_ = cav::CommandMode_t::None;
                                                    }
                                                    else
                                                    {
                                                        ROS_DEBUG_STREAM("Received command:\n\tSpeed: " << cmd.speed << "\n\tMax Accel: " << cmd.max_accel);
                                                        double speed = cmd.speed;
                                                        double accel = cmd.max_accel;

                                                        if(speed > cav::max_commanded_speed)
                                                        {
                                                            speed  = cav::max_commanded_speed;
                                                            ROS_WARN("Speed Command exceeds max allowed, capping to: %f", speed);
                                                        }

                                                        if(accel > cav::max_commanded_accel)
                                                        {
                                                            accel = cav::max_commanded_accel;
                                                            ROS_WARN("Max Accel Command exceeds max allowed, capping to: %f", accel);
                                                        }

                                                        speed_controller_->command.speed = speed;
                                                        speed_controller_->command.max_accel = accel;
                                                        cmd_mode_ = cav::CommandMode_t::ClosedLoop;
                                                    }
                                                });

    }



    {
        //This manages the robotic_enabled topic
        active_robotic_status_provider_.reset(new cav::ActiveRoboticStatusProvider());
        auto& v = active_robotic_status_provider_->get_api();
        api_.insert(api_.end(),v.begin(),v.end());
    }

    {
        //this manages the enable_robotic service
        enable_robotic_service_.reset(new cav::EnableRoboticService());
        auto& v = enable_robotic_service_->get_api();
        api_.insert(api_.end(),v.begin(),v.end());
        enable_robotic_service_->onEnabledChanged.connect([this](bool new_value)
                                                          {
                                                              ROS_INFO_STREAM("Robotic_enabled set to: " << (new_value ? "true" : "false"));
                                                              if(!new_value)
                                                              {
                                                                  cmd_mode_ = cav::CommandMode_t::DisableRobotic;
                                                              }
                                                            
                                                                if(active_robotic_status_provider_)
                                                                    active_robotic_status_provider_->setEnabled(new_value);
                                                          });
    }

    //Connect the vehicleMode signal so that we know when the robot is robot active
    xgv_client_->vehicleModeReceivedSignal.connect([this](const XGVJausClient::VehicleModeEventArgs& args)
                                                   {
                                                      ROS_DEBUG_STREAM_NAMED("vehicle_mode","Received VehicleMode Event: " << std::endl << args);
                                                      active_robotic_status_provider_->setActive(args.safetyMode == XGVJausClient::SafetyMode::DriveByWire);
                                                   });

    xgv_client_->wrenchEffortReceivedSignal.connect([this](const XGVJausClient::WrenchEffortSignalArgs& state)
                                                    {
                                                        ROS_DEBUG_STREAM_NAMED("primitive_driver","Received wrench effort report: throttle  " << state.throttle << ", brakes " << state.brakes);
                                                    });



    cav_msgs::DriverStatus status;
    status.lon_controller = static_cast<unsigned char>(true);
    status.status = cav_msgs::DriverStatus::OPERATIONAL;

    setStatus(status);

    spin_rate = 50;


}


void XGVApplication::pre_spin()
{
    active_robotic_status_provider_->publishState();
    cmd_mode_ = cmd_mode_ == cav::CommandMode_t::DisableRobotic ? cav::CommandMode_t::DisableRobotic : cav::CommandMode_t::None;
}


void XGVApplication::post_spin()
{
    if(active_robotic_status_provider_->getEnabled())
    {
        switch(cmd_mode_)
        {
            case cav::CommandMode_t::None:
                break;
            case cav::CommandMode_t::Wrench:
                xgv_client_->setWrenchEffort(effort_controller_->command.effort);
                break;
            case cav::CommandMode_t::ClosedLoop:
                xgv_client_->setSpeedAccel(speed_controller_->command.speed,speed_controller_->command.max_accel);
                break;
        }
    }
    else if(cmd_mode_ == cav::CommandMode_t::DisableRobotic)
    {
        xgv_client_->disableRoboticControl();
    }

}


void XGVApplication::shutdown()
{
    xgv_client_->shutdown();
}
