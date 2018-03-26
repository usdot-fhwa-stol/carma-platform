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

#include "xgv_jaus_client.h"

#include <driver_application/driver_application.h>
#include <cav_driver_utils/longitudinal_effort_controller.h>
#include <cav_driver_utils/longitudinal_speed_controller.h>
#include <cav_driver_utils/enable_robotic_service.h>
#include <cav_driver_utils/active_robotic_status_provider.h>

#include <vector>
#include <string>
#include <memory>
class XGVApplication : public cav::DriverApplication
{
    std::unique_ptr<XGVJausClient> xgv_client_;
    std::unique_ptr<cav::LongitudinalEffortController> effort_controller_;
    std::unique_ptr<cav::LongitudinalSpeedController> speed_controller_;
    std::unique_ptr<cav::EnableRoboticService> enable_robotic_service_;
    std::unique_ptr<cav::ActiveRoboticStatusProvider> active_robotic_status_provider_;
    cav::CommandMode_t cmd_mode_;
    std::vector<std::string> api_;
public:
    XGVApplication(int argc, char **argv);

private:

    inline std::vector<std::string> &get_api() override { return api_; }
    void initialize() override;
    void pre_spin() override;
    void post_spin() override;
    void shutdown() override;
};