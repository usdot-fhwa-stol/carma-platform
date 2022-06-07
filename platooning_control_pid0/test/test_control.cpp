/*------------------------------------------------------------------------------
* Copyright (C) 2020-2021 LEIDOS.
*
* Licensed under the Apache License, Version 2.0 (the "License"); you may not
* use this file except in compliance with the License. You may obtain a copy of
* the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
* License for the specific language governing permissions and limitations under
* the License.
------------------------------------------------------------------------------*/

#include "platoon_control_pid0.h"
#include "platoon_control_config.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(PlatoonControlPluginTest, test_config)
{
    /***
    ros::Time::init();
    platoon_control_pid0::PlatoonControlPid0Plugin pc;
    pc.initialize();
    platoon_control_pid0::PlatoonControlPluginConfig c = pc.get_config();
    EXPECT_NEAR(0.1, c.gamma_h, 0.01);
    EXPECT_NEAR(-1.01, c.pid_h_kp1, 0.01);
    EXPECT_NEAR(0.02, c.pid_c_kp1, 0.01);
    ***/
}
