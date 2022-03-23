
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

#include "pure_pursuit.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(PurePursuitTest, test_filter)
{
    PlatooningControlPluginConfig config;
	config.lowpassGain = 0.5;
    
    platoon_control::PurePursuit pp;
    pp.config_ = config;
    double new_angle = pp.lowPassfilter(3.0, 0, config.lowpassGain);
    EXPECT_EQ(1.5, new_angle);
}

TEST(PurePursuitTest, test1)
{

    cav_msgs::TrajectoryPlanPoint point;
    point.x = 1.0;
    point.y = 1.0;
    point.target_time = ros::Time(1.0);
    platoon_control::PurePursuit pp;
    pp.calculateSteer(point);
    EXPECT_EQ(0, pp.steering_angle_);


}

