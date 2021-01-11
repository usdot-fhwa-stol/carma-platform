/*
 * Copyright (C) 2019-2020 LEIDOS.
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
 */


#include "localizer.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
namespace localizer{


TEST(GnssNdtSelectorTest, testTimeOuts)
{
    
    localizer::Localizer node;
    node.init();
    bool gnss_operational = true, ndt_operational = true, gnss_initialized = true, ndt_initialized = true;

    // if both sensors not initialized, nothing should happen
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_FALSE(gnss_operational);
    ASSERT_FALSE(ndt_operational);
    ASSERT_FALSE(gnss_initialized);
    ASSERT_FALSE(ndt_initialized);

    geometry_msgs::PoseStamped gnss_msg;
    gnss_msg.pose.position.x = 1;
    geometry_msgs::PoseStampedConstPtr msg_ptr = boost::make_shared<const geometry_msgs::PoseStamped>(gnss_msg);
    ros::Time::setNow(ros::Time(0));
    node.gnssPoseCallback(msg_ptr);
    // Since gnss is operational now, we can check operation before timeout
    ros::Time::setNow(ros::Time(1));
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_TRUE(gnss_operational);
    ASSERT_FALSE(ndt_operational);
    ASSERT_TRUE(gnss_initialized);
    ASSERT_FALSE(ndt_initialized);

    ROS_ERROR_STREAM("Below three error messages are expected");
    // Now we check if timeout works
    ros::Time::setNow(ros::Time(3));
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_FALSE(gnss_operational);
    ASSERT_FALSE(ndt_operational);
    ASSERT_TRUE(gnss_initialized);
    ASSERT_FALSE(ndt_initialized);

    ros::Time::setNow(ros::Time(4));
    
    node.ndtPoseCallback(msg_ptr);
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_FALSE(gnss_operational);
    ASSERT_TRUE(ndt_operational);
    ASSERT_TRUE(gnss_initialized);
    ASSERT_TRUE(ndt_initialized);

    ros::Time::setNow(ros::Time(4.5));
    node.gnssPoseCallback(msg_ptr);  
    node.spinCallback();

    ros::Time::setNow(ros::Time(5.9));
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_TRUE(gnss_operational);
    ASSERT_FALSE(ndt_operational);
    ASSERT_TRUE(gnss_initialized);
    ASSERT_TRUE(ndt_initialized);

    ros::Time::setNow(ros::Time(6.5));
    node.spinCallback();
    node.reportStatus(gnss_operational, ndt_operational, gnss_initialized, ndt_initialized);
    ASSERT_FALSE(gnss_operational);
    ASSERT_FALSE(ndt_operational);
    ASSERT_TRUE(gnss_initialized);
    ASSERT_TRUE(ndt_initialized);

}
}


/*!
 * \brief Main entrypoint for unit tests
 */
int main (int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "gnss_ndt_selector_node");

    std::thread spinner([] {while (ros::ok()) ros::spin();});

    auto res = RUN_ALL_TESTS();

    ros::shutdown();

    return res;
}