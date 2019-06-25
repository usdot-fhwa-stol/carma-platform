/*
 * Copyright (C) 2018-2019 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License") { you may not
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <cav_msgs/TrajectoryPlan.h>
#include <cav_msgs/SystemAlert.h>

class TrajectoryExecutorTestSuite : public ::testing::Test 
{
    public:
        TrajectoryExecutorTestSuite() {
            _nh = ros::NodeHandle();
            traj_pub = _nh.advertise<cav_msgs::TrajectoryPlan>("/trajectory", 5);
            traj_sub = _nh.subscribe<cav_msgs::TrajectoryPlan>("/carma/guidance/control_plugins/pure_pursuit/trajectory", 100, 
            &TrajectoryExecutorTestSuite::trajEmitCallback, this);
            sys_alert_sub = _nh.subscribe<cav_msgs::SystemAlert>("/system_alert", 100, 
            &TrajectoryExecutorTestSuite::sysAlertCallback, this);
        }
        ros::NodeHandle _nh;
        ros::Publisher traj_pub;
        ros::Subscriber traj_sub;
        ros::Subscriber sys_alert_sub;
        int msg_count = 0;
        int last_points = 9999;
        bool shrinking = true;
        bool recv_sys_alert = false;

        void trajEmitCallback(cav_msgs::TrajectoryPlan msg) {
            msg_count++;
            if (msg.trajectory_points.size() < last_points && shrinking) {
                last_points = msg.trajectory_points.size();
            } else {
                shrinking = false;
            }
        }

        void sysAlertCallback(cav_msgs::SystemAlert msg) {
            recv_sys_alert = true;
        }
};

void waitForSubscribers(ros::Publisher pub, int num_subscribers, int timeout_millis) {
    int elapsed_millis = 0;
    while (pub.getNumSubscribers() < num_subscribers && elapsed_millis < timeout_millis) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        elapsed_millis += 100;
    }

    if (elapsed_millis > timeout_millis && pub.getNumSubscribers() < num_subscribers) {
        ROS_WARN_STREAM("Target number of subscribers for " << pub.getTopic() 
        << " not reached due to timeout! (" << pub.getNumSubscribers() 
        << "/" << num_subscribers << " subscribers)");
    }
}

cav_msgs::TrajectoryPlan buildSampleTraj() {
    cav_msgs::TrajectoryPlan plan;

    plan.header.stamp = ros::Time::now();
    plan.trajectory_id = "TEST TRAJECTORY 1";

    for (int i = 0; i < 10; i++) {
        cav_msgs::TrajectoryPlanPoint p;
        p.controller_plugin_name = "pure_pursuit";
        p.lane_id = "0";
        p.planner_plugin_name = "cruising";
        p.target_time = i * 0.1;
        p.x = 10 * i;
        p.y = 10 * i;
        plan.trajectory_points.push_back(p);
    }

    return plan;
}