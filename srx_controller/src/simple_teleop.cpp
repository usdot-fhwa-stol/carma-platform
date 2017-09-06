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

#include <cav_srvs/GetLights.h>
#include <cav_srvs/SetLights.h>
#include <cav_srvs/SetEnableRobotic.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32.h>

#include <ros/ros.h>

/**
 * @brief A very simple Teleop controller
 * that uses Joystick to control throttle. This was used to test the efforct control
 */
class TeleopLongitudinalController
{
public:
    TeleopLongitudinalController();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    ros::NodeHandle nh_;

    int accel_, brake_;
    double l_scale_, l_offset_;
    int dead_man_idx_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient client_;

};


TeleopLongitudinalController::TeleopLongitudinalController():
        accel_(5),brake_(2),dead_man_idx_(4), l_scale_(-50.0), l_offset_(1.0)
{

    nh_.param("axis_accel", accel_, accel_);
    nh_.param("axis_brake", brake_, brake_);
    nh_.param("scale_linear", l_scale_, l_scale_);
    nh_.param("deadman_button", dead_man_idx_,dead_man_idx_);


    vel_pub_ = nh_.advertise<std_msgs::Float32>("output", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopLongitudinalController::joyCallback, this);
    client_ = nh_.serviceClient<cav_srvs::SetLights>("control/set_lights");

}

void TeleopLongitudinalController::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    ros::ServiceClient client = nh_.serviceClient<cav_srvs::SetEnableRobotic>("control/enable_robotic");

    std_msgs::Float32 effort;
    if(joy->buttons[dead_man_idx_] == 0)
    {
        cav_srvs::SetEnableRobotic req2;
        req2.request.set = cav_srvs::SetEnableRoboticRequest::DISABLE;
        client.call(req2);

        cav_srvs::SetLights req;
        req.request.set_state.flash = cav_msgs::LightBarStatus::OFF;
        client_.call(req);
    }
    else
    {
        cav_srvs::SetLights req;
        req.request.set_state.flash = cav_msgs::LightBarStatus::ON;
        client_.call(req);

        cav_srvs::SetEnableRobotic req2;
        req2.request.set = cav_srvs::SetEnableRoboticRequest::ENABLE;
        client.call(req2);


        effort.data = 0.0;
        double brake_effort = l_scale_*(joy->axes[brake_] - l_offset_);
        double accel_effort = l_scale_*(joy->axes[accel_] - l_offset_);
        effort.data = static_cast<float>(brake_effort >= 0.0001 ? -brake_effort : accel_effort);
    }

    vel_pub_.publish(effort);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_controller");
    TeleopLongitudinalController controller;

    ros::spin();
}