/*
 * Copyright (C) 2019-2021 LEIDOS.
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

#include "platoon_control_pid0.h"

namespace platoon_control_pid0
{
// @SONAR_STOP@
    PlatoonControlPid0Plugin::PlatoonControlPid0Plugin() {}
    

    void PlatoonControlPid0Plugin::initialize() {

    	nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        ROS_DEBUG_STREAM("Attempting to initialize configs from file - * indicates failed items:");
        if (!pnh_->param<double>("pid_h_deadband",       config_.pid_h_deadband,     config_.pid_h_deadband))   ROS_DEBUG_STREAM("*pid_h_deadband");
        if (!pnh_->param<double>("pid_h_slope_break",    config_.pid_h_slope_break,  config_.pid_h_slope_break)) ROS_DEBUG_STREAM("*pid_h_slope_break");
        if (!pnh_->param<double>("pid_h_kp1",            config_.pid_h_kp1,          config_.pid_h_kp1))        ROS_DEBUG_STREAM("*pid_h_kp1");
        if (!pnh_->param<double>("pid_h_kp2",            config_.pid_h_kp2,          config_.pid_h_kp2))        ROS_DEBUG_STREAM("*pid_h_kp2");
        pnh_->param<double>("pid_h_ki",             config_.pid_h_ki,           config_.pid_h_ki);
        pnh_->param<double>("pid_h_kd",             config_.pid_h_kd,           config_.pid_h_kd);
        pnh_->param<double>("pid_h_integral_min",   config_.pid_h_integral_min, config_.pid_h_integral_min);
        pnh_->param<double>("pid_h_integral_max",   config_.pid_h_integral_max, config_.pid_h_integral_max);
        pnh_->param<double>("pid_c_deadband",       config_.pid_c_deadband,     config_.pid_c_deadband);
        pnh_->param<double>("pid_c_slope_break",    config_.pid_c_slope_break,  config_.pid_c_slope_break);
        pnh_->param<double>("pid_c_kp1",            config_.pid_c_kp1,          config_.pid_c_kp1);
        pnh_->param<double>("pid_c_kp2",            config_.pid_c_kp2,          config_.pid_c_kp2);
        pnh_->param<double>("pid_c_ki",             config_.pid_c_ki,           config_.pid_c_ki);
        pnh_->param<double>("pid_c_kd",             config_.pid_c_kd,           config_.pid_c_kd);
        pnh_->param<double>("pid_c_integral_min",   config_.pid_c_integral_min, config_.pid_c_integral_min);
        pnh_->param<double>("pid_c_integral_max",   config_.pid_c_integral_max, config_.pid_c_integral_max);
        pnh_->param<double>("time_step",            config_.time_step,          config_.time_step);
        pnh_->param<double>("gamma_h",              config_.gamma_h,            config_.gamma_h);
        pnh_->param<double>("max_steering_angle",   config_.max_steering_angle, config_.max_steering_angle);
        pnh_->param<double>("max_accel",            config_.max_accel,          config_.max_accel);
        pnh_->param<double>("speed_adjustment_cap", config_.speed_adjustment_cap, config_.speed_adjustment_cap);
        ROS_DEBUG_STREAM("Done setting package-specific config params.");

        // Global params (from vehicle config)
        pnh_->getParam("/vehicle_id", config_.vehicle_id);
        pnh_->getParam("/vehicle_wheel_base", config_.wheelbase);
        pnh_->getParam("/control_plugin_shutdown_timeout", config_.shutdown_timeout);
        pnh_->getParam("/control_plugin_ignore_initial_inputs", config_.ignore_initial_inputs);
        //ROS_DEBUG_STREAM("Configuration settings:\n" << config_);
        ROS_DEBUG_STREAM("Config pid_h_kp1 = " << config_.pid_h_kp1 << ", pid_h_kd = " << config_.pid_h_kd);

        pcw_.set_config_params(config_);

	  	// Define the topic subscribers
		trajectory_plan_sub_ = nh_->subscribe<cav_msgs::TrajectoryPlan>("PlatooningControlPlugin/plan_trajectory", 1, 
                                                                        &PlatoonControlPid0Plugin::trajectory_plan_cb, this);
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &PlatoonControlPid0Plugin::current_twist_cb, this);
        platoon_info_sub_ = nh_->subscribe<cav_msgs::PlatooningInfo>("platoon_info", 1, &PlatoonControlPid0Plugin::platoon_info_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlPid0Plugin::pose_cb, this);  //TODO: why this form of subscribe?

		// Define the topic publishers
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_raw", 5, true);
        ctrl_pub_ = nh_->advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 5, true);
        platoon_info_pub_ = nh_->advertise<cav_msgs::PlatooningInfo>("platooning_info", 1, true);
		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);

        // Populate the default content to be published for plugin discovery (most of this will never change)
        plugin_discovery_msg_.name = "PlatooningControlPid0Plugin";
        plugin_discovery_msg_.version_id = "v0.1"; //TODO set this when done testing; sync with package.xml
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
        plugin_discovery_msg_.capability = "control/trajectory_control";

        // Set up the discovery info to be published at 10 Hz
        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { plugin_discovery_pub_.publish(plugin_discovery_msg_); }
        );

        // Set up a timer to run the control loop at 30 Hz
        control_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(30.0)), //CAUTION: config param time_step must change if this value changes
            [this](const auto&) { control_timer_cb(); }
        );
        
        ROS_INFO_STREAM("///// PlatoonControlPid0Plugin initialized.");
    }

                                    
    void PlatoonControlPid0Plugin::run() {
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void PlatoonControlPid0Plugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg) {
        geometry_msgs::PoseStamped pose_msg = geometry_msgs::PoseStamped(*msg.get());
        pcw_.set_current_pose(pose_msg);
    }


    void PlatoonControlPid0Plugin::platoon_info_cb(const cav_msgs::PlatooningInfoConstPtr& msg) {
        // Copy the incoming message content to a new message for publishing
        cav_msgs::PlatooningInfo output_msg = *msg;

        // Add speed command to the output message and publish
        output_msg.host_cmd_speed = pcw_.get_speed_cmd();
        platoon_info_pub_.publish(output_msg);
 
        // Grab a few items to update our internal knowledge
        DynamicLeaderInfo dl;
        dl.staticId = msg->leader_id;
        dl.vehiclePosition = msg->leader_downtrack_distance;
        dl.commandSpeed = msg->leader_cmd_speed;
        pcw_.set_lead_info(dl, msg->desired_gap, msg->actual_gap);
        ROS_DEBUG_STREAM("leader id:  " << dl.staticId);
        ROS_DEBUG_STREAM("leader pose:  " << dl.vehiclePosition);
        ROS_DEBUG_STREAM("leader cmd speed:  " << dl.commandSpeed);
    }


    void PlatoonControlPid0Plugin::current_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist) {
        pcw_.set_current_speed(twist->twist.linear.x);
    }


    void PlatoonControlPid0Plugin::trajectory_plan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp) {
        
        // Ensure we have enough remaining trajectory to do our work
        if (tp->trajectory_points.size() < 2) {
            ROS_WARN_STREAM("PlatoonControlPid0Plugin cannot execute - insufficient trajectory points provided.");
            return;
        }

        pcw_.set_trajectory(tp);

        // Update timing info
        prev_input_time_ = ros::Time::now().toNSec() / 1000000;
        ++consecutive_input_ctr_;
        ROS_DEBUG_STREAM("Trajectory " << consecutive_input_ctr_ << " received.");
    }


    bool PlatoonControlPid0Plugin::control_timer_cb() {

        // If it has been a long time since input data has arrived then reset the input counter and return
        // Note: this quiets the controller after its input stream stops, which is necessary to allow 
        // the replacement controller to publish on the same output topic after this one is done.
        long current_time = ros::Time::now().toNSec() / 1000000;
        ROS_DEBUG_STREAM("current_time = " << current_time << ", prev_input_time_ = " << prev_input_time_ << ", input counter = " << consecutive_input_ctr_);
        if (current_time - prev_input_time_ > config_.shutdown_timeout)
        {
            ROS_DEBUG_STREAM("returning due to timeout");
            consecutive_input_ctr_ = 0;
            return false;
        }

        // If there have not been enough consecutive timely inputs then return (waiting for
        // previous control plugin to time out and stop publishing, since it uses same output topic)
        if (consecutive_input_ctr_ <= config_.ignore_initial_inputs)
        {
            ROS_DEBUG_STREAM("returning due to first data input");
            return false;
        }

        // Generate the control signal, publish outputs
        pcw_.generate_control_signal();
        double speed_cmd = pcw_.get_speed_cmd();
        double steer_cmd = pcw_.get_steering_cmd();
        double angle_cmd = pcw_.get_angular_vel_cmd();

        geometry_msgs::TwistStamped twist_msg = compose_twist_cmd(speed_cmd, angle_cmd);
        twist_pub_.publish(twist_msg);

        autoware_msgs::ControlCommandStamped ctrl_msg = compose_ctrl_cmd(speed_cmd, steer_cmd);
        ctrl_pub_.publish(ctrl_msg);
        return true;
    }


    PlatoonControlPluginConfig PlatoonControlPid0Plugin::get_config() const {
        return config_;
    }


    geometry_msgs::TwistStamped PlatoonControlPid0Plugin::compose_twist_cmd(double linear_vel, double angular_vel) {
        geometry_msgs::TwistStamped cmd_twist;
        cmd_twist.header.stamp = ros::Time::now();
        cmd_twist.twist.linear.x = linear_vel;
        cmd_twist.twist.angular.z = angular_vel;
        return cmd_twist;
    }


    autoware_msgs::ControlCommandStamped PlatoonControlPid0Plugin::compose_ctrl_cmd(double linear_vel, double steering_angle) {
        autoware_msgs::ControlCommandStamped cmd_ctrl;
        cmd_ctrl.header.stamp = ros::Time::now();
        cmd_ctrl.cmd.linear_velocity = linear_vel;
        cmd_ctrl.cmd.steering_angle = steering_angle;
        return cmd_ctrl;
    }


// @SONAR_START@
}
