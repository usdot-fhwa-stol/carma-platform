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

        //JOHN rebuild all this
        pnh_->param<double>("timeHeadway", config.timeHeadway, config.timeHeadway);
        pnh_->param<double>("standStillHeadway", config.standStillHeadway, config.standStillHeadway);
        pnh_->param<double>("maxAccel", config.maxAccel, config.maxAccel);
        pnh_->param<double>("Kp", config.Kp, config.Kp);
        pnh_->param<double>("Kd", config.Kd, config.Kd);
        pnh_->param<double>("Ki", config.Ki, config.Ki);
        pnh_->param<double>("maxValue", config.maxValue, config.maxValue);
        pnh_->param<double>("minValue", config.minValue, config.minValue);
        pnh_->param<double>("dt", config.dt, config.dt);
        pnh_->param<double>("adjustmentCap", config.adjustmentCap, config.adjustmentCap);
        pnh_->param<double>("integratorMax", config.integratorMax, config.integratorMax);
        pnh_->param<double>("integratorMin", config.integratorMin, config.integratorMin);
        pnh_->param<double>("Kdd", config.Kdd, config.Kdd);
        pnh_->param<int>("cmdTmestamp", config.cmdTmestamp, config.cmdTmestamp);
        pnh_->param<double>("lowpassGain", config.lowpassGain, config.lowpassGain);
        pnh_->param<double>("lookaheadRatio", config.lookaheadRatio, config.lookaheadRatio);
        pnh_->param<double>("minLookaheadDist", config.minLookaheadDist, config.minLookaheadDist);

        // Global params (from vehicle config)
        pnh_->getParam("/vehicle_id", config.vehicleID); //TODO: need this?
        pnh_->getParam("/vehicle_wheel_base", config.wheelBase); //TODO: need this?
        pnh_->getParam("/control_plugin_shutdown_timeout", config.shutdownTimeout);
        pnh_->getParam("/control_plugin_ignore_initial_inputs", config.ignoreInitialInputs);

        pcw_.updateConfigParams(config_);

	  	// Define the topic subscribers
		trajectory_plan_sub_ = nh_->subscribe<cav_msgs::TrajectoryPlan>("PlatooningControlPlugin/plan_trajectory", 1, &PlatoonControlPlugin::trajectoryPlan_cb, this);
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &PlatoonControlPlugin::currentTwist_cb, this);
        platoon_info_sub_ = nh_->subscribe<cav_msgs::PlatooningInfo>("platoon_info", 1, &PlatoonControlPlugin::platoonInfo_cb, this);
        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlPlugin::pose_cb, this);  //TODO: why this form of subscribe?

		// Define the topic publishers
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_raw", 5, true);
        ctrl_pub_ = nh_->advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 5, true);
        platoon_info_pub_ = nh_->advertise<cav_msgs::PlatooningInfo>("platooning_info", 1, true);
		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);

        // Populate the default content to be published for plugin discovery (most of this will never change)
        plugin_discovery_msg_.name = "PlatooningControlPid0Plugin";
        plugin_discovery_msg_.version_id = "v0.1"; //TODO set this when done testing
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
            ros::Duration(ros::Rate(30.0)),
            [this](const auto&) { control_timer_cb(); }
        );
    }

                                    
    void PlatoonControlPid0Plugin::run(){
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void PlatoonControlPid0Plugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
        pcw_.setCurrentPose(pose_msg_);
    }


    void PlatoonControlPid0Plugin::platoon_info_cb(const cav_msgs::PlatooningInfoConstPtr& msg)
    {
        // Copy the incoming message content to a new message for publishing
        cav_msgs::PlatooningInfo output_msg = *msg;

        // Add speed command to the output message and publish
        platooing_info_msg.host_cmd_speed = pcw_.speedCmd_; // JOHN don't allow direct access
        platoon_info_pub_.publish(output_msg);
 
        // Grab a few items to update our internal knowledge - JOHN fix all these for accessors
        platoon_leader_.staticId = msg->leader_id;
        platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
        platoon_leader_.commandSpeed = msg->leader_cmd_speed;
        // TODO: index is 0 temp to test the leader state
        platoon_leader_.NumberOfVehicleInFront = msg->host_platoon_position;
        platoon_leader_.leaderIndex = 0;

        pcw_.actual_gap_ = platooing_info_msg.actual_gap;
        pcw_.desired_gap_ = platooing_info_msg.desired_gap;
        ROS_DEBUG_STREAM("Platoon leader id:  " << platoon_leader_.staticId);
        ROS_DEBUG_STREAM("Platoon leader pose:  " << platoon_leader_.vehiclePosition);
        ROS_DEBUG_STREAM("Platoon leader cmd speed:  " << platoon_leader_.commandSpeed);
    }


    void PlatoonControlPid0Plugin::current_twist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist){
        pcw_.set_current_speed(twist->twist.linear.x);
    }


    void PlatoonControlPid0Plugin::trajectory_plan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
        
        // Ensure we have enough remaining trajectory to do our work
        if (tp->trajectory_points.size() < 2) {
            ROS_WARN_STREAM("PlatoonControlPid0Plugin cannot execute - insufficient trajectory points provided.");
            return;
        }

        pcw_.set_trajectory(tp);

        // Update timing info
        prev_input_time_ = ros::Time::now().toNSec() / 1000000;
        ++consecutive_input_ctr_;
    }


    bool PlatoonControlPid0Plugin::control_timer_cb(){

        ROS_DEBUG_STREAM("In control timer callback ");
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

        // Generate the control signal, publish outputs & return
        pcw_.generate_control_signal();
        double speed_cmd = pcw_.get_speed_cmd();
        double steer_cmd = pcw_.get_steering_cmd();
        double angle_cmd = pcw_.get_angular_vel_cmd();

        geometry_msgs::TwistStamped twist_msg = composeTwistCmd(speed_cmd, angle_cmd);
        twist_pub_.publish(twist_msg);

        autoware_msgs::ControlCommandStamped ctrl_msg = composeCtrlCmd(speed_cmd, steer_cmd);
        ctrl_pub_.publish(ctrl_msg);
        return true;
    }


    geometry_msgs::TwistStamped PlatoonControlPid0Plugin::compose_twist_cmd(double linear_vel, double angular_vel)
    {
        geometry_msgs::TwistStamped cmd_twist;
        cmd_twist.twist.linear.x = linear_vel;
        cmd_twist.twist.angular.z = angular_vel;
        cmd_twist.header.stamp = ros::Time::now();
        return cmd_twist;
    }


    autoware_msgs::ControlCommandStamped PlatoonControlPid0Plugin::compose_ctrl_cmd(double linear_vel, double steering_angle)
    {
        autoware_msgs::ControlCommandStamped cmd_ctrl;
        cmd_ctrl.header.stamp = ros::Time::now();
        cmd_ctrl.cmd.linear_velocity = linear_vel;
        cmd_ctrl.cmd.steering_angle = steering_angle;
        return cmd_ctrl;
    }


// @SONAR_START@


//JOHN - reorder methods once I determine which ones need to be kept

    void PlatoonControlPid0Plugin::generate_control_signals(const cav_msgs::TrajectoryPlanPoint& first_trajectory_point, 
                                                            const cav_msgs::TrajectoryPlanPoint& lookahead_point)
    {

        pcw_.setCurrentSpeed(trajectory_speed_);
        // pcw_.setCurrentSpeed(current_speed_);
        pcw_.setLeader(platoon_leader_);
    	pcw_.generateSpeed(first_trajectory_point);
    	pcw_.generateSteer(lookahead_point);


    }


    // JOHN not sure I need this one
    // extract maximum speed of trajectory
    double PlatoonControlPid0Plugin::get_trajectory_speed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points)
    {   
        double trajectory_speed = 0;

        double dx1 = trajectory_points[trajectory_points.size()-1].x - trajectory_points[0].x;
        double dy1 = trajectory_points[trajectory_points.size()-1].y - trajectory_points[0].y;
        double d1 = sqrt(dx1*dx1 + dy1*dy1); 
        double t1 = (trajectory_points[trajectory_points.size()-1].target_time.toSec() - trajectory_points[0].target_time.toSec());

        double avg_speed = d1/t1;

        for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
        {            double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
            double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
            double d = sqrt(dx*dx + dy*dy); 
            double t = (trajectory_points[i + 1].target_time.toSec() - trajectory_points[i].target_time.toSec());
            double v = d/t;
            if(v > trajectory_speed)
            {
                trajectory_speed = v;
            }
        }

        ROS_DEBUG_STREAM("trajectory speed: " << trajectory_speed);
        ROS_DEBUG_STREAM("avg trajectory speed: " << avg_speed);

        return avg_speed;
    }
}
