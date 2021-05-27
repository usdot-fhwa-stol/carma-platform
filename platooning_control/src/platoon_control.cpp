
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

#include "platoon_control.hpp"

namespace platoon_control
{
// @SONAR_STOP@
    PlatoonControlPlugin::PlatoonControlPlugin()
    {
        pcw_ = PlatoonControlWorker();
    }
    

    void PlatoonControlPlugin::initialize(){

    	nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        PlatooningControlPluginConfig config;

        pnh_->param<double>("timeHeadway", config.timeHeadway, config.timeHeadway);
        pnh_->param<double>("standStillHeadway", config.standStillHeadway, config.standStillHeadway);
        pnh_->param<double>("maxAccel", config.maxAccel, config.maxAccel);
        pnh_->param<double>("Kp", config.Kp, config.Kp);
        pnh_->param<double>("Kd", config.Kd, config.Kd);
        pnh_->param<double>("Ki", config.Ki, config.Ki);
        pnh_->param<double>("max_value", config.max_value, config.max_value);
        pnh_->param<double>("min_value", config.min_value, config.min_value);
        pnh_->param<double>("dt", config.dt, config.dt);
        pnh_->param<double>("adjustmentCap", config.adjustmentCap, config.adjustmentCap);
        pnh_->param<double>("integratorMax", config.integratorMax, config.integratorMax);
        pnh_->param<double>("integratorMin", config.integratorMin, config.integratorMin);
        pnh_->param<double>("Kdd", config.Kdd, config.Kdd);
        pnh_->param<int>("CMD_TIMESTEP", config.CMD_TIMESTEP, config.CMD_TIMESTEP);
        pnh_->param<double>("wheelbase", config.wheelbase, config.wheelbase);

        pcw_.updateConfigParams(config);

	  	// Trajectory Plan Subscriber
		trajectory_plan_sub = nh_->subscribe<cav_msgs::TrajectoryPlan>("PlatooningControlPlugin/plan_trajectory", 1, &PlatoonControlPlugin::trajectoryPlan_cb, this);
        
        // Current Twist Subscriber
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &PlatoonControlPlugin::currentTwist_cb, this);

        // Platoon Info Subscriber
        platoon_info_sub_ = nh_->subscribe<cav_msgs::PlatooningInfo>("platoon_info", 1, &PlatoonControlPlugin::platoonInfo_cb, this);

		// Control Publisher
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_raw", 10, true);
        ctrl_pub_ = nh_->advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 10, true);


        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlPlugin::pose_cb, this);

        // pcw_.setInitialPose(pose_msg_);
		

		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "PlatooningControlPlugin";
        plugin_discovery_msg_.versionId = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
        plugin_discovery_msg_.capability = "control/trajectory_control";


        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { plugin_discovery_pub_.publish(plugin_discovery_msg_); });
    }

                                    
    void PlatoonControlPlugin::run(){
        initialize();
        ros::CARMANodeHandle::spin();
    }


    void  PlatoonControlPlugin::trajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp){
        
        if (!initial_pose_set_)
        {
            pcw_.setInitialPose(pose_msg_);
            ROS_DEBUG_STREAM("initial pose set");
            initial_pose_set_ = true;
        }

        cav_msgs::TrajectoryPlanPoint t1 = tp->trajectory_points[1];
        cav_msgs::TrajectoryPlanPoint t2 = tp->trajectory_points[3];

    	geometry_msgs::TwistStamped twist_msg = composeTwist(t1, t2);

    	publishTwist(twist_msg);

        autoware_msgs::ControlCommandStamped ctrl_msg;
        ctrl_msg.cmd.linear_velocity = twist_msg.twist.linear.x;
        ROS_DEBUG_STREAM("command speed " << ctrl_msg.cmd.linear_velocity);
        ctrl_msg.cmd.steering_angle = twist_msg.twist.angular.z;// * 180/M_PI;
        ROS_DEBUG_STREAM("command steering " << ctrl_msg.cmd.steering_angle);
        ctrl_pub_.publish(ctrl_msg);

    }

    void PlatoonControlPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
        pcw_.setCurrentPose(pose_msg_);
    }

    void PlatoonControlPlugin::platoonInfo_cb(const cav_msgs::PlatooningInfoConstPtr& msg)
    {
        
        platoon_leader_.staticId = msg->leader_id;
        platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
        platoon_leader_.commandSpeed = msg->leader_cmd_speed;
        // TODO: index is 0 temp to test the leader state
        platoon_leader_.NumberOfVehicleInFront = 0;
        platoon_leader_.leaderIndex = 0;

        ROS_DEBUG_STREAM("Platoon leader leader id:  " << platoon_leader_.staticId);
        ROS_DEBUG_STREAM("Platoon leader leader pose:  " << platoon_leader_.vehiclePosition);
        ROS_DEBUG_STREAM("Platoon leader leader cmd speed:  " << platoon_leader_.commandSpeed);
    }


    void PlatoonControlPlugin::currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist){
        current_speed_ = twist->twist.linear.x;
    }

    void PlatoonControlPlugin::publishTwist(const geometry_msgs::TwistStamped& twist) const {
    	twist_pub_.publish(twist);
    }

// @SONAR_START@
    geometry_msgs::TwistStamped PlatoonControlPlugin::composeTwist(const cav_msgs::TrajectoryPlanPoint& point0, const cav_msgs::TrajectoryPlanPoint& point_end){
    	geometry_msgs::TwistStamped current_twist;
        pcw_.setCurrentSpeed(current_speed_);
        pcw_.setLeader(platoon_leader_);
        // pcw_.setCurrentPose(pose_msg_);
    	pcw_.generateSpeed(point0);
    	pcw_.generateSteer(point_end);
    	current_twist.twist.linear.x = pcw_.speedCmd_;
        ROS_DEBUG_STREAM("desired speed:  " << pcw_.speedCmd_);
    	current_twist.twist.angular.z = pcw_.steerCmd_;
        ROS_DEBUG_STREAM("desired steering:  " << pcw_.steerCmd_);
        current_twist.header.stamp = ros::Time::now();
    	return current_twist;
    }

}
