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

#include "platoon_control_ihp.h"

namespace platoon_control_ihp
{
// @SONAR_STOP@
    PlatoonControlIHPPlugin::PlatoonControlIHPPlugin()
    {
        pcw_ = PlatoonControlIHPWorker();
    }
    

    void PlatoonControlIHPPlugin::initialize()
    {
    	nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));

        PlatooningControlIHPPluginConfig config;

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
        pnh_->getParam("/vehicle_id", config.vehicleID);
        pnh_->getParam("/vehicle_wheel_base", config.wheelBase);
        pnh_->getParam("/control_plugin_shutdown_timeout", config.shutdownTimeout);
        pnh_->getParam("/control_plugin_ignore_initial_inputs", config.ignoreInitialInputs);

        pcw_.updateConfigParams(config);
        config_ = config;

	  	// Trajectory Plan Subscriber
		trajectory_plan_sub = nh_->subscribe<cav_msgs::TrajectoryPlan>("platoon_control_ihp/plan_trajectory", 1, &PlatoonControlIHPPlugin::trajectoryPlan_cb, this);
        
        // Current Twist Subscriber
        current_twist_sub_ = nh_->subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &PlatoonControlIHPPlugin::currentTwist_cb, this);

        // Platoon Info Subscriber
        // topic name so it is specific to ihp plugins
        platoon_info_sub_ = nh_->subscribe<cav_msgs::PlatooningInfo>("platoon_info_ihp", 1, &PlatoonControlIHPPlugin::platoonInfo_cb, this);

		// Control Publisher
		twist_pub_ = nh_->advertise<geometry_msgs::TwistStamped>("twist_raw", 5, true);
        ctrl_pub_ = nh_->advertise<autoware_msgs::ControlCommandStamped>("ctrl_raw", 5, true);
        platoon_info_pub_ = nh_->advertise<cav_msgs::PlatooningInfo>("platooning_info", 1, true);


        pose_sub_ = nh_->subscribe("current_pose", 1, &PlatoonControlIHPPlugin::pose_cb, this);

		plugin_discovery_pub_ = nh_->advertise<cav_msgs::Plugin>("plugin_discovery", 1);
        plugin_discovery_msg_.name = "platoon_control_ihp";
        plugin_discovery_msg_.version_id = "v1.0";
        plugin_discovery_msg_.available = true;
        plugin_discovery_msg_.activated = true;
        plugin_discovery_msg_.type = cav_msgs::Plugin::CONTROL;
        plugin_discovery_msg_.capability = "control/trajectory_control";


        discovery_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(10.0)),
            [this](const auto&) { plugin_discovery_pub_.publish(plugin_discovery_msg_);
                                  ROS_DEBUG_STREAM("10hz timer callback called");});
        
        ROS_DEBUG_STREAM("discovery timer created");
        
        control_pub_timer_ = pnh_->createTimer(
            ros::Duration(ros::Rate(30.0)),
            [this](const auto&) { ROS_DEBUG_STREAM("30hz timer callback called"); 
                                  controlTimerCb();  }); 
        
        ROS_DEBUG_STREAM("control timer created ");    
    }

                                    
    void PlatoonControlIHPPlugin::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    bool PlatoonControlIHPPlugin::controlTimerCb()
    {
        ROS_DEBUG_STREAM("In control timer callback ");
        // If it has been a long time since input data has arrived then reset the input counter and return
        // Note: this quiets the controller after its input stream stops, which is necessary to allow 
        // the replacement controller to publish on the same output topic after this one is done.
        long current_time = ros::Time::now().toNSec() / 1000000;
        ROS_DEBUG_STREAM("current_time = " << current_time << ", prev_input_time_ = " << prev_input_time_ << ", input counter = " << consecutive_input_counter_);
        if (current_time - prev_input_time_ > config_.shutdownTimeout)
        {
            ROS_DEBUG_STREAM("returning due to timeout.");
            consecutive_input_counter_ = 0;
            return false;
        }

        // If there have not been enough consecutive timely inputs then return (waiting for
        // previous control plugin to time out and stop publishing, since it uses same output topic)
        if (consecutive_input_counter_ <= config_.ignoreInitialInputs)
        {
            ROS_DEBUG_STREAM("returning due to first data input");
            return false;
        }

        cav_msgs::TrajectoryPlanPoint second_trajectory_point = latest_trajectory_.trajectory_points[1]; 
        cav_msgs::TrajectoryPlanPoint lookahead_point = getLookaheadTrajectoryPoint(latest_trajectory_);

        trajectory_speed_ = getTrajectorySpeed(latest_trajectory_.trajectory_points);
        
        generateControlSignals(second_trajectory_point, lookahead_point); 

        return true;
    } 

    void  PlatoonControlIHPPlugin::trajectoryPlan_cb(const cav_msgs::TrajectoryPlan::ConstPtr& tp)
    {
        if (tp->trajectory_points.size() < 2) {
            ROS_WARN_STREAM("PlatoonControlIHPPlugin cannot execute trajectory as only 1 point was provided");
            return;
        }

        latest_trajectory_ = *tp;
        prev_input_time_ = ros::Time::now().toNSec() / 1000000;
        ++consecutive_input_counter_;
        ROS_DEBUG_STREAM("New trajectory plan #" << consecutive_input_counter_ << " at time " << prev_input_time_);
        ROS_DEBUG_STREAM("tp header time =                " << tp->header.stamp.toNSec() / 1000000);
    }

    cav_msgs::TrajectoryPlanPoint PlatoonControlIHPPlugin::getLookaheadTrajectoryPoint(cav_msgs::TrajectoryPlan trajectory_plan)
    {   
        cav_msgs::TrajectoryPlanPoint lookahead_point;

        double lookahead_dist = config_.lookaheadRatio * current_speed_;
        ROS_DEBUG_STREAM("lookahead based on speed: " << lookahead_dist);

        lookahead_dist = std::max(config_.minLookaheadDist, lookahead_dist);
        ROS_DEBUG_STREAM("final lookahead: " << lookahead_dist);
            
        double traveled_dist = 0.0;
        bool found_point = false;

        for (size_t i = 1; i<trajectory_plan.trajectory_points.size() - 1; i++)
        {
            double dx =  pose_msg_.pose.position.x - trajectory_plan.trajectory_points[i].x;
            double dy =  pose_msg_.pose.position.y - trajectory_plan.trajectory_points[i].y;

            double dx1 = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
            double dy1 = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
            double dist1 = std::sqrt(dx1*dx1 + dy1*dy1);  
            ROS_DEBUG_STREAM("trajectory spacing: " << dist1);          

            double dist = std::sqrt(dx*dx + dy*dy);            

            traveled_dist = dist;

            if ((lookahead_dist - traveled_dist) < 1.0)
            {
                lookahead_point =  trajectory_plan.trajectory_points[i];
                found_point = true;
                ROS_DEBUG_STREAM("found lookahead point at index: " << i);
                break;
            }
        }

        if (!found_point)
        {
            lookahead_point = trajectory_plan.trajectory_points.back();
            ROS_DEBUG_STREAM("lookahead point set as the last trajectory point");
        }

        
        return lookahead_point;
    }

    void PlatoonControlIHPPlugin::pose_cb(const geometry_msgs::PoseStampedConstPtr& msg)
    {
        pose_msg_ = geometry_msgs::PoseStamped(*msg.get());
        pcw_.setCurrentPose(pose_msg_);
    }

    void PlatoonControlIHPPlugin::platoonInfo_cb(const cav_msgs::PlatooningInfoConstPtr& msg)
    {
        
        platoon_leader_.staticId = msg->leader_id;
        platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
        platoon_leader_.commandSpeed = msg->leader_cmd_speed;
        // TODO: index is 0 temp to test the leader state
        platoon_leader_.NumberOfVehicleInFront = msg->host_platoon_position;
        platoon_leader_.leaderIndex = 0;

        ROS_DEBUG_STREAM("Platoon leader leader id:  " << platoon_leader_.staticId);
        ROS_DEBUG_STREAM("Platoon leader leader pose:  " << platoon_leader_.vehiclePosition);
        ROS_DEBUG_STREAM("Platoon leader leader cmd speed:  " << platoon_leader_.commandSpeed);

        cav_msgs::PlatooningInfo platooing_info_msg = *msg;
        // platooing_info_msg.desired_gap = pcw_.desired_gap_;
        // platooing_info_msg.actual_gap = pcw_.actual_gap_;
        pcw_.actual_gap_ = platooing_info_msg.actual_gap;
        pcw_.desired_gap_ = platooing_info_msg.desired_gap;

        // UCLA: Read host platoon position 
        pcw_.host_platoon_position_ = platooing_info_msg.host_platoon_position;
        // UCLA: Read the time headway summation of all predecessors   
        pcw_.current_predecessor_time_headway_sum_ = platooing_info_msg.current_predecessor_time_headway_sum;
        // UCLA: Read predecessor speed m/s, and DtD position m.
        pcw_.predecessor_speed_ = platooing_info_msg.predecessor_speed;
        pcw_.predecessor_position_ = platooing_info_msg.predecessor_position;

        platooing_info_msg.host_cmd_speed = pcw_.speedCmd_;
        platoon_info_pub_.publish(platooing_info_msg);
    }


    void PlatoonControlIHPPlugin::currentTwist_cb(const geometry_msgs::TwistStamped::ConstPtr& twist)
    {
        current_speed_ = twist->twist.linear.x;
    }


// @SONAR_START@

    geometry_msgs::TwistStamped PlatoonControlIHPPlugin::composeTwistCmd(double linear_vel, double angular_vel)
    {
        geometry_msgs::TwistStamped cmd_twist;
        cmd_twist.twist.linear.x = linear_vel;
        cmd_twist.twist.angular.z = angular_vel;
        cmd_twist.header.stamp = ros::Time::now();
        return cmd_twist;
    }

    autoware_msgs::ControlCommandStamped PlatoonControlIHPPlugin::composeCtrlCmd(double linear_vel, double steering_angle)
    {
        autoware_msgs::ControlCommandStamped cmd_ctrl;
        cmd_ctrl.header.stamp = ros::Time::now();
        cmd_ctrl.cmd.linear_velocity = linear_vel;
        ROS_DEBUG_STREAM("ctrl command speed " << cmd_ctrl.cmd.linear_velocity);
        cmd_ctrl.cmd.steering_angle = steering_angle;
        ROS_DEBUG_STREAM("ctrl command steering " << cmd_ctrl.cmd.steering_angle);

        return cmd_ctrl;
    }

    void PlatoonControlIHPPlugin::generateControlSignals(const cav_msgs::TrajectoryPlanPoint& first_trajectory_point, const cav_msgs::TrajectoryPlanPoint& lookahead_point)
    {

        // setting a speed baseline according to the trajectory speed profile. PID will calculate additional speed changes in addition to this value. 
        pcw_.setCurrentSpeed(trajectory_speed_); //TODO why this and not the actual vehicle speed?  Method name suggests different use than this.
        // pcw_.setCurrentSpeed(current_speed_);
        pcw_.setLeader(platoon_leader_);
    	pcw_.generateSpeed(first_trajectory_point);
    	pcw_.generateSteer(lookahead_point);


        geometry_msgs::TwistStamped twist_msg = composeTwistCmd(pcw_.speedCmd_, pcw_.angVelCmd_);
        twist_pub_.publish(twist_msg);

        autoware_msgs::ControlCommandStamped ctrl_msg = composeCtrlCmd(pcw_.speedCmd_, pcw_.steerCmd_);
        ctrl_pub_.publish(ctrl_msg);
    }

    // extract maximum speed of trajectory
    double PlatoonControlIHPPlugin::getTrajectorySpeed(std::vector<cav_msgs::TrajectoryPlanPoint> trajectory_points)
    {   
        double trajectory_speed = 0;

        double dx1 = trajectory_points[trajectory_points.size()-1].x - trajectory_points[0].x;
        double dy1 = trajectory_points[trajectory_points.size()-1].y - trajectory_points[0].y;
        double d1 = sqrt(dx1*dx1 + dy1*dy1); 
        double t1 = (trajectory_points[trajectory_points.size()-1].target_time.toSec() - trajectory_points[0].target_time.toSec());

        double avg_speed = d1/t1;
        ROS_DEBUG_STREAM("trajectory_points size = " << trajectory_points.size() << ", d1 = " << d1 << ", t1 = " << t1 << ", avg_speed = " << avg_speed);

        for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
        {            
            double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
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

        return avg_speed; //TODO: why are 2 speeds being calculated? Which should be returned?

    }

}
