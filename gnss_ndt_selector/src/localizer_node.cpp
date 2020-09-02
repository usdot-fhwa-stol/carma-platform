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

namespace localizer
{
	Localizer::Localizer(){}

	void Localizer::publishTransform(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = msg->header.stamp;
		transformStamped.header.frame_id = "map";
		transformStamped.child_frame_id = "base_link";
		transformStamped.transform.translation.x = msg->pose.position.x;
		transformStamped.transform.translation.y = msg->pose.position.y;
		transformStamped.transform.translation.z = msg->pose.position.z;
		transformStamped.transform.rotation.x = msg->pose.orientation.x;
		transformStamped.transform.rotation.y = msg->pose.orientation.y;
		transformStamped.transform.rotation.z = msg->pose.orientation.z;
		transformStamped.transform.rotation.w = msg->pose.orientation.w;
		br_.sendTransform(transformStamped);
	}

    void Localizer::publishPoseStamped(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		pose_pub_.publish(msg);
		publishTransform(msg);
	}

    void Localizer::ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
	{
        if(localization_mode_ == LocalizerMode::NDT)
        {
            publishPoseStamped(msg);
        } else if(localization_mode_ == LocalizerMode::AUTO &&
					counter.getNDTReliabilityCounter() <= unreliable_message_upper_limit_)
        {
			publishPoseStamped(msg);
        }
	}
    
    void Localizer::ndtScoreCallback(const autoware_msgs::NDTStatConstPtr& msg)
	{
		counter.onNDTScore(msg->score);
	}

	void Localizer::gnssPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
	{
		if(localization_mode_ == LocalizerMode::GNSS)
		{
			publishPoseStamped(msg);
		} else if(localization_mode_ == LocalizerMode::AUTO &&
					counter.getNDTReliabilityCounter() > unreliable_message_upper_limit_)
		{
			publishPoseStamped(msg);
		}
	}

	void Localizer::run()
	{
		// initialize node handles
		nh_.reset(new ros::CARMANodeHandle());
		pnh_.reset(new ros::CARMANodeHandle("~"));
		// get params
		pnh_->param<double>("spin_rate", spin_rate_, 10.0);
		pnh_->param<double>("score_upper_limit", score_upper_limit_, 2.0);
		pnh_->param<int>("unreliable_message_upper_limit", unreliable_message_upper_limit_, 3);
		pnh_->param<int>("localization_mode", localization_mode_, 0);
		// initialize counter
		counter = NDTReliabilityCounter(score_upper_limit_, unreliable_message_upper_limit_);
		// initialize subscribers
		ndt_pose_sub_ = nh_->subscribe("ndt_pose", 5, &Localizer::ndtPoseCallback, this);
		ndt_score_sub_ = nh_->subscribe("ndt_stat", 5, &Localizer::ndtScoreCallback, this);
		gnss_pose_sub_ = nh_->subscribe("gnss_pose", 5, &Localizer::gnssPoseCallback, this);
		// initialize publishers
		pose_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("selected_pose", 5);
        // spin
        nh_->setSpinRate(spin_rate_);
        nh_->spin();
	}
}



