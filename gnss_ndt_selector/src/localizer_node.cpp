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
	using std::placeholders::_1;
	Localizer::Localizer(){}

	void Localizer::publishTransform(const geometry_msgs::TransformStamped& msg)
	{
		br_.sendTransform(msg);
	}

	void Localizer::publishPoseStamped(const geometry_msgs::PoseStamped& msg)
	{
		pose_pub_.publish(msg);
	}

	void Localizer::run()
	{
		// initialize node handles
		nh_ = ros::CARMANodeHandle();
		pnh_ = ros::CARMANodeHandle("~");
		// get params
		LocalizationManagerConfig config;

		pnh_.param<double>("spin_rate", spin_rate_, 10.0);
		pnh_.param<double>("score_upper_limit", config.score_upper_limit, 2.0);
		pnh_.param<int>("unreliable_message_upper_limit", config.unreliable_message_upper_limit, 3);
		int localization_mode;
		pnh_.param<int>("localization_mode", localization_mode, 0);
		config.localization_mode = static_cast<LocalizerMode>(localization_mode);

		LocalizationManager manager(std::bind(&Localizer::publishPoseStamped, this, _1),
			std::bind(&Localizer::publishTransform, this, _1), config);

		// initialize subscribers
		ndt_pose_sub_ = nh_.subscribe("ndt_pose", 5, &LocalizationManager::ndtPoseCallback, &manager);
		ndt_score_sub_ = nh_.subscribe("ndt_stat", 5, &LocalizationManager::ndtScoreCallback, &manager);
		gnss_pose_sub_ = nh_.subscribe("gnss_pose", 5, &LocalizationManager::gnssPoseCallback, &manager);
		// initialize publishers
		pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("selected_pose", 5);
		// spin
		nh_.setSpinRate(spin_rate_);
		nh_.spin();
	}
}



