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

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pose_to_tf/PoseToTF2.h>
#include <pose_to_tf/PoseToTF2Config.h>
#include <carma_utils/CARMANodeHandle.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_tf");

  ros::CARMANodeHandle nh;
  ros::CARMANodeHandle pnh("~");

  tf2_ros::TransformBroadcaster tf_broadcaster;

  pose_to_tf::PoseToTF2Config config;
  pnh.param<std::string>("child_frame", config.child_frame, config.child_frame);
  pnh.param<std::string>("default_parent_frame", config.default_parent_frame, config.default_parent_frame);

  pose_to_tf::PoseToTF2 worker(config, [&tf_broadcaster](auto msg){tf_broadcaster.sendTransform(msg);});

  ros::Subscriber pose_sub = nh.subscribe("pose_to_tf", 10, &pose_to_tf::PoseToTF2::poseCallback, &worker);
  ros::Subscriber pose_stamped_sub = nh.subscribe("pose_stamped_to_tf", 10, &pose_to_tf::PoseToTF2::poseStampedCallback, &worker);
  ros::Subscriber pose_with_cov_sub = nh.subscribe("pose_with_cov_to_tf", 10, &pose_to_tf::PoseToTF2::poseWithCovarianceCallback, &worker);
  ros::Subscriber pose_with_cov_stamped_sub = nh.subscribe("pose_with_cov_stamped_to_tf", 10, &pose_to_tf::PoseToTF2::poseWithCovarianceStampedCallback, &worker);

  ros::CARMANodeHandle::spin();
  return 0;
};
