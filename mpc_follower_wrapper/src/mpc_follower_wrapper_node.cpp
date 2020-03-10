/*
 * Copyright (C) 2018-2020 LEIDOS.
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
#include "mpc_follower_wrapper/mpc_follower_wrapper.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "mpc_follower_wrapper_node");
  ros::NodeHandle nh("");

  mpc_follower_wrapper::MPCFollowerWrapper MPCFollowerWrapper(nh);

  // Approximate time of 100ms used because NDT outputs at 10Hz
  message_filters::Synchronizer<mpc_follower_wrapper::MPCFollowerWrapper::SyncPolicy> sync(mpc_follower_wrapper::MPCFollowerWrapper::SyncPolicy(100), MPCFollowerWrapper.pose_sub, MPCFollowerWrapper.trajectory_plan_sub);
  sync.registerCallback(boost::bind(&mpc_follower_wrapper::MPCFollowerWrapper::TrajectoryPlanPoseHandler, &MPCFollowerWrapper, _1, _2));

  while (ros::ok() && !MPCFollowerWrapper.shutting_down_) {
    ros::spinOnce();
  }

  ROS_INFO("Successfully launched node.");
  return 0;
}