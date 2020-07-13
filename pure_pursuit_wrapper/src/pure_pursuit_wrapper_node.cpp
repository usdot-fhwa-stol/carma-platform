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
#include "pure_pursuit_wrapper/pure_pursuit_wrapper.hpp"

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "pure_pursuit_wrapper_node");
  ros::NodeHandle nh("");

  pure_pursuit_wrapper::PurePursuitWrapper PurePursuitWrapper(nh);

  // Approximate time of 100ms used because NDT outputs at 10Hz
  message_filters::Synchronizer<pure_pursuit_wrapper::PurePursuitWrapper::SyncPolicy> sync(pure_pursuit_wrapper::PurePursuitWrapper::SyncPolicy(100), PurePursuitWrapper.pose_sub, PurePursuitWrapper.trajectory_plan_sub);
  sync.registerCallback(boost::bind(&pure_pursuit_wrapper::PurePursuitWrapper::TrajectoryPlanPoseHandler, &PurePursuitWrapper, _1, _2));

  while (ros::ok() && !PurePursuitWrapper.shutting_down_) {
    ros::spinOnce();
  }

  ROS_INFO("Successfully launched node.");
  return 0;
}