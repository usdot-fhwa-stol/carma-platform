/*
 * Copyright (C) 2018-2019 LEIDOS.
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

#include "pure_pursuit_wrapper/pure_pursuit_wrapper_worker.hpp"

namespace pure_pursuit_wrapper {

autoware_msgs::Waypoint PurePursuitWrapperWorker::TrajectoryPlanPointToWaypointConverter(double current_time, geometry_msgs::PoseStamped pose, cav_msgs::TrajectoryPlanPoint tpp) {

  autoware_msgs::Waypoint waypoint;

  waypoint.pose.pose.position.x = tpp.x;
  waypoint.pose.pose.position.y = tpp.y;
  double delta_t = (tpp.target_time / 1000) - (current_time);

  waypoint.twist.twist.linear.x = (pose.pose.position.x - tpp.x) / delta_t;
  waypoint.twist.twist.linear.y = (pose.pose.position.y - tpp.y) / delta_t;

  return waypoint;
};

}
