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
  double delta_x = tpp.x - pose.pose.position.x;
  double delta_y = tpp.y - pose.pose.position.y;
  double delta_pos = sqrt(delta_x * delta_x + delta_y * delta_y);
  double delta_t_secound = abs((tpp.target_time / 1e9) - current_time);

  if(delta_t_secound != 0) {
    waypoint.twist.twist.linear.x = 3.6 * (delta_pos / delta_t_secound);
  }
  else {
    waypoint.twist.twist.linear.x = 0;
  }

  return waypoint;
};

}
