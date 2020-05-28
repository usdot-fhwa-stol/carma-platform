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

#include <sstream>

#include <ros/ros.h>
#include "collision_checking/collision_checking.cpp"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "collision_checking");
  ros::NodeHandle nodeHandle("~");
  collision_checking::CollisionChecking CollisionChecking(nodeHandle);

  ros::Rate rate(10);

  while (ros::ok())
  {
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
