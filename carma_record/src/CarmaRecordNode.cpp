/*
 * Copyright (C) 2020-2021 LEIDOS.
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

#include <carma_record/CarmaRecordNode.h>

namespace carma_record
{

int CarmaRecordNode::run() const
{

  // boolean values that will load in from the rosparam server
  // they control if a set of topics will be excluded
  bool exclude_default;
  bool exclude_lidar;
  bool exclude_camera;
  bool exclude_can;

  cnh_.getParam("/exclude_default", exclude_default);
  cnh_.getParam("/exclude_lidar", exclude_lidar);
  cnh_.getParam("/exclude_camera", exclude_camera);
  cnh_.getParam("/exclude_can", exclude_can);

  // exclude_regex is the final list of topics to be excluded
  std::string exclude_regex = ""; // Add default exclusion to this string

  // if a set of topics needs to be excluded, it will be iterated through and added to the exclude regex
  if(exclude_default){
    std::vector<std::string> excluded_default_topics;
    cnh_.getParam("/excluded_default_topics", excluded_default_topics);

    for (const auto& exclusion_string : excluded_default_topics){
      exclude_regex += exclusion_string + "|";
    }
  }

  if(exclude_lidar){
    std::vector<std::string> excluded_lidar_topics;
    cnh_.getParam("/excluded_lidar_topics", excluded_lidar_topics);

    for (const auto& exclusion_string : excluded_lidar_topics){
      exclude_regex += exclusion_string + "|";
    }
  }

  if(exclude_camera){
    std::vector<std::string> excluded_camera_topics;
    cnh_.getParam("/excluded_camera_topics", excluded_camera_topics);

    for (const auto& exclusion_string : excluded_camera_topics){
      exclude_regex += exclusion_string + "|";
    }
  }

  if(exclude_can){
    std::vector<std::string> excluded_can_topics;
    cnh_.getParam("/excluded_can_topics", excluded_can_topics);

    for (const auto& exclusion_string : excluded_can_topics){
      exclude_regex += exclusion_string + "|";
    }
  }

  // remove the final "|" from the regex
  exclude_regex.pop_back();
 
  // set the exclude_regex as a param in the param server
  cnh_.setParam("exclude_regex", exclude_regex);

  // if no topics are being excluded, set the no_exclusions parameter so the record script will still run
  if(!(exclude_default || exclude_lidar || exclude_camera || exclude_can)){
    cnh_.setParam("no_exclusions", true);
  } else {
    cnh_.setParam("no_exclusions", false);
  }
  // No need to spin here, but it prevents ros1_bridge from continuously 
  // printing error logs about failing to connect
  // to this node's services indefinitely which is confusing to many users
  ros::spin();

  return 0;
}

}  // namespace carma_record
