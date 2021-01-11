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

CarmaRecordNode::CarmaRecordNode(){};

int CarmaRecordNode::run()
{

  // boolean values that will load in from the rosparam server
  //they control if a set of topics will be excluded
  bool exclude_default;
  bool exclude_lidar;
  bool exclude_camera;
  bool exclude_can;

  cnh_.getParam("/exclude_default", exclude_default);
  cnh_.getParam("/exclude_lidar", exclude_lidar);
  cnh_.getParam("/exclude_camera", exclude_camera);
  cnh_.getParam("/exclude_can", exclude_can);

  // param_itr is used to iterate through the parameter list 
  std::vector<std::string>::iterator param_itr;

  // exclude_regex is the final list of topics to be excluded
  std::string exclude_regex = ""; // Add default exclusion to this string

  // if a set of topics needs to be excluded, it will be iterated through and added to the exclude regex
  if(exclude_default){
    std::vector<std::string> excluded_default_topics;
    cnh_.getParam("/excluded_default_topics", excluded_default_topics);

    for (param_itr = excluded_default_topics.begin(); param_itr < excluded_default_topics.end(); param_itr++){
      exclude_regex += ((*param_itr) + "|");
    }
  }

  if(exclude_lidar){
    std::vector<std::string> excluded_lidar_topics;
    cnh_.getParam("/excluded_lidar_topics", excluded_lidar_topics);

    for (param_itr = excluded_lidar_topics.begin(); param_itr < excluded_lidar_topics.end(); param_itr++){
      exclude_regex += ((*param_itr) + "|");
    }
  }

  if(exclude_camera){
    std::vector<std::string> excluded_camera_topics;
    cnh_.getParam("/excluded_camera_topics", excluded_camera_topics);

    for (param_itr = excluded_camera_topics.begin(); param_itr < excluded_camera_topics.end(); param_itr++){
      exclude_regex += ((*param_itr) + "|");
    }
  }

  if(exclude_can){
    std::vector<std::string> excluded_can_topics;
    cnh_.getParam("/excluded_can_topics", excluded_can_topics);

    for (param_itr = excluded_can_topics.begin(); param_itr < excluded_can_topics.end(); param_itr++){
      exclude_regex += ((*param_itr) + "|");
    }
  }

  // remove the final "|" from the regex
  exclude_regex.pop_back();
 
  // set the exclude_regex as a param in the param server
  cnh_.setParam("/exclude_regex", exclude_regex);

  // Spin
  cnh_.spin();
  return 0;
}

}  // namespace carma_record