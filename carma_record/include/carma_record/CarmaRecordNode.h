#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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

#include <functional>
#include <carma_utils/CARMAUtils.h>
#include <ros/ros.h>

namespace carma_record
{
/*!
 * \brief Node which manages rosbag record with additional carma specific functionality
 *
 * The CarmaRecordNode handles communicating with the ROS network, loading in configuration parameters, and 
 * excluding record topics that are not desired.
 *
 */
class CarmaRecordNode
{
public:
  /**
   * @brief Constructor
   */
  CarmaRecordNode()=default;

  /**
   * @brief Starts the Node
   *
   * @return 0 on exit with no errors
   */
  int run() const;
  
private:

  ros::CARMANodeHandle cnh_;
};
}  // namespace carma_record
