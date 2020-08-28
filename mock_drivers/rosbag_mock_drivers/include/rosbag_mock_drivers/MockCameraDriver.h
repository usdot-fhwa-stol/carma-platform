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

#pragma once

#include "rosbag_mock_drivers/MockDriver.h"
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <autoware_msgs/ProjectionMatrix.h>

namespace mock_drivers
{
class MockCameraDriver : public MockDriver
{
private:
  const std::string camera_info_topic_ = "camera/camera_info";
  const std::string image_raw_topic_ = "camera/image_raw";
  const std::string image_rects_topic_ = "camera/image_rect";
  const std::string projection_matrix_topic_ = "camera/projection_matrix";

public:
  MockCameraDriver(bool dummy = false);
  int run();
  bool driverDiscovery();
};

}  // namespace mock_drivers