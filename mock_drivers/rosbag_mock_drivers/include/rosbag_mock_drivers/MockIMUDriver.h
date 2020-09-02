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
#include <sensor_msgs/Imu.h>

namespace mock_drivers
{
class MockIMUDriver : public MockDriver
{
private:
  const std::string raw_data_topic_ = "imu";

protected:
  int onRun();
  
public:
  MockIMUDriver(bool dummy = false);
  ~MockIMUDriver() {};
  std::vector<DriverType> getDriverTypes();
  uint8_t getDriverStatus();
  unsigned int getRate();
};

}  // namespace mock_drivers