/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <iostream>
#include <carma_wm/CARMAWorldModel.hpp>
#include "TestHelpers.hpp"
#include <carma_wm/WMTestLibForGuidance.hpp>
#include <rclcpp/rclcpp.hpp>
#include <carma_wm/WorldModelUtils.hpp>

namespace carma_wm
{

namespace utils
{

TEST(WorldModelUtilsTest, get32BitId)
{
  uint16_t b1 = 1;
  uint8_t b2 = 1;
  uint32_t b3 = 257;
  EXPECT_EQ(get32BitId(b1,b2), b3);
}

}

}  // namespace carma_wm
