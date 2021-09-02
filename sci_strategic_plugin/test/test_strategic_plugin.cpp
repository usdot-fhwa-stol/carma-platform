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

#include <gtest/gtest.h>
#include <ros/console.h>
#include "test_fixture.h"
#include "sci_strategic_plugin.h"
#include "sci_strategic_plugin_config.h"

// Unit tests for strategic plugin helper methods
namespace sci_strategic_plugin
{
TEST_F(SCIStrategicPluginTest, getDiscoveryMsg)
{
  SCIStrategicPluginConfig config;
  config.strategic_plugin_name = "test name";
  SCIStrategicPlugin wzp(cmw_, config);

  auto msg = wzp.getDiscoveryMsg();
  ASSERT_TRUE(msg.name.compare("test name") == 0);
  ASSERT_TRUE(msg.versionId.compare("v1.0") == 0);
  ASSERT_TRUE(msg.available);
  ASSERT_TRUE(msg.activated);
  ASSERT_EQ(msg.type, cav_msgs::Plugin::STRATEGIC);
  ASSERT_TRUE(msg.capability.compare("strategic_plan/plan_maneuvers") == 0);
}
