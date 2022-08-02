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
#include <sstream>
#include "localization_manager/LocalizationTypes.hpp"

namespace localization_manager
{

TEST(LocalizationState, testStream)
{
  std::stringstream output;
  output << LocalizationState::UNINITIALIZED;
  ASSERT_EQ("UNINITIALIZED", output.str());
  output.str(std::string());

  output << LocalizationState::INITIALIZING;
  ASSERT_EQ("INITIALIZING", output.str());
  output.str(std::string());

  output << LocalizationState::OPERATIONAL;
  ASSERT_EQ("OPERATIONAL", output.str());
  output.str(std::string());

  output << LocalizationState::DEGRADED;
  ASSERT_EQ("DEGRADED", output.str());
  output.str(std::string());

  output << LocalizationState::DEGRADED_NO_LIDAR_FIX;
  ASSERT_EQ("DEGRADED_NO_LIDAR_FIX", output.str());
  output.str(std::string());

  output << LocalizationState::AWAIT_MANUAL_INITIALIZATION;
  ASSERT_EQ("AWAIT_MANUAL_INITIALIZATION", output.str());
  output.str(std::string());
}

TEST(LocalizationState, testStateToMsg)
{
  carma_localization_msgs::msg::LocalizationStatusReport msg;
  rclcpp::Time stamp = rclcpp::Time(1.0,0, RCL_SYSTEM_TIME);
  LocalizationState state = LocalizationState::UNINITIALIZED;

  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::UNINITIALIZED);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());

  state = LocalizationState::INITIALIZING;
  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::INITIALIZING);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());

  state = LocalizationState::OPERATIONAL;
  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::OPERATIONAL);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());

  state = LocalizationState::DEGRADED;
  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());

  state = LocalizationState::DEGRADED_NO_LIDAR_FIX;
  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::DEGRADED_NO_LIDAR_FIX);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());

  state = LocalizationState::AWAIT_MANUAL_INITIALIZATION;
  msg = stateToMsg(state, stamp);
  ASSERT_EQ(msg.status, carma_localization_msgs::msg::LocalizationStatusReport::AWAIT_MANUAL_INITIALIZATION);
  ASSERT_EQ(msg.header.stamp.sec, stamp.seconds());
}

TEST(LocalizationSignal, testStream)
{
  std::stringstream output;
  output << LocalizationSignal::INITIAL_POSE;
  ASSERT_EQ("INITIAL_POSE", output.str());
  output.str(std::string());

  output << LocalizationSignal::GOOD_NDT_FREQ_AND_FITNESS_SCORE;
  ASSERT_EQ("GOOD_NDT_FREQ_AND_FITNESS_SCORE", output.str());
  output.str(std::string());

  output << LocalizationSignal::POOR_NDT_FREQ_OR_FITNESS_SCORE;
  ASSERT_EQ("POOR_NDT_FREQ_OR_FITNESS_SCORE", output.str());
  output.str(std::string());

  output << LocalizationSignal::UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE;
  ASSERT_EQ("UNUSABLE_NDT_FREQ_OR_FITNESS_SCORE", output.str());
  output.str(std::string());

  output << LocalizationSignal::TIMEOUT;
  ASSERT_EQ("TIMEOUT", output.str());
  output.str(std::string());

  output << LocalizationSignal::LIDAR_SENSOR_FAILURE;
  ASSERT_EQ("LIDAR_SENSOR_FAILURE", output.str());
  output.str(std::string());
}

TEST(LocalizerMode, testStream)
{
  std::stringstream output;
  output << LocalizerMode::GNSS;
  ASSERT_EQ("GNSS", output.str());
  output.str(std::string());

  output << LocalizerMode::NDT;
  ASSERT_EQ("NDT", output.str());
  output.str(std::string());

  output << LocalizerMode::AUTO_WITH_TIMEOUT;
  ASSERT_EQ("AUTO_WITH_TIMEOUT", output.str());
  output.str(std::string());

  output << LocalizerMode::AUTO_WITHOUT_TIMEOUT;
  ASSERT_EQ("AUTO_WITHOUT_TIMEOUT", output.str());
  output.str(std::string());
}


}
