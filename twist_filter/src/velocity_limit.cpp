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

#include "velocity_limit.hpp"

namespace twist_filter{
geometry_msgs::msg::TwistStamped
  longitudinalLimitTwist(const geometry_msgs::msg::TwistStamped& msg, const double limit)
{
  geometry_msgs::msg::TwistStamped ts;
  ts = msg;

  auto orig_longitudinal_velocity = msg.twist.linear.x;
  auto longitudinal_velocity = msg.twist.linear.x;

  if (longitudinal_velocity > limit) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("twist_filter::TwistFilter"), "Longitudinal velocity of " << 
      orig_longitudinal_velocity << 
      " exceeds configured limit of " <<
      limit);
    longitudinal_velocity = limit;
  }
  if (longitudinal_velocity > MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("twist_filter::TwistFilter"), "Longitudinal velocity of " << 
      orig_longitudinal_velocity << 
      " exceeds hard-coded limit of " <<
      MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S);
    longitudinal_velocity = MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S;
  }
  
  ts.twist.linear.x = longitudinal_velocity;
  return ts;
}

autoware_msgs::msg::ControlCommandStamped
  longitudinalLimitCtrl(const autoware_msgs::msg::ControlCommandStamped& msg, const double limit)
{
  autoware_msgs::msg::ControlCommandStamped ccs;
  ccs = msg;

  auto orig_longitudinal_velocity = msg.cmd.linear_velocity;
  auto longitudinal_velocity = msg.cmd.linear_velocity;

  if (longitudinal_velocity > limit) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("twist_filter::TwistFilter"),"Longitudinal velocity of " << 
      orig_longitudinal_velocity << 
      " exceeds configured limit of " <<
      limit);
    longitudinal_velocity = limit;
  }
  if (longitudinal_velocity > MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("twist_filter::TwistFilter"), "Longitudinal velocity of " << 
      orig_longitudinal_velocity << 
      " exceeds hard-coded limit of " <<
      MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S);
    longitudinal_velocity = MAX_LONGITUDINAL_VELOCITY_HARDCODED_LIMIT_M_S;
  }
  
  ccs.cmd.linear_velocity = longitudinal_velocity;
  return ccs;
}

}