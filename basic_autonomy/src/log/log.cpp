/*
 * Copyright (C) 2021-2022 LEIDOS.
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

#include <basic_autonomy/log/log.hpp>

namespace basic_autonomy
{
namespace log
{
/**
 * \brief Helper function to convert a lanelet::BasicPoint2d to a string
 */ 
std::string basicPointToStream(lanelet::BasicPoint2d point)
{
  std::ostringstream out;
  out << point.x() << ", " << point.y();
  return out.str();
}
/**
 * \brief Helper function to convert a PointSpeedPair to a string
 */ 
std::string pointSpeedPairToStream(waypoint_generation::PointSpeedPair point)
{
  std::ostringstream out;
  out << "Point: " << basicPointToStream(point.point) << " Speed: " << point.speed;
  return out.str();
}

/**
 * \brief Print a ROS_DEBUG_STREAM for each value in values where the printed value is << prefix << value
 */ 
void printDoublesPerLineWithPrefix(const std::string& prefix, const std::vector<double>& values)
{
  for (const auto& value : values)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), prefix << value);
  }
}

}  // namespace log
}  // namespace basic_autonomy