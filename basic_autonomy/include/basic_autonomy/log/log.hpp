/*------------------------------------------------------------------------------
* Copyright (C) 2020-2022 LEIDOS.
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
------------------------------------------------------------------------------*/

#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <basic_autonomy/basic_autonomy.hpp>

namespace basic_autonomy
{

/**
 * \brief Return the module-level logger used by all basic_autonomy functions.
 *
 * By default this is a standalone logger named "basic_autonomy" which routes
 * to stdout only. Call set_logger() once from your node at startup to route
 * all basic_autonomy log output through rosout so it is recorded in MCAP:
 *
 *   basic_autonomy::set_logger(get_logger().get_child("basic_autonomy"));
 */
rclcpp::Logger get_logger();

/**
 * \brief Replace the module-level logger used by all basic_autonomy functions.
 *
 * \param logger  Logger to adopt — typically the calling node's get_child("basic_autonomy").
 */
void set_logger(rclcpp::Logger logger);

    namespace log
    {
    /**
    * \brief Helper function to convert a lanelet::BasicPoint2d to a string
    */
    std::string basicPointToStream(lanelet::BasicPoint2d point);

    /**
    * \brief Helper function to convert a PointSpeedPair to a string
    */
    std::string pointSpeedPairToStream(waypoint_generation::PointSpeedPair point);

    /**
    * \brief Print a RCLCPP_DEBUG_STREAM for each value in values where the printed value is a string returned by func
    */
    template <class T>
    void printDebugPerLine(const std::vector<T>& values, std::function<std::string(T)> func)
    {
        for (const auto& value : values)
        {
            RCLCPP_DEBUG_STREAM(basic_autonomy::get_logger(), func(value));
        }
    }

    /**
    * \brief Print a RCLCPP_DEBUG_STREAM for each value in values where the printed value is a string returned by free_func
    */
    template <class T>
    void printDebugPerLine(const std::vector<T>& values, std::string (*free_func)(T))
    {
        auto function = static_cast<std::function<std::string(T)>>(free_func);
        printDebugPerLine(values, function);
    }

    /**
    * \brief Print a RCLCPP_DEBUG_STREAM for each value in values where the printed value is << prefix << value
    */
    void printDoublesPerLineWithPrefix(const std::string& prefix, const std::vector<double>& values);

    }  // namespace log
}  // namespace basic_autonomy