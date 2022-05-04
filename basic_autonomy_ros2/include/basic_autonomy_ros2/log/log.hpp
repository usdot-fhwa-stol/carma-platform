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
#include <basic_autonomy_ros2/basic_autonomy.hpp>

namespace basic_autonomy
{
    static const std::string BASIC_AUTONOMY_LOGGER = "basic_autonomy";

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
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(BASIC_AUTONOMY_LOGGER), func(value));
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