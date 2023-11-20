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
#ifndef __ARBITRATOR_INCLUDE_ARBITRATOR_CONFIG_HPP__
#define __ARBITRATOR_INCLUDE_ARBITRATOR_CONFIG_HPP__

#include <iostream>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>
#include <string>


namespace arbitrator
{
    // Stream operator for map data structure
    std::ostream &operator<<(std::ostream &output, const std::map<std::string, double> &map)
    {
        output << "Map { " << std::endl;
        for (auto const& pair : map)
        {
            output << pair.first << ": " << pair.second << std::endl;
        }
        output << "}";
        return output;
    }

    /**
     * \brief Config struct
     */
    struct Config
    {
        double min_plan_duration = 6.0; // The minimum amount of time in seconds that an arbitrated plan must cover for the
                                        // system to proceed with execution
        double target_plan_duration = 15.0; // The nominal amount of time in seconds that an arbitrated plan should cover for the
                                        // system to operate at best performance
        double planning_frequency = 1.0; // The planning frequency (hz) for generation for arbitrated plans
        int beam_width = 3; // The width of the search beam to use for arbitrator planning, 1 =
                                  // greedy search, as it approaches infinity the search approaches breadth-first search
        bool use_fixed_costs = false; // Use fixed priority cost function over using the cost system for
                                     // evaluating maneuver plans
        std::map<std::string, double> plugin_priorities = {}; // The priorities associated with each plugin during the planning
                                                               // process, values will be normalized at runtime and inverted into costs

        // Stream operator for this config
        friend std::ostream &operator<<(std::ostream &output, const Config &c)
        {
            output << "Arbitrator::Config { " << std::endl
                << "min_plan_duration: " << c.min_plan_duration << std::endl
                << "target_plan_duration: " << c.target_plan_duration << std::endl
                << "planning_frequency: " << c.planning_frequency << std::endl
                << "beam_width: " << c.beam_width << std::endl
                << "use_fixed_costs: " << c.use_fixed_costs << std::endl
                << "plugin_priorities: " << c.plugin_priorities << std::endl
                << "}" << std::endl;
            return output;
        };
    };

}   // namespace abitrator

#endif
