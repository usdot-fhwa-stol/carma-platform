#pragma once

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

namespace trajectory_executor
{

  /**
   * \brief Struct containing the algorithm configuration values for trajectory_executor
   */
  struct Config
  {
    double trajectory_publish_rate = 10.0; // Publish rate (in Hz) for the outbound trajectories to the control plugins

    std::string default_control_plugin = "Pure Pursuit"; // Name of default control plugin

    std::string default_control_plugin_topic = "/guidance/pure_pursuit/plan_trajectory"; // Full path to default control plugin's trajectory input topic

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "trajectory_executor::Config { " << std::endl
           << "trajectory_publish_rate: " << c.trajectory_publish_rate << std::endl
           << "default_control_plugin: " << c.default_control_plugin << std::endl
           << "default_control_plugin_topic: " << c.default_control_plugin_topic << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // trajectory_executor