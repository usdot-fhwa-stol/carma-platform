#pragma once

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

#include <iostream>
#include <vector>

namespace subsystem_controllers
{
  /**
 * \brief Stuct containing the algorithm configuration values for the BaseSubsystemController
 */
  struct BaseSubSystemControllerConfig
  {
    std::vector<std::string> required_subsystem_nodes;

    std::string subsystem_namespace = "/false_namespace";

    uint64_t service_timeout_ms = 1000;

    uint64_t call_timeout_ms = 1000;

    friend std::ostream &operator<<(std::ostream &output, const BaseSubSystemControllerConfig &c)
    {
      output << "BaseSubSystemControllerConfig { " << std::endl
             << "service_timeout_ms: " << c.service_timeout_ms << std::endl
             << "call_timeout_ms: " << c.call_timeout_ms << std::endl
             << "subsystem_namespace: " << c.subsystem_namespace << std::endl
             << "required_subsystem_nodes: [ ";

      for (auto node : c.required_subsystem_nodes)
        output << node << " ";

      output << "] " << std::endl
             << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
