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
    //! List of nodes to consider required and who's failure shall result in system shutdown
    std::vector<std::string> required_subsystem_nodes;

    //! Node Namespace. Any node under this namespace shall have its lifecycle managed by this controller
    std::string subsystem_namespace = "/false_namespace";

    //! Timeout in ms for service availability
    int service_timeout_ms = 1000;

    //! Timeout in ms for service calls
    int call_timeout_ms = 1000;

    //! If this flag is true then all nodes under subsystem_namespace are treated as required in addition to any nodes in required_subsystem_nodes
    bool full_subsystem_required = false;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const BaseSubSystemControllerConfig &c)
    {
      
      output << "BaseSubSystemControllerConfig { " << std::endl
             << "service_timeout_ms: " << c.service_timeout_ms << std::endl
             << "call_timeout_ms: " << c.call_timeout_ms << std::endl
             << "subsystem_namespace: " << c.subsystem_namespace << std::endl
             << "full_subsystem_required: " << c.full_subsystem_required << std::endl
             << "required_subsystem_nodes: [ ";

      for (auto node : c.required_subsystem_nodes)
        output << node << " ";

      output << "] " << std::endl
             << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
