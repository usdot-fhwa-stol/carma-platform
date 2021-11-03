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
/**
 * \brief Stuct containing the algorithm configuration values for the SystemController
 */
struct SystemControllerConfig
{
  //! List of nodes to consider required and who's failure shall result in system shutdown
  //  This is also the total set of nodes which will be managed by this controller
  std::vector<std::string> required_subsystem_nodes;

  //! Time in seconds to wait before telling all nodes to configure
  double signal_configure_delay = 20.0;

  //! Timeout in ms for service availability
  uint64_t service_timeout_ms = 1000;
  
  //! Timeout in ms for service calls
  uint64_t call_timeout_ms = 1000;

  // Stream operator for this config
  friend std::ostream& operator<<(std::ostream& output, const SystemControllerConfig& c)
  {
    output << "SystemControllerConfig { " << std::endl
           << "signal_configure_delay: " << c.signal_configure_delay << std::endl
           << "service_timeout_ms: " << c.service_timeout_ms << std::endl
           << "call_timeout_ms: " << c.call_timeout_ms << std::endl
           << "required_subsystem_nodes: [ ";
    
    for (auto node : c.required_subsystem_nodes)
        output << node << " ";
    

    output << "] " << std::endl << "}" << std::endl;
    return output;
  }
};