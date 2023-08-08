#pragma once

/*
 * Copyright (C) 2023 LEIDOS.
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
  * \brief Stuct containing the algorithm configuration values for the GuidanceController
  */
  struct DriversControllerConfig
  
  {
    //! List of drivers (node name) to consider required and who's failure shall result in automation abort. 
    //  Required plugins will be automatically activated at startup
    //  Required plugins cannot be deactivated by the user
    std::vector<std::string> required_subsystem_nodes;

    //! List of drivers that are ROS2. If it is not in the list, it is assumed to be ROS1 and not managed
    std::vector<std::string> unmanaged_required_nodes;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const GuidanceControllerConfig &c)
    {
      
      output << "DriversControllerConfig { " << std::endl
             << "required_subsystem_nodes: [ " << std::endl;
            
      for (auto node : c.required_subsystem_nodes)
        output << node << " ";

      output << "] " << std::endl << "unmanaged_required_nodes: [ ";

      for (auto node : c.unmanaged_required_nodes)
        output << node << " ";
      
      output << "] " << std::endl 
        << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
