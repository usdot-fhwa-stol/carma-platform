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
  * \brief Stuct containing the algorithm configuration values for the GuidanceController
  */
  struct GuidanceControllerConfig
  
  {
    //! List of guidance plugins (node name) to consider required and who's failure shall result in automation abort. 
    //  Required plugins will be automatically activated at startup
    //  Required plugins cannot be deactivated by the user
    std::vector<std::string> required_plugins;

    //! List of guidance plugins which are not required but the user wishes to have automatically activated
    //  so that the user doesn't need to manually activate them via the UI on each launch (though they still can)
    //  this list should have zero intersection with the required_plugins
    std::vector<std::string> auto_activated_plugins;

    //! List of guidance plugins that are ROS2. If it is not in the list, it is assumed to be ROS1 and not managed
    std::vector<std::string> ros2_initial_plugins;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const GuidanceControllerConfig &c)
    {
      
      output << "GuidanceControllerConfig { " << std::endl
             << "required_plugins: [ " << std::endl;
            
      for (auto node : c.required_plugins)
        output << node << " ";

      output << "] " << std::endl << "auto_activated_plugins: [ ";

      for (auto node : c.auto_activated_plugins)
        output << node << " ";

      output << "] " << std::endl << "ros2_initial_plugins: [ ";

       for (auto node : c.ros2_initial_plugins)
        output << node << " ";
      
      output << "] " << std::endl 
        << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
