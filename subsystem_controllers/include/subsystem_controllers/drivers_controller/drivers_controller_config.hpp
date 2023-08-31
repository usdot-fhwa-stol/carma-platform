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
    //! List of ros1 controller drivers (node name) to consider required and who's failure shall result in automation abort. 
    std::vector<std::string> ros1_required_drivers_;
    //! List of ros1 camera drivers (node name) to consider required and who's failure shall result in automation abort.
    std::vector<std::string> ros1_camera_drivers_;
    //! List of nodes in the namespace which will not be managed by this subsystem controller
    std::vector<std::string> excluded_namespace_nodes_;
    //! The time allocated for system startup in seconds
    int startup_duration_;
    //! The timeout threshold for essential drivers in ms
    double driver_timeout_ = 1000;
    

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const DriversControllerConfig &c)
    {
      
      output << "DriversControllerConfig { " << std::endl
             << "ros1_required_drivers: [ " << std::endl;
            
      for (auto node : c.ros1_required_drivers_)
        output << node << " ";

      output << "] " << std::endl << "ros1_camera_drivers: [ ";

      for (auto node : c.ros1_camera_drivers_)
        output << node << " ";
      
      output << "] " << std::endl << "excluded_namespace_nodes: [ ";

      for (auto node : c.excluded_namespace_nodes_)
        output << node << " ";

      output<< "] " << std::endl << "startup_duration: "<< c.startup_duration_ << std::endl;

      output <<"driver_timeout: "<< c.driver_timeout_ << std::endl
      
        << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
