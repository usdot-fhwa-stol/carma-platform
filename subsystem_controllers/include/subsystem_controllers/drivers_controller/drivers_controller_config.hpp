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
    std::vector<std::string> required_drivers_;
    std::vector<std::string> lidar_gps_drivers_;
    std::vector<std::string> camera_drivers_;
    double startup_duration_;
    double driver_timeout_;
    bool truck_;
    bool car_;
    

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const DriversControllerConfig &c)
    {
      
      output << "DriversControllerConfig { " << std::endl
             << "required_drivers_: [ " << std::endl;
            
      for (auto node : c.required_drivers_)
        output << node << " ";

      output << "] " << std::endl << "lidar_gps_drivers: [ ";

      for (auto node : c.lidar_gps_drivers_)
        output << node << " ";

      output << "] " << std::endl << "camera_drivers: [ ";

      for (auto node : c.camera_drivers_)
        output << node << " ";
      
      output << "] " << std::endl; 

      output<< "startup_duration: "<< c.startup_duration_ << std::endl;

      output <<"driver_timeout: "<< c.driver_timeout_ << std::endl;

      output <<"truck: " << c.truck_ << std::endl;

      output <<"car: " << c.car_ << std::endl
      
        << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
