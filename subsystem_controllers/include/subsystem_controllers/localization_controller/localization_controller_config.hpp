#pragma once

/*
 * Copyright (C) 2021-2022 LEIDOS.
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
#include <unordered_map>
#include <type_traits>
#include <boost/functional/hash/extensions.hpp>
#include <rapidjson/document.h>

namespace subsystem_controllers
{
  enum class SensorBooleanStatus {
    FAILED=0,
    OPERATIONAL=1
  };

  enum class SensorAlertStatus {
    FATAL=0, 
    OPERATIONAL=1,
    CAUTION=2,
    WARNING=3
  };

  struct VectorHash : boost::hash<std::vector<SensorBooleanStatus>> {};

 /**
  * \brief Stuct containing the algorithm configuration values for the BaseSubsystemController
  */
  struct LocalizationControllerConfig
  {
    //! sensor_nodes
    std::vector<std::string> sensor_nodes;

    // TODO comments
    std::unordered_map<std::vector<SensorBooleanStatus>, SensorAlertStatus, VectorHash> sensor_fault_map;


    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const LocalizationControllerConfig &c)
    {
      
      output << "LocalizationControllerConfig { " << std::endl
             << "sensor_nodes: [ ";

      for (auto node : c.sensor_nodes)
        output << node << ", ";

      output << "] " << std::endl;
      
      output << "sensor_fault_map: [ ";

      for (auto const& pair: c.sensor_fault_map) {
        output << "{ ";
        
        for (auto const& status : pair.first) {
          output << static_cast<std::underlying_type<SensorBooleanStatus>::type>(status) << ", ";
        }

        output << ": " << static_cast<std::underlying_type<SensorAlertStatus>::type>(pair.second) << " }" << std::endl;
      }

      output << "] " << std::endl
             << "}" << std::endl;
      return output;
    }
  };

} // namespace subsystem_controllers
