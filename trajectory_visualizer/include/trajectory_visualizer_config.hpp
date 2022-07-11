#pragma once
/*
 * Copyright (C) 2019-2022 LEIDOS.
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

#include <string>

namespace trajectory_visualizer 
{
    /**
   * \brief max_speed values for trajectory_visualizer 
   */
   
  struct Config
  {
    double max_speed = 25.0;
 
    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "trajectory_visualizer::Config { " << std::endl
           << "max_speed: " << c.max_speed << std::endl
           << "}" << std::endl;
      return output;
    }
  };
} // namespace trajectory_visualizer 