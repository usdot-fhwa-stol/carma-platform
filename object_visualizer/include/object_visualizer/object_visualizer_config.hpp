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

#include <iostream>
#include <vector>

namespace object_visualizer
{

  /**
   * \brief Stuct containing the algorithm configuration values for object_visualizer
   */
  struct Config
  {
    //! If true then RViz markers will be forwarded for received carma_perception_msgs/ExternalObjectList messages
    bool enable_external_objects_viz = true;

    //! If true then RViz markers will be forwarded for received carma_perception_msgs/RoadwayObstacleList messages
    bool enable_roadway_objects_viz = true;

    //! External Objects marker rviz namespace
    std::string external_objects_viz_ns = "external_objects";

    //! Roadway Obstacles marker rviz namespace
    std::string roadway_obstacles_viz_ns = "roadway_obstacles";

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "object_visualizer::Config { " << std::endl
           << "enable_external_objects_viz: " << c.enable_external_objects_viz << std::endl
           << "enable_roadway_objects_viz: " << c.enable_roadway_objects_viz << std::endl
           << "external_objects_viz_ns: " << c.external_objects_viz_ns << std::endl
           << "roadway_obstacles_viz_ns: " << c.roadway_obstacles_viz_ns << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // object_visualizer