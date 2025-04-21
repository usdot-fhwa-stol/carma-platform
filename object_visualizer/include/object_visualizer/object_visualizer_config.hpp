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

    // Unsigned int: Visualization marker shape: CUBE=1, SPHERE=2, CYLINDER=3
    uint8_t marker_shape = 2;

    //! External Objects marker rviz namespace
    std::string external_objects_viz_ns = "external_objects";

    //! Roadway Obstacles marker rviz namespace
    std::string roadway_obstacles_viz_ns = "roadway_obstacles";

    //! If true, pedestrians will be visualized using specialized icons/models instead of basic markers
    bool use_pedestrian_icon = true;

    //! Path to the 3D model file for pedestrian visualization
    std::string pedestrian_icon_path = "package://object_visualizer/meshes/pedestrian.dae";

    //! Scale factor to apply to the pedestrian icon model
    double pedestrian_icon_scale = 1.0;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "object_visualizer::Config { " << std::endl
           << "enable_external_objects_viz: " << c.enable_external_objects_viz << std::endl
           << "enable_roadway_objects_viz: " << c.enable_roadway_objects_viz << std::endl
           << "external_objects_viz_ns: " << c.external_objects_viz_ns << std::endl
           << "roadway_obstacles_viz_ns: " << c.roadway_obstacles_viz_ns << std::endl
           << "marker_shape: " << static_cast<int>(c.marker_shape) << std::endl
           << "use_pedestrian_icon: " << c.use_pedestrian_icon << std::endl
           << "pedestrian_icon_path: " << c.pedestrian_icon_path << std::endl
           << "pedestrian_icon_scale: " << c.pedestrian_icon_scale << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // object_visualizer
