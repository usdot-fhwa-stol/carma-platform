/*
 * Copyright (C) 2019-2020 LEIDOS.
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
#include "RoadwayObjectsWorker.h"
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>
#include <lanelet2_core/utility/Optional.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// TODO whole file

namespace objects
{
RoadwayObjectsWorker::RoadwayObjectsWorker(carma_wm::WorldModelConstPtr wm, PublishObstaclesCallback obj_pub)
  : obj_pub_(obj_pub), wm_(wm)
{
}

void RoadwayObjectsWorker::externalObjectsCallback(const cav_msgs::ExternalObjectListConstPtr& obj_array)
{
  cav_msgs::RoadwayObstacleList obstacle_list;
  auto map = wm_->getMap();
  if (!map)
  {
    ROS_WARN("roadway_objects could not process external objects as no semantic map was available");
    return;
  }

  if (map->laneletLayer.size() == 0)
  {
    ROS_WARN("roadway_objects could not process external objects as the semantic map does not contain any lanelets");
    return;
  }

  for (auto object : obj_array->objects)
  {
    lanelet::Optional<cav_msgs::RoadwayObstacle> obs = wm_->toRoadwayObstacle(object);
    if (!obs)
    {
      continue;
    }

    obstacle_list.roadway_obstacles.emplace_back(obs.get());
  }

  obj_pub_(obstacle_list);
}
}  // namespace objects
