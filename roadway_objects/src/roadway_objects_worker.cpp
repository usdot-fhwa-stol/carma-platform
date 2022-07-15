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
#include "roadway_objects/roadway_objects_worker.hpp"

namespace roadway_objects
{
RoadwayObjectsWorker::RoadwayObjectsWorker(carma_wm::WorldModelConstPtr wm, const PublishObstaclesCallback& obj_pub, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger)
  : obj_pub_(obj_pub), wm_(wm), logger_(logger)
{
}

void RoadwayObjectsWorker::externalObjectsCallback(const carma_perception_msgs::msg::ExternalObjectList::UniquePtr obj_array)
{
  carma_perception_msgs::msg::RoadwayObstacleList obstacle_list;
  auto map = wm_->getMap();
  if (!map)
  {
    RCLCPP_WARN_STREAM(logger_->get_logger(), "roadway_objects could not process external objects as no semantic map was available");
    return;
  }

  if (map->laneletLayer.size() == 0)
  {
    RCLCPP_WARN_STREAM(logger_->get_logger(), "roadway_objects could not process external objects as the semantic map does not contain any lanelets");
    return;
  }

  for (auto object : obj_array->objects)
  {
    lanelet::Optional<carma_perception_msgs::msg::RoadwayObstacle> obs = wm_->toRoadwayObstacle(object);
    if (!obs)
    {
      RCLCPP_DEBUG_STREAM(logger_->get_logger(), "roadway_objects dropping detected object with id: " << object.id << " as it is off the road.");
      continue;
    }

    obstacle_list.roadway_obstacles.emplace_back(obs.get());
  }

  obj_pub_(obstacle_list);

}
}  // namespace objects