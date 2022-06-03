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

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <carma_perception_msgs/msg/external_object.hpp>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/utility/Optional.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <functional>

namespace roadway_objects
{
class RoadwayObjectsWorker
{
public:
  using PublishObstaclesCallback = std::function<void(const carma_perception_msgs::msg::RoadwayObstacleList&)>;

  /*!
   * \brief Constructor
   */
  RoadwayObjectsWorker(carma_wm::WorldModelConstPtr wm, const PublishObstaclesCallback& obj_pub, rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger);

  /*!
   * \brief Converts the provided ExternalObjectList to a RoadwayObstacleList and re-publishes it
   * \param msg array of detected objects.
  */
  void externalObjectsCallback(const carma_perception_msgs::msg::ExternalObjectList::UniquePtr msg);

private:
  // local copy of external object publihsers
  PublishObstaclesCallback obj_pub_;

  // Logger interface
  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

  carma_wm::WorldModelConstPtr wm_;
};

}  // namespace roadway_objects