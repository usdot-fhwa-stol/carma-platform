// Copyright 2019-2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROADWAY_OBJECTS__ROADWAY_OBJECTS_COMPONENT_HPP_
#define ROADWAY_OBJECTS__ROADWAY_OBJECTS_COMPONENT_HPP_

#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_wm/WMListener.hpp>
#include <carma_wm/WorldModel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <memory>

namespace roadway_objects
{
/**
   * \class RoadwayObjectsNode
   * \brief The class responsible for converting detected objects into objects that are mapped
   * onto specific lanelets.
   *
   */
class RoadwayObjectsNode : public carma_ros2_utils::CarmaLifecycleNode
{
public:
  explicit RoadwayObjectsNode(const rclcpp::NodeOptions &);

  auto publish_obstacles(const carma_perception_msgs::msg::ExternalObjectList & msg) -> void;

  auto handle_on_configure(const rclcpp_lifecycle::State &)
    -> carma_ros2_utils::CallbackReturn override;

private:
  rclcpp::Subscription<carma_perception_msgs::msg::ExternalObjectList>::SharedPtr
    external_objects_sub_{nullptr};

  rclcpp_lifecycle::LifecyclePublisher<carma_perception_msgs::msg::RoadwayObstacleList>::SharedPtr
    roadway_obs_pub_{nullptr};

  std::shared_ptr<carma_wm::WMListener> wm_listener_{nullptr};
};

}  // namespace roadway_objects

#endif  // ROADWAY_OBJECTS__ROADWAY_OBJECTS_COMPONENT_HPP_
