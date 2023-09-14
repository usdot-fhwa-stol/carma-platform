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

#include "roadway_objects/roadway_objects_component.hpp"

#include <memory>

namespace roadway_objects
{
namespace std_ph = std::placeholders;

RoadwayObjectsNode::RoadwayObjectsNode(const rclcpp::NodeOptions & options)
: carma_ros2_utils::CarmaLifecycleNode(options)
{
}

auto RoadwayObjectsNode::handle_on_configure(const rclcpp_lifecycle::State &)
  -> carma_ros2_utils::CallbackReturn
{
  RCLCPP_INFO_STREAM(get_logger(), "RoadwayObjectsNode trying to configure");

  wm_listener_ = std::make_shared<carma_wm::WMListener>(
    get_node_base_interface(), get_node_logging_interface(), get_node_topics_interface(),
    get_node_parameters_interface());

  roadway_obs_pub_ =
    create_publisher<carma_perception_msgs::msg::RoadwayObstacleList>("roadway_objects", 10);

  external_objects_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>(
    "external_objects", 10,
    [this](const carma_perception_msgs::msg::ExternalObjectList::SharedPtr msg_ptr) {
      publish_obstacles(*msg_ptr);
    });

  return CallbackReturn::SUCCESS;
}

auto RoadwayObjectsNode::publish_obstacles(
  const carma_perception_msgs::msg::ExternalObjectList & msg) -> void
{
  carma_perception_msgs::msg::RoadwayObstacleList obstacle_list;

  const auto map{wm_listener_->getWorldModel()->getMap()};

  if (map == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "roadway_objects could not process external objects as no semantic map was available");

    return;
  }

  if (std::size(map->laneletLayer) == 0) {
    RCLCPP_WARN(
      get_logger(),
      "roadway_objects could not process external objects as the semantic map does not contain any "
      "lanelets");

    return;
  }

  const auto world_model{wm_listener_->getWorldModel()};
  for (auto object : msg.objects) {
    const auto obstacle{world_model->toRoadwayObstacle(object)};

    if (!obstacle.has_value()) {
      RCLCPP_DEBUG_STREAM(
        get_logger(), "roadway_objects dropping detected object with id: "
                        << object.id << " as it is off the road.");

      continue;
    }

    obstacle_list.roadway_obstacles.emplace_back(obstacle.get());
  }

  roadway_obs_pub_->publish(obstacle_list);
}

}  // namespace roadway_objects

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(roadway_objects::RoadwayObjectsNode)
