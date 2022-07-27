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
#include <functional>
#include <carma_wm_ros2/WMListener.hpp>
#include <carma_wm_ros2/WorldModel.hpp>
#include <carma_perception_msgs/msg/roadway_obstacle_list.hpp>
#include <functional>

#include "roadway_objects_worker.hpp"

#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace roadway_objects
{

  /**
   * \class RoadwayObjectsNode
   * \brief The class responsible for converting detected objects into objects that are mapped onto specific lanelets.
   * 
   */
  class RoadwayObjectsNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:

    // Subscribers
    carma_ros2_utils::SubPtr<carma_perception_msgs::msg::ExternalObjectList> external_objects_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<carma_perception_msgs::msg::RoadwayObstacleList> roadway_obs_pub_;

    // World Model Listener. Must be declared before object_worker_ for proper initialization.
    std::shared_ptr<carma_wm::WMListener> wm_listener_;

    // Worker class object
    std::shared_ptr<RoadwayObjectsWorker> object_worker_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit RoadwayObjectsNode(const rclcpp::NodeOptions &);

    /*!
      \brief Callback to publish RoadwayObstacleList
    */
    void publishObstacles(const carma_perception_msgs::msg::RoadwayObstacleList& msg);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
  };

} // roadway_objects
