#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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



#include <memory>

#include "carma_msgs/msg/system_alert.hpp"
#include "ros2_lifecycle_manager/ros2_lifecycle_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subsystem_controllers/base_subsystem_controller/base_subsystem_controller.hpp"

namespace subsystem_controllers
{

  class GuidanceControllerNode : public BaseSubsystemController
  {
  public:
    
    GuidanceControllerNode() = delete;

    ~GuidanceControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit GuidanceControllerNode(const rclcpp::NodeOptions &options);

    // TODO The ROS1 plugin manager functionality should be updated to properly interact with or exist in this node or a guidance_plugin_controller node
    // https://github.com/usdot-fhwa-stol/carma-platform/issues/1499

  };

} // namespace v2x_controller

