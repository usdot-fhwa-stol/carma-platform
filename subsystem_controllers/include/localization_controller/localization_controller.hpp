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
#include "base_subsystem_controller/base_subsystem_controller.hpp"

namespace subsystem_controller
{

  class LocalizationControllerNode : public BaseSubsystemController
  {
  public:
    LocalizationControllerNode() = delete;

    ~LocalizationControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit LocalizationControllerNode(const rclcpp::NodeOptions &options);

    // TODO we should add a callback for the localization state and if the we have fallen back to GNSS something should happen

  };

} // namespace v2x_controller
