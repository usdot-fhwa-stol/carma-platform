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

  class DriversControllerNode : public BaseSubsystemController
  {
  public:
    
    DriversControllerNode() = delete;

    ~DriversControllerNode() = default;

    /**
     * \brief Constructor. Set explicitly to support node composition.
     * 
     * \param options The node options to use for configuring this node
     */
    explicit DriversControllerNode(const rclcpp::NodeOptions &options);

    // TODO integrate driver_discovery/health_monitor behavior into this node
    // https://github.com/usdot-fhwa-stol/carma-platform/issues/1500

  };

} // namespace v2x_controller

