#pragma once
/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <rclcpp/time.hpp>
#include <rclcpp/timer.hpp>
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include "TimerFactory.hpp"
#include "Timer.hpp"
#include "ROSTimer.hpp"

namespace carma_ros2_utils
{
namespace timers
{
/**
 * @brief Implementation of the TimerFactory interface that returns ROSTimer objects.
 *
 */
class ROSTimerFactory : public TimerFactory
{
public:
  ROSTimerFactory(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer);
  /**
   * @brief Destructor
   */
  ~ROSTimerFactory();

  void setCarmaLifecycleNode(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer);

  //// Overrides
  std::unique_ptr<Timer> buildTimer(uint32_t id, rclcpp::Duration duration,
                                    std::function<void()> callback, bool oneshot = false,
                                    bool autostart = true) override;
  rclcpp::Time now();

private:
  std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer_;
};
}  // namespace timers
}  // namespace carma_ros2_utils
