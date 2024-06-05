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
#include <chrono>
#include <memory>
#include "carma_ros2_utils/carma_lifecycle_node.hpp"
#include <rclcpp/timer.hpp>
#include <rclcpp/time.hpp>
#include "Timer.hpp"

namespace carma_ros2_utils
{
namespace timers
{
/**
 * @brief Implementation of the Timer interface that uses rclcpp::Timer objects on the backend.
 *
 */
class ROSTimer : public Timer
{
  rclcpp::TimerBase::SharedPtr timer_;
  std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer_;
  rclcpp::Duration duration_ = rclcpp::Duration(std::chrono::nanoseconds{0});
  bool autostart_=true;
  std::function<void()> callback_;


public:
  ROSTimer(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer);

  ~ROSTimer();

  //// Overrides
  void initializeTimer(rclcpp::Duration duration, std::function<void()> callback,
                       bool oneshot = false, bool autostart = true) override;

  void start() override;

  void stop() override;
};
}  // namespace timers
}  // namespace carma_ros2_utils
