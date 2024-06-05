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
#include <carma_ros2_utils/timers/ROSTimerFactory.hpp>

// <SONAR_IGNORE_START> // Disable sonar cloud analysis for ROS dependant logic
namespace carma_ros2_utils
{
namespace timers
{
ROSTimerFactory::ROSTimerFactory(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer): weak_node_pointer_(weak_node_pointer){}
ROSTimerFactory::~ROSTimerFactory(){}
std::unique_ptr<Timer> ROSTimerFactory::buildTimer(uint32_t id, rclcpp::Duration duration,
                                                   std::function<void()> callback, bool oneshot,
                                                   bool autostart)
{
  std::unique_ptr<Timer> timer_ptr(new ROSTimer(weak_node_pointer_));
  timer_ptr->initializeTimer(duration, callback, oneshot, autostart);
  timer_ptr->setId(id);
  return timer_ptr;
}

void ROSTimerFactory::setCarmaLifecycleNode(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer)
{
  weak_node_pointer_ = weak_node_pointer;
}

rclcpp::Time  ROSTimerFactory::now()
{
  if (!weak_node_pointer_.expired())
    return weak_node_pointer_.lock()->get_clock()->now();
  else
  {
    std::cerr << "ROSTimerFactory's weak pointer to the owner node is expired and clock is not available! Returning rclcpp::Time(0)" << std::endl;
    return rclcpp::Time(0, 0);
  }
    
}

}  // namespace timers
}  // namespace carma_ros2_utils