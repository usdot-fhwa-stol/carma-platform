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
#include <carma_ros2_utils/timers/ROSTimer.hpp>

// <SONAR_IGNORE_START> // Disable sonar cloud analysis for ROS dependant logic
namespace carma_ros2_utils
{
namespace timers
{
ROSTimer::ROSTimer(std::weak_ptr<carma_ros2_utils::CarmaLifecycleNode> weak_node_pointer) : weak_node_pointer_(weak_node_pointer) {}

ROSTimer::~ROSTimer(){}

void ROSTimer::initializeTimer(rclcpp::Duration duration, std::function<void()> callback,
                               bool oneshot, bool autostart)
{
  if (timer_) {
    timer_->cancel();
  }

  duration_ = duration;
  autostart_ = autostart;

  // Wrap the input callback with a lambda to cancel the timer if it was a oneshot timer
  callback_ = [oneshot, callback, this] () -> void {
    callback();
    if (oneshot && this->timer_) {
      this->timer_->cancel();
    }
  };
  
  if (autostart) {

    if (auto temp_node_ptr = weak_node_pointer_.lock())
    {
      timer_ = temp_node_ptr->create_timer(
        temp_node_ptr->get_clock(),
        duration_,
        callback_
      );
    
      if (!timer_) 
      {
        throw std::runtime_error("Null timer returned from node handle");
      }
    }
    else
    {
      // This is an log instead of exception because at the moment, I believe this could only ever happen during a multi-threaded node shutdown.
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_ros2_utils"), "Failed to acquire pointer for node handle in ROSTimer. Timer could not be intialized");
    }
  } 
  else 
  {
    timer_ = nullptr; // Delay timer activate till start is called
  }
}

void ROSTimer::start()
{

  // If the timer is not intialized and this is an autostart timer then attempt to initialize the timer
  if (!timer_ && autostart_) {

    if (auto temp_node_ptr = weak_node_pointer_.lock())
    {
      timer_ = temp_node_ptr->create_timer(
        temp_node_ptr->get_clock(),
        duration_,
        callback_);
    
      if (!timer_) 
      {
        throw std::runtime_error("Null timer returned from node handle");
      }
    } 
    else 
    {
      // This is an log instead of exception because at the moment, I believe this could only ever happen during a multi-threaded node shutdown.
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("carma_ros2_utils"), "Failed to acquire pointer for node handle in ROSTimer. Timer could not be started");
    }
    
  }

  if (timer_ && timer_->is_canceled())
  {
    timer_->reset();
  }
}

void ROSTimer::stop()
{
  if (timer_) 
  {
    timer_->cancel();
  }
}
}  // namespace timers
}  // namespace carma_ros2_utils