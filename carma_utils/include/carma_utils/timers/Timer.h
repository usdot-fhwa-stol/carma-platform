#pragma once
/*
 * Copyright (C) 2020-2021 LEIDOS.
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
#include <ros/time.h>
#include <ros/timer.h>

namespace carma_utils
{
namespace timers
{
/**
 * @brief A timer class interface which will trigger the provided callback after the requested duration.
 *        This class provides a mechanism for separating the initialization of ROS style timers from the implementation.
 *
 * Timer objects can additionally have a user defined ID field to help track timers.
 */
class Timer
{
protected:
  uint32_t id_ = 0;

public:
  /**
   * @brief Destructor
   */
  virtual ~Timer(){};

  /**
   * @brief Initialize a timer to trigger the provided callback after the provided duration
   *
   * @param duration The duration the timer will wait for before a callback is triggered
   * @param callback The callback to trigger after duration has elapsed
   * @param oneshot If true the timer will only trigger one. If false it will trigger repeatedly with duration length
   * increments
   * @param autostart If true the timer will immediately start after this function is called. Otherwise the start()
   * function must be called
   */
  virtual void initializeTimer(ros::Duration duration, std::function<void(const ros::TimerEvent&)> callback,
                               bool oneshot = false, bool autostart = true) = 0;

  /**
   * @brief Start the timer coutdown
   */
  virtual void start() = 0;

  /**
   * @brief Stop the timer coutdown
   */
  virtual void stop() = 0;

  /**
   * @brief Get the timer id. It is up to the user to ensure uniqueness of the ID.
   *
   * @return The id of this timer
   */
  virtual uint32_t getId()
  {
    return id_;
  }

  /**
   * @brief Set the id of this timer
   *
   * @param id The id to set
   */
  virtual void setId(uint32_t id)
  {
    id_ = id;
  }
};
}  // namespace timers
}  // namespace carma_utils