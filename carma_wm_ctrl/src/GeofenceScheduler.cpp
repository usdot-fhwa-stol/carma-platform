/*
 * Copyright (C) 2022 LEIDOS.
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

#include <carma_wm_ctrl/GeofenceScheduler.hpp>

namespace carma_wm_ctrl
{
using std::placeholders::_1;

GeofenceScheduler::GeofenceScheduler(std::shared_ptr<TimerFactory> timerFactory)
  : timerFactory_(timerFactory)
{
  // Create repeating loop to clear geofence timers which are no longer needed
  deletion_timer_ =
      timerFactory_->buildTimer(nextId(), rclcpp::Duration(1), std::bind(&GeofenceScheduler::clearTimers, this));
  clock_type_ = timerFactory_->now().get_clock_type();
}

rclcpp::Time GeofenceScheduler::now()
{
  return timerFactory_->now();
}

uint32_t GeofenceScheduler::nextId()
{
  next_id_++;
  return next_id_;
}

rcl_clock_type_t GeofenceScheduler::getClockType()
{
  return clock_type_;
}

void GeofenceScheduler::clearTimers()
{
  std::lock_guard<std::mutex> guard(mutex_);
  auto it = timers_.begin();

  // Erase all expired timers_ using iterators to ensure operation is safe
  while (it != timers_.end())
  {
    // Check if timer is marked for deletion
    if (it->second.second)
    {
      // erase() function returns the iterator of the next
      // to last deleted element.
      it = timers_.erase(it);
    }
    else
    {
      it++;
    }
  }
}

void GeofenceScheduler::addGeofence(std::shared_ptr<Geofence> gf_ptr)
{ 
  std::lock_guard<std::mutex> guard(mutex_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Attempting to add Geofence with Id: " << gf_ptr->id_);

  // Create timer for next start time
  for (size_t schedule_idx = 0; schedule_idx < gf_ptr->schedules.size(); schedule_idx++)
  {
    // resolve clock type
    auto interval_info = gf_ptr->schedules[schedule_idx].getNextInterval(timerFactory_->now());
    rclcpp::Time startTime = interval_info.second;

    if (!interval_info.first && startTime == rclcpp::Time(0, 0, clock_type_))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("carma_wm_ctrl"), 
          "Failed to add geofence as its schedule did not contain an active or upcoming control period. GF Id: "
          << gf_ptr->id_);
      return;
    }
    // If this geofence is currently active set the start time to now
    if (interval_info.first)
    {
      startTime = timerFactory_->now();
    }

    int32_t timer_id = nextId();

    rclcpp::Duration control_duration = rclcpp::Duration(std::max((startTime - timerFactory_->now()).seconds() * 1e9, 0.0)); //guard for negative value

    // Build timer to trigger when this geofence becomes active
    TimerPtr timer = timerFactory_->buildTimer(
        timer_id, control_duration,
        std::bind(&GeofenceScheduler::startGeofenceCallback, this, gf_ptr, schedule_idx, timer_id), true, true);

    timers_[timer_id] = std::make_pair(std::move(timer), false);  // Add start timer to map by Id
  }

}

void GeofenceScheduler::startGeofenceCallback(std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id)
{
  std::lock_guard<std::mutex> guard(mutex_);
  rclcpp::Time endTime = timerFactory_->now() + gf_ptr->schedules[schedule_id].control_span_;

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Activating Geofence with Id: " << gf_ptr->id_ << ", which will end at:" << endTime.seconds());

  active_callback_(gf_ptr);

  // Build timer to trigger when this geofence becomes inactive
  int32_t ending_timer_id = nextId();

  rclcpp::Duration control_duration = rclcpp::Duration(std::max((endTime - timerFactory_->now()).seconds() * 1e9, 0.0)); //guard for negative value

  TimerPtr timer = timerFactory_->buildTimer(
      ending_timer_id, control_duration,
      std::bind(&GeofenceScheduler::endGeofenceCallback, this, gf_ptr, schedule_id, ending_timer_id), true, true);
  timers_[ending_timer_id] = std::make_pair(std::move(timer), false);  // Add end timer to map by Id

  timers_[timer_id].second = true;  // Mark start timer for deletion
}

void GeofenceScheduler::endGeofenceCallback(std::shared_ptr<Geofence> gf_ptr, const unsigned int schedule_id, const int32_t timer_id)
{
  std::lock_guard<std::mutex> guard(mutex_);

  RCLCPP_INFO_STREAM(rclcpp::get_logger("carma_wm_ctrl"), "Deactivating Geofence with Id: " << gf_ptr->id_);

  inactive_callback_(gf_ptr);
  timers_[timer_id].second = true;  // Mark timer for deletion

  // Determine if a new timer is needed for this geofence
  auto interval_info = gf_ptr->schedules[schedule_id].getNextInterval(timerFactory_->now());
  rclcpp::Time startTime = interval_info.second;

  // If this geofence should currently be active set the start time to now
  if (interval_info.first)
  {
    startTime = timerFactory_->now();
  }

  if (!interval_info.first && startTime == rclcpp::Time(0, 0, clock_type_))
  {
    // No more active periods for this geofence so return
    return;
  }

  // Build timer to trigger when this geofence becomes active
  int32_t start_timer_id = nextId();

  rclcpp::Duration control_duration = rclcpp::Duration(std::max((startTime - timerFactory_->now()).seconds() * 1e9, 0.0)); //guard for negative value

  TimerPtr timer = timerFactory_->buildTimer(
      start_timer_id, control_duration,
      std::bind(&GeofenceScheduler::startGeofenceCallback, this, gf_ptr, schedule_id, start_timer_id), true, true);

  timers_[start_timer_id] = std::make_pair(std::move(timer), false);  // Add start timer to map by Id
}

void GeofenceScheduler::onGeofenceActive(std::function<void(std::shared_ptr<Geofence>)> active_callback)
{
  std::lock_guard<std::mutex> guard(mutex_);
  active_callback_ = active_callback;
}

void GeofenceScheduler::onGeofenceInactive(std::function<void(std::shared_ptr<Geofence>)> inactive_callback)
{
  std::lock_guard<std::mutex> guard(mutex_);
  inactive_callback_ = inactive_callback;
}
}  // namespace carma_wm_ctrl