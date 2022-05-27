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

#include <carma_wm_ctrl_ros2/GeofenceSchedule.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
namespace carma_wm_ctrl
{

double toSec(const boost::posix_time::time_duration& duration) {
  if (duration.is_special()) {
    throw std::invalid_argument("Cannot convert special duration to seconds");
  }
  return duration.total_microseconds() / 1000000.0;
}

double toSec(const boost::posix_time::ptime& time) {
  return toSec(time - boost::posix_time::from_time_t(0));
}

GeofenceSchedule::GeofenceSchedule()
{
}

GeofenceSchedule::GeofenceSchedule(rclcpp::Time schedule_start, rclcpp::Time schedule_end, rclcpp::Duration control_start,
                                   rclcpp::Duration control_duration, rclcpp::Duration control_offset,
                                   rclcpp::Duration control_span, rclcpp::Duration control_period, DayOfTheWeekSet week_day_set):
                                   schedule_start_(schedule_start), schedule_end_(schedule_end),
                                   control_start_ (control_start), control_duration_ (control_duration),
                                   control_offset_ (control_offset), control_span_ (control_span), control_period_ (control_period), week_day_set_(week_day_set)
{}

bool GeofenceSchedule::scheduleExpired(const rclcpp::Time& time) const
{
  return schedule_end_ < time;
}

bool GeofenceSchedule::scheduleStarted(const rclcpp::Time& time) const
{
  return schedule_start_ <= time;
}

// returns rclcpp::Time(0) when the schedule is expired or the next interval will be on a different day of the week
// Argument provided as absolute time (since 1970)
std::pair<bool, rclcpp::Time> GeofenceSchedule::getNextInterval(const rclcpp::Time& time) const
{
  if (scheduleExpired(time))
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl::GeofenceSchedule"), "Geofence schedule expired");
    std::cerr<<"here1"<<std::endl;
    return std::make_pair(false, rclcpp::Time(0));  // If the schedule has expired or was never started
  }

    std::cerr<<"1111: " << time.seconds()<<std::endl;


  boost::posix_time::ptime boost_time = boost::posix_time::from_time_t(time.seconds());
  boost::gregorian::date date = boost_time.date();

  if (week_day_set_.find(date.day_of_week()) == week_day_set_.end())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl::GeofenceSchedule"), "Geofence wrong day of the week");
    std::cerr<<"here2"<<std::endl;
    return std::make_pair(false, rclcpp::Time(0));  // This geofence is not active on this day
  }

  auto time_of_day = boost_time.time_of_day();

  std::cerr << "boost_time" << boost::posix_time::to_time_t(boost_time) << std::endl;
  std::cerr << "time_of_day" << time_of_day << std::endl;

  // Convert schedule into workable components
  boost::posix_time::ptime ptime_start_of_day(date, boost::posix_time::hours(0));  // Get absolute start time of the day
  
  std::cerr<<"here9: " << std::to_string(toSec(time_of_day)) <<std::endl;


  rclcpp::Time ros_time_of_day = rclcpp::Time(toSec(time_of_day) * 1e9);
  std::cerr<<"here9: " << (ros_time_of_day).seconds() <<std::endl;

  rclcpp::Time abs_day_start = rclcpp::Time(toSec(ptime_start_of_day) * 1e9);
  rclcpp::Duration cur_start = control_start_ + control_offset_; // accounting for the shift of repetition start
  std::cerr<<"here8" << (cur_start).seconds() <<std::endl;
  std::cerr<<"here8" << (control_start_).seconds() <<std::endl;
  std::cerr<<"here8" << (control_offset_).seconds() <<std::endl;

  // Check if current time is after end of control
  if (ros_time_of_day > rclcpp::Time((control_start_ + control_duration_).seconds() * 1e9))
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl::GeofenceSchedule"), "Geofence schedule too late in the day");
    std::cerr<<"here3"<<std::endl;
    // The requested time is after control end so there will not be another interval
    return std::make_pair(false, rclcpp::Time(0));
  }

  // Iterate over the day to find the next control interval
  constexpr int num_sec_in_day = 86400;
  const rclcpp::Duration full_day(num_sec_in_day* 1e9);
  bool time_in_active_period = false;  // Flag indicating if the requested time is within an active control span

  while (cur_start < full_day && ros_time_of_day > rclcpp::Time(cur_start.seconds() * 1e9))
  {
    // Check if the requested time is within the control period being evaluated
    if (rclcpp::Time(cur_start.seconds() * 1e9) < ros_time_of_day &&
        ros_time_of_day < rclcpp::Time((cur_start + control_span_).seconds() * 1e9))
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl::GeofenceSchedule"), "Geofence schedule active!");
      
      time_in_active_period = true;
    }
    cur_start = cur_start + control_period_;

  }
    std::cerr<<"here4: "<< full_day.seconds()<<std::endl;
    std::cerr<<"here4: "<< schedule_end_.seconds()<<std::endl;
    std::cerr<<"here4: "<< (abs_day_start + cur_start).seconds()<<std::endl;


  // check if the only next interval is after the schedule end or past the end of the day
  if (abs_day_start + cur_start > schedule_end_ || cur_start > full_day || cur_start > (control_start_ + control_duration_))
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("carma_wm_ctrl::GeofenceSchedule"), "Geofence schedule beyond end time");
    std::cerr<<"here5"<<std::endl;
    return std::make_pair(time_in_active_period, rclcpp::Time(0));
  }

  std::cerr<<"here6" << (abs_day_start + cur_start).seconds() <<std::endl;
  // At this point we should have the next start time which is still within the schedule and day
  return std::make_pair(time_in_active_period, abs_day_start + cur_start);
}

}  // namespace carma_wm_ctrl
