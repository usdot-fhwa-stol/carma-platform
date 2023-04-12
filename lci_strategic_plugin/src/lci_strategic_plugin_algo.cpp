/*
 * Copyright (C) 2023 LEIDOS.
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
#include "lci_strategic_plugin/lci_strategic_plugin.hpp"
#include "lci_strategic_plugin/lci_states.hpp"

namespace lci_strategic_plugin
{

#define EPSILON 0.01

double LCIStrategicPlugin::estimate_distance_to_stop(double v, double a) const
{
  return (v * v) / (2.0 * a);
}

double LCIStrategicPlugin::estimate_time_to_stop(double d, double v) const
{
  return 2.0 * d / v;
};

double LCIStrategicPlugin::get_distance_to_accel_or_decel_twice(double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const
{
  // (v_e^2 - v^2)/ 2a_a + (v_d^2 - v_e^2)/ 2a_d
  return (std::pow(free_flow_speed, 2) - std::pow(current_speed, 2))/(2 * max_accel) + (std::pow(departure_speed, 2) - std::pow(free_flow_speed, 2))/(2* max_decel);
}

double LCIStrategicPlugin::get_distance_to_accel_or_decel_once (double current_speed, double departure_speed, double max_accel, double max_decel) const
{
  if (current_speed <= departure_speed + epsilon_)
  {
    return (std::pow(departure_speed, 2) - std::pow(current_speed, 2))/(2 * max_accel);
  }
  else
  {
    return (std::pow(departure_speed, 2) - std::pow(current_speed, 2))/(2 * max_decel);
  }
}

rclcpp::Time LCIStrategicPlugin::get_nearest_green_entry_time(const rclcpp::Time& current_time, const rclcpp::Time& earliest_entry_time, lanelet::CarmaTrafficSignalPtr signal, double minimum_required_green_time) const
{
  boost::posix_time::time_duration g =  lanelet::time::durationFromSec(minimum_required_green_time);         // provided by considering min headways of vehicles in front
  boost::posix_time::ptime t = lanelet::time::timeFromSec(current_time.seconds());                        // time variable
  boost::posix_time::ptime eet = lanelet::time::timeFromSec(earliest_entry_time.seconds());                        // earliest entry time

  auto curr_pair = signal->predictState(t);
  if (!curr_pair)
    throw std::invalid_argument("Traffic signal with id:" + std::to_string(signal->id()) + ", does not have any recorded time stamps!");

  boost::posix_time::time_duration theta =  curr_pair.get().first - t;   // remaining time left in this state
  auto p = curr_pair.get().second;
  while ( 0.0 < g.total_milliseconds() || p != lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED) //green
  {
    if ( p == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
    {
      if (g < theta)
      {
        t = t + g;
        theta = theta - g;
        g = boost::posix_time::seconds(0);
      }
      else
      {
        t = t + theta;
        g = g - theta;
        curr_pair = signal->predictState(t + boost::posix_time::milliseconds(20)); // select next phase
        p = curr_pair.get().second;
        theta = curr_pair.get().first - t;
      }
    }
    else
    {
      t = t + theta;
      curr_pair = signal->predictState(t + boost::posix_time::milliseconds(20)); // select next phase
      p = curr_pair.get().second;
      theta = curr_pair.get().first - t;
    }
  }

  if (t <= eet)
  {
    double cycle_duration = signal->fixed_cycle_duration.total_milliseconds()/1000.0;
    if (cycle_duration < 0.001) //if it is a dynamic traffic signal not fixed
      cycle_duration = lanelet::time::toSec(signal->recorded_time_stamps.back().first) - lanelet::time::toSec(signal->recorded_start_time_stamps.front());
    
    t = t + lanelet::time::durationFromSec(std::floor((eet - t).total_milliseconds()/1000.0/cycle_duration) * cycle_duration); //fancy logic was needed to compile
    curr_pair = signal->predictState(t + boost::posix_time::milliseconds(20)); // select next phase
    p = curr_pair.get().second;
    theta = curr_pair.get().first - t;
    while ( t < eet || p != lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED)
    {
      if ( p == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED && eet - t < theta)
      {
        t = eet;
        theta = theta - (eet - t);
      }
      else
      {
        t = t + theta;
        curr_pair = signal->predictState(t + boost::posix_time::milliseconds(20)); // select next phase
        p = curr_pair.get().second;
        theta = curr_pair.get().first - t;
      }
    }
  }

  return rclcpp::Time(lanelet::time::toSec(t) * 1e9);
}

double LCIStrategicPlugin::get_trajectory_smoothing_activation_distance(double time_remaining_at_free_flow, double full_cycle_duration, double current_speed, double speed_limit, double departure_speed, double max_accel, double max_decel) const
{
  // TSMO USE CASE 2: Figure 7. Trajectory smoothing solution Case 2. Subsituted a+ as max_accel and solved for inflection_speed
  double accel_ratio = max_accel / max_decel;
  double remaining_time = time_remaining_at_free_flow - full_cycle_duration / 2;
  double inflection_speed = (max_accel * remaining_time - accel_ratio * departure_speed + current_speed)/ (1 - accel_ratio);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "ENTER TRAJ CALC: time_remaining_at_free_flow: " << time_remaining_at_free_flow << ", full_cycle_duration: " << full_cycle_duration << ", inflection_speed: " << inflection_speed);

  if (remaining_time < 0)
    return -1;

  if (inflection_speed > 0 && inflection_speed <= speed_limit + epsilon_ && inflection_speed >= departure_speed - epsilon_)
  {
    // kinematic equation to find distance of acceleration + deceleration
    // (vf^2 - vi^2)/2a = d
    double d = (std::pow(inflection_speed, 2) - std::pow (current_speed, 2)) / (2 * max_accel) +  (std::pow(departure_speed, 2) - std::pow(inflection_speed, 2)) / (2 * max_decel);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "calculated distance WITHOUT cruising: " << d);
    return d;
  }
  else //there must be cruising
  {
    // acceleration and deceleration parts must reach maximum speed
    // kinematic equation: t = (vf - vi)/ a where vf = 0
    double decel_time = (current_speed - speed_limit) / max_decel;
    double accel_time = (speed_limit - current_speed) / max_accel;
    double cruising_time = remaining_time - decel_time - accel_time;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "decel_time: " << decel_time << ", accel_time: " << accel_time << ", cruising_time: " << cruising_time);
    double d = (std::pow(speed_limit, 2) - std::pow (current_speed, 2)) / (2 * max_accel) +  (std::pow(departure_speed, 2) - std::pow(speed_limit, 2)) / (2 * max_decel) + cruising_time * speed_limit;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "calculated distance with cruising: " <<  d << ", accel_seg: " << (std::pow(speed_limit, 2) - std::pow (current_speed, 2)) / (2 * max_accel) << 
                      ", cruising: " << + cruising_time * speed_limit << ", decel_seg:" << (std::pow(departure_speed, 2) - std::pow(speed_limit, 2)) / (2 * max_decel));
    return d;
  }
}

rclcpp::Duration LCIStrategicPlugin::get_earliest_entry_time(double remaining_distance, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const
{
  double x = remaining_distance;
  double x2 = get_distance_to_accel_or_decel_once(current_speed, departure_speed, max_accel, max_decel);
  double x1 = get_distance_to_accel_or_decel_twice(free_flow_speed, current_speed, departure_speed, max_accel, max_decel);
  double v_hat = get_inflection_speed_value(x, x1, x2, free_flow_speed, current_speed, departure_speed, max_accel, max_decel);
  
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "x: " << x << ", x2: " << x2 << ", x1: " << x1 << ", v_hat: " << v_hat);

  if (v_hat <= config_.algo_minimum_speed - epsilon_ || isnan(v_hat))
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected that v_hat is smaller than allowed!!!: " << v_hat);
    v_hat = config_.algo_minimum_speed;
  }

  if (v_hat >= free_flow_speed + epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Detected that v_hat is Bigger than allowed!!!: " << v_hat);    
    v_hat = free_flow_speed;
  }

  rclcpp::Duration t_accel(0,0);
  if ( x < x2 && current_speed > departure_speed)
  {
    t_accel = rclcpp::Duration(0.0);
  }
  else
  {
    t_accel = rclcpp::Duration(std::max((v_hat - current_speed) / max_accel, 0.0) * 1e9);
  }
  rclcpp::Duration t_decel(0,0);
  if ( x < x2 && current_speed < departure_speed)
  {
    t_decel = rclcpp::Duration(0.0);
  }
  else
  {
    if (x < x2)
    {
      t_decel = rclcpp::Duration(std::max((v_hat - current_speed) / max_decel, 0.0) * 1e9);

    }
    else
    {
      t_decel = rclcpp::Duration(std::max((departure_speed - v_hat) / max_decel, 0.0) * 1e9);
    }
  }

  rclcpp::Duration t_cruise(0,0);
  if (x1 <= x)
  {
    t_cruise = rclcpp::Duration(std::max((x - x1)/v_hat, 0.0) * 1e9);
  }
  else
  {
    t_cruise = rclcpp::Duration(0.0);
  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "t_accel: " <<  t_accel.seconds() << ", t_cruise: " << t_cruise.seconds() << ", t_decel: " << t_decel.seconds());
  return t_accel + t_cruise + t_decel;

}

std::tuple<rclcpp::Time, bool, bool> LCIStrategicPlugin::get_final_entry_time_and_conditions(const VehicleState& current_state, const rclcpp::Time& earliest_entry_time, lanelet::CarmaTrafficSignalPtr traffic_light)
{
  rclcpp::Time nearest_green_entry_time = rclcpp::Time(0);
  bool is_entry_time_within_green_or_tbd = false;
  bool in_tbd = true;

  if (config_.enable_carma_streets_connection ==false || scheduled_enter_time_ == 0) //UC2
  {
    nearest_green_entry_time = get_nearest_green_entry_time(current_state.stamp, earliest_entry_time, traffic_light) 
                                          + rclcpp::Duration(EPSILON * 1e9); //0.01sec more buffer since green_light algorithm's timestamp picks the previous signal - Vehicle Estimation
    is_entry_time_within_green_or_tbd = true; 
  }
  else if(config_.enable_carma_streets_connection ==true && scheduled_enter_time_ != 0 ) // UC3
  {
    nearest_green_entry_time = rclcpp::Time(std::max(earliest_entry_time.seconds(), (scheduled_enter_time_)/1000.0) * 1e9) + rclcpp::Duration(EPSILON * 1e9); //Carma Street 
    
    // check if scheduled_enter_time_ is inside the available states interval
    size_t i = 0;
    

    for (auto pair : traffic_light->recorded_time_stamps)
    {
      if (lanelet::time::timeFromSec(nearest_green_entry_time.seconds()) < pair.first)
      {
        if (pair.second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED) 
        {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "ET is inside the GREEN phase! where starting time: " << std::to_string(lanelet::time::toSec(traffic_light->recorded_start_time_stamps[i])) 
            << ", ending time of that green signal is: " << std::to_string(lanelet::time::toSec(pair.first)));
          is_entry_time_within_green_or_tbd = true;
        }
        else
        {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Vehicle should plan cruise and stop as ET is inside the RED or YELLOW phase! where starting time: " << std::to_string(lanelet::time::toSec(traffic_light->recorded_start_time_stamps[i])) 
            << ", ending time of that green signal is: " << std::to_string(lanelet::time::toSec(pair.first)));
          is_entry_time_within_green_or_tbd = false;
        }

        in_tbd = false;
        break;
      }
      i++;
    }

    if (in_tbd)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "ET is inside TBD phase! where starting time: " << std::to_string(lanelet::time::toSec(traffic_light->recorded_time_stamps.back().first)));
      is_entry_time_within_green_or_tbd = true;
    }

  }
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "nearest_green_entry_time: " << nearest_green_entry_time.get_clock_type());
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "current_state.stamp: " << current_state.stamp.get_clock_type());

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "nearest_green_entry_time: " << std::to_string(nearest_green_entry_time.seconds()) << ", with : " << std::to_string((nearest_green_entry_time - current_state.stamp).seconds())  << " seconds left at: " << std::to_string(current_state.stamp.seconds()));
  
  if (nearest_green_entry_time_cached_) 
  { // always pick later of buffered green entry time, or earliest entry time
    nearest_green_entry_time = rclcpp::Time(std::max(nearest_green_entry_time.seconds(), nearest_green_entry_time_cached_.get().seconds()) * 1e9);
  }

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "After accounting for cached - nearest_green_entry_time: " << std::to_string(nearest_green_entry_time.seconds()) << ", with : " << std::to_string((nearest_green_entry_time - current_state.stamp).seconds())  << " seconds left at: " << std::to_string(current_state.stamp.seconds()));
  
  if (!nearest_green_entry_time_cached_ && is_entry_time_within_green_or_tbd) 
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Applying green_light_buffer for the first time and caching! nearest_green_entry_time (without buffer):" << std::to_string(nearest_green_entry_time.seconds()) << ", and earliest_entry_time: " << std::to_string(earliest_entry_time.seconds()));
    // save first calculated nearest_green_entry_time + buffer to compare against in the future as nearest_green_entry_time changes with earliest_entry_time
    
    // check if it needs buffer below:
    rclcpp::Time early_arrival_time_green_et =
        nearest_green_entry_time - rclcpp::Duration(config_.green_light_time_buffer * 1e9);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "early_arrival_time_green_et: " << std::to_string(early_arrival_time_green_et.seconds()));

    auto early_arrival_state_green_et_optional = traffic_light->predictState(lanelet::time::timeFromSec(early_arrival_time_green_et.seconds()));

    if (!validLightState(early_arrival_state_green_et_optional, early_arrival_time_green_et))
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve give signal...");
      return std::make_tuple(rclcpp::Time(0), is_entry_time_within_green_or_tbd, in_tbd);
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "early_arrival_state_green_et: " << early_arrival_state_green_et_optional.get().second);

    bool can_make_early_arrival  = (early_arrival_state_green_et_optional.get().second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED);
   
    // nearest_green_entry_time is by definition on green, so only check early_arrival
    if (can_make_early_arrival)  // Green light with Certainty
    {
      nearest_green_entry_time_cached_ = nearest_green_entry_time;  //don't apply buffer if ET is in green
    }  
    else //buffer is needed
    {
      // below logic stores correct buffered timestamp into nearest_green_entry_time_cached_ to be used later
      
      rclcpp::Time nearest_green_signal_start_time = rclcpp::Time(0);
      if (traffic_light->fixed_cycle_duration.total_milliseconds()/1000.0 > 1.0) // UC2
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "UC2 Handling");
        auto normal_arrival_state_green_et_optional = traffic_light->predictState(lanelet::time::timeFromSec(nearest_green_entry_time.seconds()));

        if (!validLightState(normal_arrival_state_green_et_optional, nearest_green_entry_time))
        {
          RCLCPP_ERROR_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Unable to resolve give signal...");
          return std::make_tuple(rclcpp::Time(0), is_entry_time_within_green_or_tbd, in_tbd);
        }

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "normal_arrival_signal_end_time: " << std::to_string(lanelet::time::toSec(normal_arrival_state_green_et_optional.get().first)));
        
        // nearest_green_signal_start_time = normal_arrival_signal_end_time (green guaranteed) - green_signal_duration
        nearest_green_signal_start_time = rclcpp::Time(lanelet::time::toSec(normal_arrival_state_green_et_optional.get().first - traffic_light->signal_durations[lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED]) * 1e9);
      }
      else  // UC3
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "UC3 Handling");
        
        for (size_t i = 0; i < traffic_light->recorded_start_time_stamps.size(); i++)
        {
          if (traffic_light->recorded_time_stamps[i].second == lanelet::CarmaTrafficSignalState::PROTECTED_MOVEMENT_ALLOWED && 
            lanelet::time::timeFromSec(nearest_green_entry_time.seconds()) < traffic_light->recorded_time_stamps[i].first ) // Make sure it is in correct GREEN phase there are multiple
          {
            nearest_green_signal_start_time = rclcpp::Time(lanelet::time::toSec(traffic_light->recorded_start_time_stamps[i]) * 1e9); 
            break;
          }
        }

        if (nearest_green_signal_start_time == rclcpp::Time(0)) //in tdb
        {
          nearest_green_signal_start_time = rclcpp::Time(lanelet::time::toSec(traffic_light->recorded_time_stamps.back().first) * 1e9); 
        }
      }

      // If ET is within green or TBD, it should always aim for at least minimum of "start_time of green or tdb + green_buffer" for safety
  
      nearest_green_entry_time_cached_ = nearest_green_signal_start_time + rclcpp::Duration((config_.green_light_time_buffer + EPSILON) * 1e9);
      
      // EPSILON=0.01 is there because if predictState's input exactly falls on ending_time it picks the previous state.
      //For example, if 0 - 10s is GREEN, and 10 - 12s is YELLOW, checking exactly 10.0s will return GREEN,
      //but 10.01s will return YELLOW. This 0.01 convention is used throughout the file, so thought it is better
      //to keep it consistent and probably too detailed for the user to think about, which is why it is not included in the buffer.
      //Actually including in the buffer doesn't work because it uses that same buffer to check early and late. If buffer is 2s and 
      //green starts at 10s, it will check +/-2s from 12s. If the buffer was 2.01s and green starts at 10s again, it checks +/-2.01 
      //from 12.01, so both checks 10s.
      
    }

    nearest_green_entry_time = rclcpp::Time(std::max(nearest_green_entry_time.seconds(), nearest_green_entry_time_cached_.get().seconds()) * 1e9);
  }

  if (nearest_green_entry_time_cached_ && nearest_green_entry_time > nearest_green_entry_time_cached_.get())
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "Earliest entry time... has gone past the cashed entering time. nearest_green_entry_time_cached_ (which can also be TBD):" << std::to_string(nearest_green_entry_time_cached_.get().seconds()) << ", and earliest_entry_time: " << std::to_string(earliest_entry_time.seconds()));
  }
  return std::make_tuple(nearest_green_entry_time, is_entry_time_within_green_or_tbd, in_tbd);
}

double LCIStrategicPlugin::get_inflection_speed_value(double x, double x1, double x2, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const
{  
  if (x >= x1)
  {
    return free_flow_speed;
  }
  else if (x1 > x && x >= x2)
  {
    return std::sqrt((2 * x * max_accel * max_decel + max_decel * std::pow(current_speed, 2) - max_accel * (std::pow(departure_speed, 2)))/(max_decel - max_accel));
  }
  else if (x2 > x)
  {
    if (current_speed <= departure_speed)
    {
      return std::sqrt(2 * x * max_accel + std::pow(current_speed, 2));
    }
    else
    {
      return std::sqrt(2 * x * max_decel + std::pow(current_speed, 2));
    }
  }
  
}

double LCIStrategicPlugin::calc_estimated_entry_time_left(double entry_dist, double current_speed, double departure_speed) const
{
  double t_entry = 0;
  // t = 2 * d / (v_i + v_f)
  // from TSMO USE CASE 2 Algorithm Doc - Figure 4. Equation: Estimation of t*_nt
  t_entry = 2*entry_dist/(current_speed + departure_speed);
  return t_entry;
}

//// NEW EQUATIONS START
BoundaryDistances LCIStrategicPlugin::get_delta_x(double v0, double v1, double v_max, double v_min, double a_max, double a_min)
{
  BoundaryDistances distances;
  
  distances.dx1 = ((pow(v_max, 2) - pow(v0, 2)) / (2 * a_max)) + ((pow(v1, 2) - pow(v_max, 2)) / (2 * a_min));
  if (v1 > v0)
    distances.dx2 = ((pow(v1, 2) - pow(v0, 2)) / (2 * a_max));
  else
    distances.dx2 = ((pow(v1, 2) - pow(v0, 2)) / (2 * a_min));

  distances.dx3 = ((pow(v_min, 2) - pow(v0, 2)) / (2 * a_min)) + ((pow(v1, 2) - pow(v_min, 2)) / (2 * a_max));
  distances.dx4 = ((pow(v_min, 2) - pow(v0, 2)) / (2 * a_min));
  distances.dx5 = - pow(v0, 2) / (2 * a_min);

  return distances;
}

void LCIStrategicPlugin::print_params(TrajectoryParams params)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "\n");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "t0: " << std::to_string(params.t0_));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "v0: " << params.v0_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "x0: " << params.x0_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "t1: " << std::to_string(params.t1_));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "v1: " << params.v1_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "x1: " << params.x1_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "a1: " << params.a1_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "t2: " << std::to_string(params.t2_));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "v2: " << params.v2_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "x2: " << params.x2_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "a2: " << params.a2_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "t3: " << std::to_string(params.t3_));
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "v3: " << params.v3_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "x3: " << params.x3_);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "a3: " << params.a3_);

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "\n");
}

void LCIStrategicPlugin::print_boundary_distances(BoundaryDistances delta_xs)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "\n");
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "dx1: " << delta_xs.dx1);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "dx2: " << delta_xs.dx2);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "dx3: " << delta_xs.dx3);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "dx4: " << delta_xs.dx4);
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "dx5: " << delta_xs.dx5 << "\n");
}

std::vector<TrajectoryParams> LCIStrategicPlugin::get_boundary_traj_params(double t, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx, BoundaryDistances boundary_distances)
{
  double dx1 = boundary_distances.dx1;
  double dx2 = boundary_distances.dx2;
  double dx3 = boundary_distances.dx3;
  double dx4 = boundary_distances.dx4;
  double dx5 = boundary_distances.dx5;
  TrajectoryParams traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8;

  // t1, t2, t3
  if (dx < dx2)
  {
    traj1 = boundary_accel_or_decel_incomplete_upper(t, v0, v1, a_max, a_min, x0, x_end, dx);
    traj2 = traj1;
    traj3 = traj1;
  }
  else if (dx < dx1)
  {
    traj1 = boundary_accel_nocruise_notmaxspeed_decel(t, v0, v1, a_max, a_min, x0, x_end, dx);
    traj2 = traj1;
    traj3 = boundary_accel_or_decel_complete_upper(t, v0, v1, x0, x_end, dx);
  }
  else
  {
    traj1 = boundary_accel_cruise_maxspeed_decel(t, v0, v1, v_max, a_max, a_min, x0, x_end, dx);
    traj2 = boundary_accel_nocruise_maxspeed_decel(t, v0, v1, v_max, a_max, a_min, x0, x_end, dx);
    traj3 = boundary_accel_or_decel_complete_upper(t, v0, v1, x0, x_end, dx);
  }
  // t4, t5, t6, t7
  if (dx < dx4)
  {
    traj4 = traj1;
    traj5 = traj1;
    traj6 = boundary_decel_incomplete_lower(t, v0, a_min, x0, x_end, dx);
    traj7 = traj6;
  }
  else if (dx < dx3)
  {
    if (dx < dx2)
    {
      traj4 = traj1;
    }
    else
    {
      traj4 = boundary_decel_nocruise_notminspeed_accel(t, v0, v1, v_min, a_max, a_min, x0, x_end, dx);
    }
    traj5 = traj4;
    traj6 = boundary_decel_nocruise_minspeed_accel_incomplete(t, v0, v_min, a_max, a_min, x0, x_end, dx);
    traj7 = boundary_decel_cruise_minspeed(t, v0, v_min, a_min, x0, x_end, dx);
    
  }
  else
  {
    traj4 = boundary_decel_nocruise_minspeed_accel_complete(t, v0, v1, v_max, v_min, a_max, a_min, x0, x_end, dx);
    traj5 = boundary_decel_cruise_minspeed_accel(t, v0, v1, v_min, a_max, a_min, x0, x_end, dx);
    traj6 = traj5;
    traj7 = boundary_decel_cruise_minspeed(t, v0, v_min, a_min, x0, x_end, dx);
  }
  // t8
  if (dx < dx4)
  {
    traj8 = traj6;
  }
  else if (dx < dx5)
  {
    traj8 = boundary_decel_incomplete_lower(t, v0, a_min, x0, x_end, dx);
  }
  else
  {
    traj8 = boundary_decel_cruise_minspeed_decel(t, v0, v_min, a_min, x0, x_end, dx);
  }

  return {traj1, traj2, traj3, traj4, traj5, traj6, traj7, traj8};
}

TrajectoryParams LCIStrategicPlugin::get_ts_case(double t, double et, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx, BoundaryDistances boundary_distances, std::vector<TrajectoryParams> params)
{
  double dx1 = boundary_distances.dx1;
  double dx2 = boundary_distances.dx2;
  double dx3 = boundary_distances.dx3;
  double dx4 = boundary_distances.dx4;
  double dx5 = boundary_distances.dx5;
  
  if (params.size() != 8)
  {
    throw std::invalid_argument("Not enough trajectory paras given! Given size: " + std::to_string(params.size()));
  }

  TrajectoryParams traj1 = params[0];
  TrajectoryParams traj2 = params[1];
  TrajectoryParams traj3 = params[2];
  TrajectoryParams traj4 = params[3];
  TrajectoryParams traj5 = params[4];
  TrajectoryParams traj6 = params[5];
  TrajectoryParams traj7 = params[6];
  TrajectoryParams traj8 = params[7];
  TrajectoryParams veh_traj;
  veh_traj.is_algorithm_successful = true;

  if (traj1.t3_ <= et && et < traj2.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 1");
    veh_traj = ts_case1(t, et, v0, v1, v_max, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_1;
  }
  else if (traj2.t3_ <= et && et < traj3.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 2");
    veh_traj = ts_case2(t, et, v0, v1, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_2;
  }
  else if (traj3.t3_ <= et && et < traj4.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 3");
    veh_traj = ts_case3(t, et, v0, v1, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_3;
  }
  else if (traj4.t3_ <= et && et < traj5.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 4");
    veh_traj = ts_case4(t, et, v0, v1, v_min, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_4;
  }
  else if (traj5.t3_ <= et && et < traj6.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 5");
    veh_traj = ts_case5(t, et, v0, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_5;
  }
  else if (traj6.t3_ <= et && et < traj7.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 6");
    veh_traj = ts_case6(t, et, v0, v_min, a_min, x0, x_end, dx, dx3, traj6);
    veh_traj.case_num = CASE_6;
  }
  else if (traj7.t3_ <= et && et <= traj8.t3_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 7");
    veh_traj = ts_case7(t, et, v0, v_min, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_7;
  }
  else
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CALCULATED: case 8");
    veh_traj = ts_case8(dx, dx5, traj8);
    veh_traj.case_num = CASE_8;
  }

  return veh_traj;
}


TrajectoryParams LCIStrategicPlugin::ts_case1(double t, double et, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;
  double nom1 = 2 * dx * (((1 - (a_max / a_min)) * v_max) + ((a_max / a_min) * v1) - v0);
  double nom2 = dt * (((1 - (a_max / a_min)) * pow(v_max, 2)) + ((a_max / a_min) * pow(v1, 2)) - pow(v0, 2));
  double den = pow(v_max - v0, 2) - ((a_max / a_min) * pow(v_max - v1, 2));

  if (den <= epsilon_ && den >= -epsilon_)
    throw std::invalid_argument("CASE1: Received den near zero..." + std::to_string(den));

  double tc = (nom1 - nom2) / den;

  traj.v1_ = v_max;

  if (dt - tc <= epsilon_ && dt - tc >= -epsilon_)
    throw std::invalid_argument("CASE1: Received dt - tc near zero..." + std::to_string(dt - tc));

  traj.a1_ = (((1 - (a_max / a_min)) * v_max) + ((a_max / a_min) * v1) - v0) / (dt - tc);

  if (traj.a1_ <= accel_epsilon_ && traj.a1_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE1: Received traj.a1_ near zero...");
    traj.t1_ = traj.t0_ + ((dt - tc) * (a_max / (a_min + a_max)));
    traj.x1_ = traj.x0_ + (v_max * (traj.t1_ - traj.t0_));    
  }
  else
  {
    traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
    traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));
  }
  traj.a2_ = 0;
  traj.v2_ = v_max;
  traj.t2_ = traj.t1_ + tc;
  traj.x2_ = traj.x1_ + (v_max * tc);

  traj.t3_ = et;
  traj.a3_ = traj.a1_ * (a_min / a_max);
  traj.v3_ = v1;
  traj.x3_ = x_end;

  return traj;

}
TrajectoryParams LCIStrategicPlugin::ts_case2(double t, double et, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;

  if (dt <= epsilon_ && dt >= -epsilon_)
    throw std::invalid_argument("CASE2: Received dt near zero..." + std::to_string(dt));

  double sqr1 = pow(1 - (a_max / a_min), 2) * pow(dx / dt, 2);
  double sqr2 = (1 - (a_max / a_min)) * (((a_max / a_min) * v1 * (v1 - (2 * dx / dt))) + (v0 * ((2 * dx / dt) - v0)));
  double v_hat = (dx / dt) + (sqrt(sqr1 - sqr2) / (1 - (a_max / a_min)));

  traj.v1_ = v_hat;
  traj.a1_ = (((1 - (a_max / a_min)) * v_hat) + ((a_max / a_min) * v1) - v0) / dt;

  if (traj.a1_ <= accel_epsilon_ && traj.a1_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE2: Received traj.a1_ near zero...");
    traj.t1_ = traj.t0_ + (dt * (a_max / (a_min + a_max)));
    traj.x1_ = traj.x0_ + (v_hat * (traj.t1_ - traj.t0_));        
  }
  else
  {
    traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
    traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));
  }

  traj.v2_ = v1;
  traj.a2_ = traj.a1_ * a_min / a_max;

  if (traj.a2_ <= accel_epsilon_ && traj.a2_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE2: Received traj.a2_ near zero...");
    traj.t2_ = traj.t1_ + (dt * (a_min / (a_min + a_max)));
    traj.x2_ = traj.x1_ + (v_hat * (traj.t2_ - traj.t1_));    
  }
  else
  {
    traj.t2_ = traj.t1_ + ((traj.v2_ - traj.v1_) / traj.a2_);
    traj.x2_ = x_end;
  }

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}
TrajectoryParams LCIStrategicPlugin::ts_case3(double t, double et, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;
  
  if (dt <= epsilon_ && dt >= -epsilon_)
    throw std::invalid_argument("CASE3: Received dt near zero..." + std::to_string(dt));

  double sqr1 = pow((a_max / a_min) - 1, 2) * pow(dx / dt, 2);
  double sqr2 = ((a_max / a_min) - 1) * ((v1 * (v1 - (2 * dx / dt))) + ((a_max / a_min) * v0 * ((2 * dx / dt) - v0)));
  double v_hat = (dx / dt) + (sqrt(sqr1 - sqr2) / ((a_max / a_min) - 1));

  traj.v1_ = v_hat;
  traj.a1_ = (((1 - (a_min / a_max)) * v_hat) + ((a_min / a_max) * v1) - v0) / dt;
  
  if (traj.a1_ <= accel_epsilon_ && traj.a1_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE3: Received traj.a1_ near zero...");
    traj.t1_ = traj.t0_ + (dt * (a_max / (a_min + a_max)));
    traj.x1_ = traj.x0_ + (v_hat * (traj.t1_ - traj.t0_));    
  }
  else
  {
    traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
    traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));
  }

  traj.v2_ = v1;
  traj.a2_ = traj.a1_ * a_max / a_min;
  traj.t2_ = traj.t1_ + ((traj.v2_ - traj.v1_) / traj.a2_);
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}
TrajectoryParams LCIStrategicPlugin::ts_case4(double t, double et, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;
  double nom1 = 2 * dx * ((((a_max / a_min) - 1) * v_min) + v1 - ((a_max / a_min) * v0));
  double nom2 = dt * ((((a_max / a_min) - 1) * pow(v_min, 2)) + pow(v1, 2) - ((a_max / a_min) * pow(v0, 2)));
  double den = ((a_max / a_min) * pow(v_min - v0, 2)) - pow(v_min - v1, 2);

  if (den <= epsilon_ && den >= -epsilon_)
    throw std::invalid_argument("CASE4: Received den near zero..." + std::to_string(den));

  double tc = (nom1 - nom2) / den;

  traj.v1_ = v_min;

  if (dt - tc <= epsilon_ && dt - tc >= -epsilon_)
    throw std::invalid_argument("CASE4: Received dt - tc near zero..." + std::to_string(dt - tc));

  traj.a1_ = (((1 - (a_min / a_max)) * v_min) + ((a_min / a_max) * v1) - v0) / (dt - tc);

  if (traj.a1_ <= accel_epsilon_ && traj.a1_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE4: Received traj.a1_ near zero...");
    traj.t1_ = traj.t0_ + ((dt - tc) * (a_min / (a_min + a_max)));
    traj.x1_ = traj.x0_ + (v_min * (traj.t1_ - traj.t0_));    
  }
  else
  {
    traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
    traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));
  }

  traj.v2_ = v_min;
  traj.a2_ = 0;
  traj.t2_ = traj.t1_ + tc;
  traj.x2_ = traj.x1_ + (v_min * tc);

  traj.t3_ = et;
  traj.a3_ = traj.a1_ * a_max / a_min;
  traj.v3_ = v1;
  traj.x3_ = x_end;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::ts_case5(double t, double et, double v0, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;
  double sqr = ((a_max / a_min) - 1) * ((2 * a_min * (dx - (v0 * dt))) - pow(a_min * dt, 2));
  double v_hat = (v0 + (a_min * dt)) - (sqrt(sqr) / ((a_max / a_min) - 1));
  double v_p = ((1 - (a_max / a_min)) * v_hat) + ((a_max / a_min) * v0) + (a_max * dt);

  traj.v1_ = v_hat;
  traj.a1_ = a_min;

  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.v2_ = v_p;
  traj.a2_ = a_max;
  traj.t2_ = traj.t1_ + ((traj.v2_ - traj.v1_) / traj.a2_);
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::ts_case6(double t, double et, double v0, double v_min, double a_min, double x0, double x_end, double dx, double dx3, TrajectoryParams traj6)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = et - t;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  double tc;

  if (dx <= dx3)
    tc = 0;
  else
    tc = traj6.t2_ - traj6.t1_;

  traj.v2_ = v_min;
  traj.a2_ = 0;
  traj.t2_ = traj.t1_ + tc;
  traj.x2_ = traj.x1_ + (v_min * tc);

  double dt_p = dt - (traj.t1_ - traj.t0_) - tc;

  if (dt_p <= epsilon_ && dt_p >= -epsilon_)
    throw std::invalid_argument("CASE6: Received dt_p near zero..." + std::to_string(dt_p));

  double v_p = ((2 * a_min * (dx - (v_min * tc))) - (pow(v_min, 2) - pow(v0, 2)) - (v_min * dt_p * a_min)) / (dt_p * a_min);

  traj.v3_ = v_p;
  traj.a3_ = (v_p - v_min) / dt_p;
  traj.t3_ = et;
  traj.x3_ = x_end;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::ts_case7(double t, double et, double v0, double v_min, double a_min, double x0, double x_end, double dx)
{ 
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  double dt = et - t;
  double v_p = v_min - sqrt(pow(v_min - v0, 2) - (2 * a_min * ((v_min * dt) - dx)));
  double dt_p = (v_p - v_min) / a_min;

  if (dt_p <= epsilon_ && dt_p >= -epsilon_)
    throw std::invalid_argument("CASE7: Received dt_p near zero..." + std::to_string(dt_p));

  double tc = dt - ((v_p - v0) / a_min);

  traj.v2_ = v_min;
  traj.a2_ = 0;
  traj.t2_ = traj.t1_ + tc;
  traj.x2_ = traj.x1_ + (v_min * tc);

  traj.v3_ = v_p;
  traj.a3_ = (v_p - v_min) / dt_p;
  traj.t3_ = et;
  traj.x3_ = x_end;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::ts_case8(double dx, double dx5, TrajectoryParams traj8)
{ 
  TrajectoryParams traj = traj8;
  if (dx < dx5)
  { 
    traj.is_algorithm_successful = false;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "CASE8: Not within safe stopping distance originally planned!");
  }
  return traj;
}


TrajectoryParams LCIStrategicPlugin::boundary_accel_or_decel_incomplete_upper(double t, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double t_end;

  if (v0 <= v1 + epsilon_)
    t_end = t + (sqrt(pow(v0, 2) + (2 * a_max * dx)) - v0)/a_max;
  else
    t_end = t + (sqrt(pow(v0, 2) + (2 * a_min * dx)) - v0) / a_min;

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.t1_ = t_end;

  if (v0 <= v1 + epsilon_)
  {
    traj.a1_ = a_max;
    traj.v1_ = sqrt(pow(v0, 2) + (2 * a_max * dx));
  }
  else
  {
    traj.a1_ = a_min;
    traj.v1_ = sqrt(pow(v0, 2) + (2 * a_min * dx));
  }

  traj.x1_ = x_end;

  traj.t2_ = traj.t1_;
  traj.a2_ = 0;
  traj.v2_ = traj.v1_;
  traj.x2_ = traj.x1_;

  traj.t3_ = traj.t1_;
  traj.a3_ = 0;
  traj.v3_ = traj.v1_;
  traj.x3_ = traj.x1_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_accel_nocruise_notmaxspeed_decel(double t, double v0, double v1, double a_max, double a_min, double x0, double x_end, double dx)
{ 
 
  double v_hat = sqrt(((2 * dx * a_max * a_min) + (a_min * pow(v0, 2)) - (a_max * pow(v1, 2))) / (a_min - a_max));
  double t_end = t + ((v_hat * (a_min - a_max)) - (v0 * a_min) + (v1 * a_max)) / (a_max * a_min);

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_hat;
  traj.a1_ = a_max;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);

  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.t2_ = t_end;
  traj.a2_ = a_min;
  traj.v2_ = v1;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;

}

TrajectoryParams LCIStrategicPlugin::boundary_accel_cruise_maxspeed_decel(double t, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double t_end = t + (dx / v_max) + (pow(v_max - v0, 2) / (2 * a_max * v_max)) - (pow(v1 - v_max, 2) / (2 * a_min * v_max));

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_max;
  traj.a1_ = a_max;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.v2_ = v_max;
  traj.a2_ = 0;
  traj.t2_ = t_end - ((v1 - v_max) / a_min);
  traj.x2_ = x_end - ((pow(v1, 2) - pow(v_max, 2)) / (2 * a_min));

  traj.t3_ = t_end;
  traj.a3_ = a_min;
  traj.v3_ = v1;
  traj.x3_ = x_end;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_accel_nocruise_maxspeed_decel(double t, double v0, double v1, double v_max, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double nom = (v_max - v0) + ((a_max / a_min) * (v1 - v_max));
  double den = (pow(v_max, 2) - pow(v0, 2)) + ((a_max / a_min) * (pow(v1, 2) - pow(v_max, 2)));

  if (den <= epsilon_ && den >= -epsilon_)
    throw std::invalid_argument("boundary_accel_nocruise_maxspeed_decel: Received den near zero..." + std::to_string(den));
  
  double t_end = t + (2 * dx * nom / den);

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = t_end - t;
  double tc = 0;

  traj.v1_ = v_max;

  if (dt - tc <= epsilon_ && dt - tc >= -epsilon_)
    throw std::invalid_argument("boundary_accel_nocruise_maxspeed_decel: Received dt - tc near zero..." + std::to_string(dt - tc));

  traj.a1_ = (((1 - (a_max / a_min)) * v_max) + ((a_max / a_min) * v1) - v0) / (dt - tc);

  if (traj.a1_ <= accel_epsilon_ && traj.a1_ >= -accel_epsilon_)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("lci_strategic_plugin"), "boundary_accel_nocruise_maxspeed_decel: Received traj.a1_ near zero...");
    traj.t1_ = traj.t0_ + (dt * (a_max / (a_min + a_max)));
    traj.x1_ = traj.x0_ + (v_max * (traj.t1_ - traj.t0_));    
  }
  else
  {
    traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
    traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));
  }

  traj.t2_ = t_end;
  traj.a2_ = ((((a_min / a_max) - 1) * v_max) + v1 - ((a_min / a_max) * v0)) / (dt - tc);
  traj.v2_ = v1;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_accel_or_decel_complete_upper(double t, double v0, double v1, double x0, double x_end, double dx)
{ 
  if (v0 + v1 <= epsilon_ && v0 + v1 >= -epsilon_)
    throw std::invalid_argument("boundary_accel_or_decel_complete_upper: Received v0 + v1 near zero..." + std::to_string(v0 + v1));
  
  double t_end = t + ((2 * dx) / (v0 + v1));

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.t1_ = t_end;

  if (dx <= epsilon_ && dx >= -epsilon_)
    throw std::invalid_argument("boundary_accel_or_decel_complete_upper: Received dx near zero..." + std::to_string(dx));

  traj.a1_ = (pow(v1, 2) - pow(v0, 2)) / (2 * dx);
  traj.v1_ = v1;
  traj.x1_ = x_end;

  traj.t2_ = traj.t1_;
  traj.a2_ = 0;
  traj.v2_ = traj.v1_;
  traj.x2_ = traj.x1_;

  traj.t3_ = traj.t1_;
  traj.a3_ = 0;
  traj.v3_ = traj.v1_;
  traj.x3_ = traj.x1_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_decel_nocruise_notminspeed_accel(double t, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double v_hat = sqrt(((2 * dx * a_max * a_min) + (a_max * pow(v0, 2)) - (a_min * pow(v1, 2))) / (a_max - a_min));
  double t_end = t + ((v_hat * (a_max - a_min)) - (v0 * a_max) + (v1 * a_min)) / (a_max * a_min);

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_hat;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.t2_ = t_end;
  traj.a2_ = a_max;
  traj.v2_ = v1;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_decel_nocruise_minspeed_accel_incomplete(double t, double v0, double v_min, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double sqr = sqrt((2 * a_max * dx) - ((pow(v_min, 2) - pow(v0, 2)) * (a_max / a_min)) + pow(v_min, 2));

  double t_end = t + ((sqr - v_min) / a_max) + ((v_min - v0) / a_min);

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + (traj.v1_ - traj.v0_) / a_min;
  traj.x1_ = traj.x0_ + (pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * a_min);

  traj.t2_ = t_end;
  traj.a2_ = a_max;
  traj.v2_ = (traj.a2_ * (traj.t2_ - traj.t1_)) + traj.v1_;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_decel_nocruise_minspeed_accel_complete(double t, double v0, double v1, double v_max, double v_min, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double nom = (v1 - v_min) + ((a_max / a_min) * (v_min - v0));
  double den = (pow(v1, 2) - pow(v_min, 2)) + ((a_max / a_min) * (pow(v_min, 2) - pow(v0, 2)));

  if (den <= epsilon_ && den >= -epsilon_)
    throw std::invalid_argument("boundary_decel_nocruise_minspeed_accel_complete: Received den near zero..." + std::to_string(den));

  double t_end = t + (2 * dx * nom / den);
  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  double dt = t_end - t;
  double tc = 0;

  traj.v1_ = v_min;

  if (dt - tc <= epsilon_ && dt - tc >= -epsilon_)
    throw std::invalid_argument("boundary_decel_nocruise_minspeed_accel_complete: Received dt - tc near zero..." + std::to_string(dt - tc));
    
  traj.a1_ = (((1 - (a_min / a_max)) * v_min) + ((a_min / a_max) * v1) - v0) / (dt - tc);
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.t2_ = t_end;
  traj.a2_ = ((((a_max / a_min) - 1) * v_min) + v1 - ((a_max / a_min) * v0)) / (dt - tc);
  traj.v2_ = v1;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = 0;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}
TrajectoryParams LCIStrategicPlugin::boundary_decel_cruise_minspeed_accel(double t, double v0, double v1, double v_min, double a_max, double a_min, double x0, double x_end, double dx)
{ 
  double t_end = t + (dx / v_min) + ((pow(v_min - v0, 2)) / (2 * a_min * v_min)) - ((pow(v1 - v_min, 2)) / (2 * a_max * v_min));

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.v2_ = v_min;
  traj.a2_ = 0;
  traj.t2_ = t_end - ((v1 - v_min) / a_max);
  traj.x2_ = x_end - ((pow(v1, 2) - pow(v_min, 2)) / (2 * a_max));

  traj.t3_ = t_end;
  traj.a3_ = a_max;
  traj.v3_ = v1;
  traj.x3_ = x_end;

  return traj;
}

TrajectoryParams LCIStrategicPlugin::boundary_decel_cruise_minspeed(double t, double v0, double v_min, double a_min, double x0, double x_end, double dx)
{ 
  double t_end = t + (dx / v_min) + (pow(v_min - v0, 2) / (2 * a_min * v_min));

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.v2_ = v_min;
  traj.a2_ = 0;
  traj.t2_ = t_end;
  traj.x2_ = x_end;

  traj.t3_ = traj.t2_;
  traj.a3_ = traj.a2_;
  traj.v3_ = traj.v2_;
  traj.x3_ = traj.x2_;

  return traj;
}
TrajectoryParams LCIStrategicPlugin::boundary_decel_incomplete_lower(double t, double v0, double a_min, double x0, double x_end, double dx)
{ 
  double t_end = t + (sqrt(pow(v0, 2) + (2 * a_min * dx)) - v0) / a_min;

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.t1_ = t_end;
  traj.v1_ = sqrt(pow(v0, 2) + (2 * a_min * dx));
  traj.a1_ = a_min;
  traj.x1_ = x_end;

  traj.t2_ = traj.t1_;
  traj.a2_ = 0;
  traj.v2_ = traj.v1_;
  traj.x2_ = traj.x1_;

  traj.t3_ = traj.t1_;
  traj.a3_ = 0;
  traj.v3_ = traj.v1_;
  traj.x3_ = traj.x1_;

  return traj;
}
TrajectoryParams LCIStrategicPlugin::boundary_decel_cruise_minspeed_decel(double t, double v0, double v_min, double a_min, double x0, double x_end, double dx)
{ 
  double t_end = t + (dx / v_min) + (v0 * (v0 - (2 * v_min)) / (2 * a_min * v_min));

  TrajectoryParams traj;

  traj.t0_ = t;
  traj.v0_ = v0;
  traj.x0_ = x0;

  traj.v1_ = v_min;
  traj.a1_ = a_min;
  traj.t1_ = traj.t0_ + ((traj.v1_ - traj.v0_) / traj.a1_);
  traj.x1_ = traj.x0_ + ((pow(traj.v1_, 2) - pow(traj.v0_, 2)) / (2 * traj.a1_));

  traj.a2_ = 0;
  traj.v2_ = v_min;
  traj.t2_ = t_end - ((0 - traj.v2_) / a_min);
  traj.x2_ = x_end - ((0 - pow(traj.v2_, 2)) / (2 * a_min));

  traj.t3_ = t_end;
  traj.a3_ = a_min;
  traj.v3_ = 0;
  traj.x3_ = x_end;

  return traj;
}


//// NEW EQUATIONS END
}  // namespace lci_strategic_plugin