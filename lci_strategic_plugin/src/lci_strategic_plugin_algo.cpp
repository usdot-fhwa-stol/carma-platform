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
#include "lci_strategic_plugin/lci_strategic_plugin.h"
#include "lci_strategic_plugin/lci_states.h"

namespace lci_strategic_plugin
{

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

ros::Time LCIStrategicPlugin::get_nearest_green_entry_time(const ros::Time& current_time, const ros::Time& earliest_entry_time, lanelet::CarmaTrafficSignalPtr signal, double minimum_required_green_time) const
{
  boost::posix_time::time_duration g =  lanelet::time::durationFromSec(minimum_required_green_time);         // provided by considering min headways of vehicles in front
  boost::posix_time::ptime t = lanelet::time::timeFromSec(current_time.toSec());                        // time variable
  boost::posix_time::ptime eet = lanelet::time::timeFromSec(earliest_entry_time.toSec());                        // earliest entry time

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

  return ros::Time(lanelet::time::toSec(t));
}

double LCIStrategicPlugin::get_trajectory_smoothing_activation_distance(double time_remaining_at_free_flow, double full_cycle_duration, double current_speed, double speed_limit, double departure_speed, double max_accel, double max_decel) const
{
  // TSMO USE CASE 2: Figure 7. Trajectory smoothing solution Case 2. Subsituted a+ as max_accel and solved for inflection_speed
  double accel_ratio = max_accel / max_decel;
  double remaining_time = time_remaining_at_free_flow - full_cycle_duration / 2;
  double inflection_speed = (max_accel * remaining_time - accel_ratio * departure_speed + current_speed)/ (1 - accel_ratio);
  ROS_DEBUG_STREAM("ENTER TRAJ CALC: time_remaining_at_free_flow: " << time_remaining_at_free_flow << ", full_cycle_duration: " << full_cycle_duration << ", inflection_speed: " << inflection_speed);

  if (remaining_time < 0)
    return -1;

  if (inflection_speed > 0 && inflection_speed <= speed_limit + epsilon_ && inflection_speed >= departure_speed - epsilon_)
  {
    // kinematic equation to find distance of acceleration + deceleration
    // (vf^2 - vi^2)/2a = d
    double d = (std::pow(inflection_speed, 2) - std::pow (current_speed, 2)) / (2 * max_accel) +  (std::pow(departure_speed, 2) - std::pow(inflection_speed, 2)) / (2 * max_decel);
    ROS_DEBUG_STREAM("calculated distance WITHOUT cruising: " << d);
    return d;
  }
  else //there must be cruising
  {
    // acceleration and deceleration parts must reach maximum speed
    // kinematic equation: t = (vf - vi)/ a where vf = 0
    double decel_time = (current_speed - speed_limit) / max_decel;
    double accel_time = (speed_limit - current_speed) / max_accel;
    double cruising_time = remaining_time - decel_time - accel_time;
    ROS_DEBUG_STREAM("decel_time: " << decel_time << ", accel_time: " << accel_time << ", cruising_time: " << cruising_time);
    double d = (std::pow(speed_limit, 2) - std::pow (current_speed, 2)) / (2 * max_accel) +  (std::pow(departure_speed, 2) - std::pow(speed_limit, 2)) / (2 * max_decel) + cruising_time * speed_limit;
    ROS_DEBUG_STREAM("calculated distance with cruising: " <<  d << ", accel_seg: " << (std::pow(speed_limit, 2) - std::pow (current_speed, 2)) / (2 * max_accel) << 
                      ", cruising: " << + cruising_time * speed_limit << ", decel_seg:" << (std::pow(departure_speed, 2) - std::pow(speed_limit, 2)) / (2 * max_decel));
    return d;
  }
}

ros::Duration LCIStrategicPlugin::get_earliest_entry_time(double remaining_distance, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const
{
  double x = remaining_distance;
  double x2 = get_distance_to_accel_or_decel_once(current_speed, departure_speed, max_accel, max_decel);
  double x1 = get_distance_to_accel_or_decel_twice(free_flow_speed, current_speed, departure_speed, max_accel, max_decel);
  double v_hat = get_inflection_speed_value(x, x1, x2, free_flow_speed, current_speed, departure_speed, max_accel, max_decel);
  
  ROS_DEBUG_STREAM("x: " << x << ", x2: " << x2 << ", x1: " << x1);

  if (v_hat <= config_.minimum_speed - epsilon_)
  {
    ROS_ERROR_STREAM("Detected that v_hat is smaller than allowed!!!: " << v_hat);
    ROS_DEBUG_STREAM("Detected that v_hat is smaller than allowed!!!: " << v_hat);
    
    v_hat = config_.minimum_speed;
  }

  if (v_hat >= free_flow_speed + epsilon_)
  {
    ROS_ERROR_STREAM("Detected that v_hat is Bigger than allowed!!!: " << v_hat);
    ROS_DEBUG_STREAM("Detected that v_hat is Bigger than allowed!!!: " << v_hat);
    
    v_hat = free_flow_speed;
  }

  ros::Duration t_accel;
  if ( x < x2 && current_speed > departure_speed)
  {
    t_accel = ros::Duration(0.0);
  }
  else
  {
    t_accel = ros::Duration((v_hat - current_speed) / max_accel);
  }
  ros::Duration t_decel;
  if ( x < x2 && current_speed < departure_speed)
  {
    t_decel = ros::Duration(0.0);
  }
  else
  {
    t_decel = ros::Duration((departure_speed - v_hat) / max_decel);
  }
  ros::Duration t_cruise;
  if (x1 <= x)
  {
    t_cruise = ros::Duration((x - x1)/v_hat);
  }
  else
  {
    t_cruise = ros::Duration(0.0);
  }
  ROS_DEBUG_STREAM("t_accel: " <<  t_accel << ", t_cruise: " << t_cruise << ", t_decel: " << t_decel);
  return t_accel + t_cruise + t_decel;

}

double LCIStrategicPlugin::get_inflection_speed_value(double x, double x1, double x2, double free_flow_speed, double current_speed, double departure_speed, double max_accel, double max_decel) const
{  
  if (x >= x1)
  {
    return free_flow_speed;
  }
  else if (x1 > x >= x2)
  {
    return std::sqrt(2 * x * max_accel * max_decel + max_decel * std::pow(current_speed, 2) - max_accel * (std::pow(departure_speed, 2))/(max_decel - max_accel));
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

double LCIStrategicPlugin::calc_speed_before_decel(double entry_time, double entry_dist, double current_speed, double departure_speed) const
{
  double speed_before_decel = 0;
  
  // from TSMO USE CASE 2 Algorithm Doc - Figure 7. Equation: Trajectory Smoothing Solution (Case 2)

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel_/max_comfort_decel_;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((1-a_r)^2*v_r^2 - (1-a_r)(a_r*v_f*(v_f-2*v_r) + v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow(1 - (acc_dec_ratio), 2) * pow(required_speed, 2) - (1 -acc_dec_ratio) *
                        (acc_dec_ratio * departure_speed * (departure_speed - 2 * required_speed) + current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term/(1 - a_r)
  speed_before_decel = required_speed + sqr_term/(1 - acc_dec_ratio);

  return speed_before_decel;
}

double LCIStrategicPlugin::calc_speed_before_accel(double entry_time, double entry_dist, double current_speed, double departure_speed) const
{
  double speed_before_accel = 0;

  // from TSMO USE CASE 2 Algorithm Doc - Figure 11. Equation: Trajectory Smoothing Solution (Case 3)
  
  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel_/max_comfort_decel_;
  // v_r = d / t
  double required_speed = entry_dist / entry_time;
  // sqrt_term  = sqrt((a_r - 1)^2*v_r^2 - (a_r-1)(v_f*(v_f-2*v_r) + a_r*v_i*(2*v_r - v_i)))
  double sqr_term = sqrt(pow((acc_dec_ratio - 1), 2) * pow(required_speed, 2) - (acc_dec_ratio - 1) *
                        (departure_speed * (departure_speed - 2 * required_speed) + acc_dec_ratio * current_speed * (2* required_speed - current_speed)));
  // v_e = v_r + sqrt_term / (a_r - 1)
  speed_before_accel = required_speed + sqr_term/(acc_dec_ratio - 1);

  return speed_before_accel;
}

SpeedProfileCase LCIStrategicPlugin::determine_speed_profile_case(double estimated_entry_time, double scheduled_entry_time, double speed_before_decel, double speed_before_accel, double speed_limit)
{
  SpeedProfileCase case_num;
  
  ROS_DEBUG_STREAM("estimated_entry_time: " << estimated_entry_time << ", and scheduled_entry_time: " << scheduled_entry_time);
  if (estimated_entry_time < scheduled_entry_time)
  {
    ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << ", and config_.minimum_speed: " << config_.minimum_speed);

    if (speed_before_accel < config_.minimum_speed)
    {
      case_num = DECEL_CRUISE_ACCEL;
    }
    else
    {
      case_num = DECEL_ACCEL;
    }
  }
  else
  {
    ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << ", and speed_limit: " << speed_limit);

    if (speed_before_decel > speed_limit)
    {
      case_num = ACCEL_CRUISE_DECEL;
    }
    else
    {
      case_num = ACCEL_DECEL;
    }
  }
  
  return case_num;
}

TrajectorySmoothingParameters LCIStrategicPlugin::get_parameters_for_accel_cruise_decel_speed_profile(double remaining_downtrack, double remaining_time, double starting_speed, double speed_before_decel, double speed_limit, double departure_speed)
{
  TrajectorySmoothingParameters params;
  params.is_algorithm_successful = true;

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel_/max_comfort_decel_;
  
  double t_cruise = 0.0; // Cruising Time Interval for Case 2. TSMO UC 2 Algorithm draft doc Figure 7.
  double t_c_nom = 0.0;
  double t_c_den = epsilon_;

  if (speed_before_decel > speed_limit)
  {
    ROS_DEBUG_STREAM("Detected that cruising is necessary. Changed speed_before_decel: " << speed_before_decel << ", to : " << speed_limit);
    speed_before_decel = speed_limit;

    // Cruising Time Interval Equation (case 1) obtained from TSMO UC 2 Algorithm draft doc Figure 8.
    // Nominator portion
    t_c_nom = 2 * remaining_downtrack * ((1 - acc_dec_ratio) * speed_before_decel + acc_dec_ratio * departure_speed - starting_speed) - 
                    remaining_time * ((1 - acc_dec_ratio) * pow(speed_before_decel, 2) + acc_dec_ratio * pow(departure_speed, 2) - pow(starting_speed, 2));
    
    // Denominator portion
    t_c_den = pow(speed_before_decel - starting_speed, 2) - acc_dec_ratio * pow(speed_before_decel - departure_speed, 2);
    
    if (t_c_den > -epsilon_ && t_c_den < epsilon_)
    {
      ROS_DEBUG_STREAM("WARN: Denominator of cruising time interval is too close to zero: " 
                        << t_c_den << ", t_c_nom: " << t_c_nom << ", which may indicate there is only cruising portion available. Returning without any change..."); 
      params.is_algorithm_successful = false;
      return params; 
    }
    
    t_cruise = t_c_nom / t_c_den;
  }
  // From TSMO USE CASE 2 Algorithm Doc - Figure 8. Equation: Trajectory Smoothing Solution (Case 1 and 2)

  ROS_DEBUG_STREAM("max_comfort_accel_: " << max_comfort_accel_ << "\n" <<
                   "max_comfort_decel_: " << max_comfort_decel_ << "\n" <<
                   "acc_dec_ratio: " << acc_dec_ratio << "\n" <<
                   "speed_limit: " << speed_limit);
  
  // Rest of the equations for acceleration rates and time intervals for when accelerating or decelerating 
  double a_acc = ((1 - acc_dec_ratio) * speed_before_decel + acc_dec_ratio * departure_speed - starting_speed) / (remaining_time - t_cruise);
  double a_dec = ((max_comfort_decel_ - max_comfort_accel_) * speed_before_decel + max_comfort_accel_ * departure_speed - max_comfort_decel_ * starting_speed) / (max_comfort_accel_ * (remaining_time - t_cruise));
  double t_acc = (speed_before_decel - starting_speed) / a_acc;
  double t_dec =  (departure_speed - speed_before_decel) / a_dec;

  ROS_DEBUG_STREAM("speed_before_decel: " << speed_before_decel << "\n" <<
                   "departure_speed: " << departure_speed << "\n" <<
                   "remaining_downtrack: " << remaining_downtrack << "\n" <<
                   "t_c_nom: " << t_c_nom << "\n" <<
                   "t_c_den: " << t_c_den << "\n" <<
                   "t_cruise: " << t_cruise << "\n" <<
                   "a_acc: " << a_acc << "\n" <<
                   "a_dec: " << a_dec << "\n" <<
                   "t_acc: " << t_acc << "\n" <<
                   "t_dec: " << t_dec);
  
  if (remaining_time - t_cruise < epsilon_ && remaining_time - t_cruise >= 0.0)
  {
    ROS_DEBUG_STREAM("WARN: Only Cruising is needed... therefore, no speed modification is required. Returning... ");
    params.is_algorithm_successful = false;
    return params;
  }
  else if (t_cruise < -epsilon_)
  {
    throw std::invalid_argument(std::string("Input parameters are not valid or do not qualify conditions " 
                                "of estimated_time >= scheduled_time (case 1 and 2)"));
  }

  // Checking route geometry start against start_dist and adjust profile
  double dist_accel;        //Distance over which acceleration happens
  double dist_cruise;     //Distance over which cruising happens
  double dist_decel;      //Distance over which deceleration happens

  //Use maneuver parameters to create speed profile
  //Kinematic: d = v_0 * t + 1/2 * a * t^2
  dist_accel = starting_speed * t_acc + 0.5 * a_acc * pow(t_acc, 2);
  dist_cruise = speed_before_decel * t_cruise;
  dist_decel = speed_before_decel * t_dec + 0.5 * a_dec * pow(t_dec, 2);

  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  if (a_acc < - epsilon_ || a_acc > max_comfort_accel_ || a_dec > epsilon_ || a_dec < max_comfort_decel_ || total_distance_needed > remaining_downtrack + 0.5 || //0.5 meter buffer
      dist_decel < - 0.1 || dist_cruise < -0.1 || dist_accel < - 0.1) //algorithm was not able to calculate valid values
  {
    ROS_DEBUG_STREAM("WARN: get_parameters_for_accel_cruise_decel_speed_profile was NOT successful...");
    params.is_algorithm_successful = false; 
  }

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  params.a_accel = a_acc;
  params.a_decel = a_dec;
  params.dist_accel = dist_accel;
  params.dist_cruise = dist_cruise;
  params.dist_decel = dist_decel;
  params.speed_before_decel = speed_before_decel;

  return params;
}

TrajectorySmoothingParameters LCIStrategicPlugin::get_parameters_for_decel_cruise_accel_speed_profile(double remaining_downtrack, double remaining_time, double starting_speed, double speed_before_accel, double minimum_speed, double departure_speed)
{
  TrajectorySmoothingParameters params;
  params.is_algorithm_successful = true;

  // a_r = a_acc / a_dec
  double acc_dec_ratio = max_comfort_accel_/max_comfort_decel_;
  
  double t_cruise = 0.0; // Cruising Time Interval for Case 4. TSMO UC 2 Algorithm draft doc Figure 12.
  double t_c_nom = 0.0;
  double t_c_den = epsilon_;

  if (speed_before_accel < config_.minimum_speed)
  {
    ROS_DEBUG_STREAM("Detected that cruising is necessary. Changed speed_before_accel: " << speed_before_accel << ", to : " << config_.minimum_speed);
    speed_before_accel = config_.minimum_speed;

    // Cruising Time Interval Equation (case 1) obtained from TSMO UC 2 Algorithm draft doc Figure 8.
    // Nominator portion
    t_c_nom = 2 * remaining_downtrack * ((acc_dec_ratio - 1) * speed_before_accel + departure_speed - acc_dec_ratio * starting_speed) - 
                    remaining_time * ((acc_dec_ratio - 1) * pow(speed_before_accel, 2) + pow(departure_speed, 2) - acc_dec_ratio * pow(starting_speed, 2));
    
    // Denominator portion
    t_c_den = acc_dec_ratio * pow(speed_before_accel - starting_speed, 2) - pow(speed_before_accel - departure_speed, 2);
    
    if (t_c_den > -epsilon_ && t_c_den < epsilon_)
    {
      ROS_DEBUG_STREAM("WARN: Denominator of cruising time interval is too close to zero: " 
                        << t_c_den << ", t_c_nom: " << t_c_nom << ", which may indicate there is only cruising portion available. Returning without any change..."); 
      params.is_algorithm_successful = false;
      return params;
    }
    
    t_cruise = t_c_nom / t_c_den;
  }
  // From TSMO USE CASE 2 Algorithm Doc - Figure 11 - 13. Equation: Trajectory Smoothing Solution (Case 1 and 2)

  ROS_DEBUG_STREAM("max_comfort_accel_: " << max_comfort_accel_ << "\n" <<
                   "max_comfort_decel_: " << max_comfort_decel_ << "\n" <<
                   "acc_dec_ratio: " << acc_dec_ratio << "\n" <<
                   "config_.minimum_speed: " << config_.minimum_speed);
  
  // Rest of the equations for acceleration rates and time intervals for when accelerating or decelerating 
  double a_acc = ((acc_dec_ratio - 1) * speed_before_accel + departure_speed - acc_dec_ratio * starting_speed) / (remaining_time - t_cruise);
  double a_dec = ((max_comfort_accel_ - max_comfort_decel_) * speed_before_accel + max_comfort_decel_ * departure_speed - max_comfort_accel_ * starting_speed) / (max_comfort_accel_ * (remaining_time - t_cruise));
  double t_acc = (departure_speed - speed_before_accel) / a_acc;
  double t_dec =  (speed_before_accel - starting_speed) / a_dec;

  ROS_DEBUG_STREAM("speed_before_accel: " << speed_before_accel << "\n" <<
                   "departure_speed: " << departure_speed << "\n" <<
                   "remaining_downtrack: " << remaining_downtrack << "\n" <<
                   "t_c_nom: " << t_c_nom << "\n" <<
                   "t_c_den: " << t_c_den << "\n" <<
                   "t_cruise: " << t_cruise << "\n" <<
                   "a_acc: " << a_acc << "\n" <<
                   "a_dec: " << a_dec << "\n" <<
                   "t_acc: " << t_acc << "\n" <<
                   "t_dec: " << t_dec);
  
  if (remaining_time - t_cruise < epsilon_ && remaining_time - t_cruise >= 0.0)
  {
    ROS_DEBUG_STREAM("WARN: Only Cruising is needed... therefore, no speed modification is required. Returning... ");
    params.is_algorithm_successful = false;
    return params;
  }
  else if (t_cruise < -epsilon_)
  {
    throw std::invalid_argument(std::string("Input parameters are not valid or do not qualify conditions " 
                                "of estimated_time < scheduled_time (case 3 and 4)"));
  }

  // Checking route geometry start against start_dist and adjust profile
  double dist_accel;        //Distance over which acceleration happens
  double dist_cruise;     //Distance over which cruising happens
  double dist_decel;      //Distance over which deceleration happens

  //Use maneuver parameters to create speed profile
  //Kinematic: d = v_0 * t + 1/2 * a * t^2
  dist_decel = starting_speed * t_dec + 0.5 * a_dec * pow(t_dec, 2);
  dist_cruise = speed_before_accel * t_cruise;
  dist_accel = speed_before_accel * t_acc + 0.5 * a_acc * pow(t_acc, 2);
  
  
  //Check calculated total dist against maneuver limits
  double total_distance_needed = dist_accel + dist_cruise + dist_decel;

  if (a_acc < -epsilon_ || a_acc > max_comfort_accel_ || a_dec > epsilon_ || a_dec < max_comfort_decel_ || total_distance_needed > remaining_downtrack + 0.5 || //0.5 meter buffer
      dist_decel < - 0.1 || dist_cruise < -0.1 || dist_accel < - 0.1) //algorithm was not able to calculate valid values
  {
    ROS_DEBUG_STREAM("WARN: get_parameters_for_decel_cruise_accel_speed_profile was NOT successful...");
    params.is_algorithm_successful = false; 
  }

  ROS_DEBUG_STREAM("total_distance_needed: " << total_distance_needed << "\n" <<
                  "dist_accel: " << dist_accel << "\n" <<
                  "dist_decel: " << dist_decel << "\n" <<
                  "dist_cruise: " << dist_cruise);

  params.a_accel = a_acc;
  params.a_decel = a_dec;
  params.dist_accel = dist_accel;
  params.dist_cruise = dist_cruise;
  params.dist_decel = dist_decel;
  params.speed_before_accel = speed_before_accel;
  
  return params;
}

TrajectorySmoothingParameters LCIStrategicPlugin::handleFailureCase(double starting_speed, double departure_speed, double remaining_downtrack, double remaining_time)
{
  //Requested maneuver needs to be modified to meet remaining_dist req
  //by trying to get close to the target_speed and remaining_time as much as possible
  TrajectorySmoothingParameters params;

  params.is_algorithm_successful = false;
  params.speed_before_accel = -1;
  params.speed_before_decel = -1;

  if (starting_speed >= departure_speed - epsilon_) //decelerate
  {
    params.a_accel = 0;
    params.a_decel = max_comfort_decel_;
    params.speed_before_decel = starting_speed;
    params.dist_accel = 0;
    params.dist_cruise = 0;
    params.dist_decel = remaining_downtrack;
    // kinematic: TODO
    params.modified_departure_speed = sqrt(pow(starting_speed, 2) - 2 * remaining_downtrack * max_comfort_decel_norm_);
    params.modified_remaining_time = (starting_speed - params.modified_departure_speed ) / max_comfort_decel_norm_;
    params.case_num = SpeedProfileCase::ACCEL_DECEL;
  }
  else //accelerate
  {
    params.a_accel = max_comfort_accel_;
    params.a_decel = 0;
    params.speed_before_accel = starting_speed;
    params.dist_accel = remaining_downtrack;
    params.dist_cruise = 0;
    params.dist_decel = 0;
    // kinematic: TODO
    params.modified_departure_speed = sqrt(2 * remaining_downtrack * max_comfort_accel_ + pow(starting_speed, 2));
    params.modified_remaining_time = (params.modified_departure_speed - starting_speed) / max_comfort_accel_;
    params.case_num = SpeedProfileCase::DECEL_ACCEL;
  }

  if (params.modified_departure_speed > departure_speed)
  {
    ROS_ERROR_STREAM("Trying different way");
    ROS_DEBUG_STREAM("Trying different way");
    TrajectorySmoothingParameters new_params;

    // kinematic: TODO
    double new_accel = 2 * (remaining_downtrack - starting_speed * remaining_time)/ (pow(remaining_time, 2));
    
    new_params.is_algorithm_successful = false;
    new_params.speed_before_accel = -1;
    new_params.speed_before_decel = -1;

    if (new_accel < -epsilon_)
    {
      new_params.a_decel = new_accel;
      new_params.speed_before_decel = starting_speed;
      new_params.dist_decel = remaining_downtrack;
      new_params.case_num = SpeedProfileCase::ACCEL_DECEL;
    }
    else
    {
      new_params.a_accel = new_accel;
      new_params.speed_before_accel = starting_speed;
      new_params.dist_accel = remaining_downtrack;
      new_params.case_num = SpeedProfileCase::DECEL_ACCEL;
    }

    new_params.modified_departure_speed = starting_speed + new_accel * remaining_time;
    new_params.dist_cruise = 0;
    new_params.modified_remaining_time = remaining_time;
    params = new_params;
  }
  // handle hard failure case such as nan
  if (isnan(params.modified_departure_speed) || params.modified_departure_speed < - epsilon_ ||
      params.modified_departure_speed > 35.7632 ) //80_mph
  {
    throw std::invalid_argument("Calculated departure speed is invalid: " + std::to_string(params.modified_departure_speed));
  }
  ROS_DEBUG_STREAM("WARN: Maneuver needed to be modified (due to negative dist) with new distance and accelerations: \n" << 
                "a_acc: " << params.a_accel << "\n" <<
                "a_dec: " << params.a_decel << "\n" <<
                "dist_accel: " << params.dist_accel << "\n" <<
                "dist_decel: " << params.dist_decel << "\n" <<
                "dist_cruise: " << params.dist_cruise << "\n" <<
                "modified_departure_speed: " << params.modified_departure_speed << "\n" <<
                "modified_remaining_time: " << params.modified_remaining_time << "\n" <<
                "case_num: " << params.case_num << "\n" <<
                "speed_before_accel: " << params.speed_before_accel << "\n" <<
                "speed_before_decel: " << params.speed_before_decel);
  return params;
}

}  // namespace lci_strategic_plugin