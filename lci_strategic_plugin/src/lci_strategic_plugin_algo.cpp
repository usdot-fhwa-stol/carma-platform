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
  
  ROS_DEBUG_STREAM("x: " << x << ", x2: " << x2 << ", x1: " << x1 << ", v_hat: " << v_hat);

  if (v_hat <= config_.algo_minimum_speed - epsilon_ || isnan(v_hat))
  {
    ROS_DEBUG_STREAM("Detected that v_hat is smaller than allowed!!!: " << v_hat);
    v_hat = config_.algo_minimum_speed;
  }

  if (v_hat >= free_flow_speed + epsilon_)
  {
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
    t_accel = ros::Duration(std::max((v_hat - current_speed) / max_accel, 0.0));
  }
  ros::Duration t_decel;
  if ( x < x2 && current_speed < departure_speed)
  {
    t_decel = ros::Duration(0.0);
  }
  else
  {
    if (x < x2)
    {
      t_decel = ros::Duration(std::max((v_hat - current_speed) / max_decel, 0.0));

    }
    else
    {
      t_decel = ros::Duration(std::max((departure_speed - v_hat) / max_decel, 0.0));
    }
  }

  ros::Duration t_cruise;
  if (x1 <= x)
  {
    t_cruise = ros::Duration(std::max((x - x1)/v_hat, 0.0));
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
  ROS_DEBUG_STREAM("\n");
  ROS_DEBUG_STREAM("t0: " << std::to_string(params.t0_));
  ROS_DEBUG_STREAM("v0: " << params.v0_);
  ROS_DEBUG_STREAM("x0: " << params.x0_);

  ROS_DEBUG_STREAM("t1: " << std::to_string(params.t1_));
  ROS_DEBUG_STREAM("v1: " << params.v1_);
  ROS_DEBUG_STREAM("x1: " << params.x1_);
  ROS_DEBUG_STREAM("a1: " << params.a1_);

  ROS_DEBUG_STREAM("t2: " << std::to_string(params.t2_));
  ROS_DEBUG_STREAM("v2: " << params.v2_);
  ROS_DEBUG_STREAM("x2: " << params.x2_);
  ROS_DEBUG_STREAM("a2: " << params.a2_);

  ROS_DEBUG_STREAM("t3: " << std::to_string(params.t3_));
  ROS_DEBUG_STREAM("v3: " << params.v3_);
  ROS_DEBUG_STREAM("x3: " << params.x3_);
  ROS_DEBUG_STREAM("a3: " << params.a3_);

  ROS_DEBUG_STREAM("\n");
}

void LCIStrategicPlugin::print_boundary_distances(BoundaryDistances delta_xs)
{
  ROS_DEBUG_STREAM("\n");
  ROS_DEBUG_STREAM("dx1: " << delta_xs.dx1);
  ROS_DEBUG_STREAM("dx2: " << delta_xs.dx2);
  ROS_DEBUG_STREAM("dx3: " << delta_xs.dx3);
  ROS_DEBUG_STREAM("dx4: " << delta_xs.dx4);
  ROS_DEBUG_STREAM("dx5: " << delta_xs.dx5 << "\n");
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
    ROS_DEBUG_STREAM("CALCULATED: case 1");
    veh_traj = ts_case1(t, et, v0, v1, v_max, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_1;
  }
  else if (traj2.t3_ <= et && et < traj3.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 2");
    veh_traj = ts_case2(t, et, v0, v1, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_2;
  }
  else if (traj3.t3_ <= et && et < traj4.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 3");
    veh_traj = ts_case3(t, et, v0, v1, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_3;
  }
  else if (traj4.t3_ <= et && et < traj5.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 4");
    veh_traj = ts_case4(t, et, v0, v1, v_min, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_4;
  }
  else if (traj5.t3_ <= et && et < traj6.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 5");
    veh_traj = ts_case5(t, et, v0, a_max, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_5;
  }
  else if (traj6.t3_ <= et && et < traj7.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 6");
    veh_traj = ts_case6(t, et, v0, v_min, a_min, x0, x_end, dx, dx3, traj6);
    veh_traj.case_num = CASE_6;
  }
  else if (traj7.t3_ <= et && et <= traj8.t3_)
  {
    ROS_DEBUG_STREAM("CALCULATED: case 7");
    veh_traj = ts_case7(t, et, v0, v_min, a_min, x0, x_end, dx);
    veh_traj.case_num = CASE_7;
  }
  else
  {
    ROS_DEBUG_STREAM("CALCULATED: case 8");
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
    ROS_DEBUG_STREAM("CASE1: Received traj.a1_ near zero...");
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
    ROS_DEBUG_STREAM("CASE2: Received traj.a1_ near zero...");
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
    ROS_DEBUG_STREAM("CASE2: Received traj.a2_ near zero...");
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
    ROS_DEBUG_STREAM("CASE3: Received traj.a1_ near zero...");
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
    ROS_DEBUG_STREAM("CASE4: Received traj.a1_ near zero...");
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
    ROS_DEBUG_STREAM("CASE8: Not within safe stopping distance originally planned!");
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
    ROS_DEBUG_STREAM("boundary_accel_nocruise_maxspeed_decel: Received traj.a1_ near zero...");
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