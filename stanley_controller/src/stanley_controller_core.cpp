/*
 * Copyright 2020 AutonomouStuff, LLC. All Rights Reserved.
 *
 * For license details, see:
 * https://autonomoustuff.com/software-license-agreement/
 *
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stanley_controller/stanley_controller_core.h"

#include <vector>
#include <string>

namespace stanley_controller
{
StanleyController::StanleyController()
  : nh_("")
  , pnh_("~")
  , steer_cmd_prev_(0.0)
  , lateral_error_prev_(0.0)
  , heading_error_prev_(0.0)
  , my_position_ok_(false)
  , my_velocity_ok_(false)
  , my_steering_ok_(false)
{
  pnh_.param("update_rate", update_rate_, 30.0);
  pnh_.param("enable_path_smoothing", enable_path_smoothing_, true);
  pnh_.param("enable_yaw_recalculation", enable_yaw_recalculation_, false);
  pnh_.param("path_filter_moving_ave_num", path_filter_moving_ave_num_, 35);
  pnh_.param("path_smoothing_times", path_smoothing_times_, 1);
  pnh_.param("curvature_smoothing_num", curvature_smoothing_num_, 35);
  pnh_.param("traj_resample_dist", traj_resample_dist_, 0.1);  // [m]
  pnh_.param("output_interface", output_interface_, std::string("all"));

  /* stanley parameters */
  pnh_.param("kp_yaw_error", kp_yaw_error_, 1.0);
  pnh_.param("kd_yaw_error", kd_yaw_error_, 0.15);
  pnh_.param("kp_lateral_error", kp_lateral_error_, 1.0);
  pnh_.param("kd_steer", kd_steer_, 0.0);
  pnh_.param("k_soft", k_soft_, 10.0);
  pnh_.param("preview_window", preview_window_, 10);

  /* vehicle model setup */
  double mass_fl, mass_fr, mass_rl, mass_rr, cf, cr;
  nh_.param("vehicle_info/wheel_base", wheelbase_, 2.789);
  pnh_.param("mass_fl", mass_fl, 550.0);
  pnh_.param("mass_fr", mass_fr, 550.0);
  pnh_.param("mass_rl", mass_rl, 550.0);
  pnh_.param("mass_rr", mass_rr, 550.0);
  pnh_.param("cf", cf, 155494.663);
  pnh_.param("cr", cr, 155494.663);
  const double mass_f = mass_fl + mass_fr;
  const double mass_r = mass_rl + mass_rr;
  const double mass = mass_f + mass_r;
  const double lf = wheelbase_ * (1.0 - mass_f / mass);
  const double lr = wheelbase_ - lf;

  k_ag_ = mass / (cf * (1 + lf / lr));

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(1.0/update_rate_), &StanleyController::controlTimerCallback, this);
  std::string out_twist, out_ctrl_cmd, in_vehicle_status, in_waypoints, in_selfpose;
  pnh_.param("out_twist_name", out_twist, std::string("/twist_raw"));
  pnh_.param("out_ctrl_cmd_name", out_ctrl_cmd, std::string("/ctrl_raw"));
  pnh_.param("in_waypoints_name", in_waypoints, std::string("/base_waypoints"));
  pnh_.param("in_selfpose_name", in_selfpose, std::string("/current_pose"));
  pnh_.param("in_vehicle_status_name", in_vehicle_status, std::string("/vehicle_status"));
  pub_twist_cmd_ = nh_.advertise<geometry_msgs::TwistStamped>(out_twist, 1);
  pub_steer_vel_ctrl_cmd_ = nh_.advertise<autoware_msgs::ControlCommandStamped>(out_ctrl_cmd, 1);
  pub_lat_err_ = nh_.advertise<std_msgs::Float32>("/lateral_tracking_error", 1);
  sub_ref_path_ = nh_.subscribe(in_waypoints, 1, &StanleyController::callbackRefPath, this);
  sub_pose_ = nh_.subscribe(in_selfpose, 1, &StanleyController::callbackPose, this);
  sub_vehicle_status_ = nh_.subscribe(in_vehicle_status, 1, &StanleyController::callbackVehicleStatus, this);

  pub_ref_traj_marker_ = nh_.advertise<visualization_msgs::Marker>("ref_traj_viz", 1);
};

void StanleyController::controlTimerCallback(const ros::TimerEvent& te)
{
  if (ref_traj_.size() == 0 || !my_position_ok_ || !my_velocity_ok_ || !my_steering_ok_)
  {
    ROS_DEBUG_THROTTLE(1,
                      "[Stanley] ref_traj_.size() = %d, my_position_ok_ = %d,  my_velocity_ok_ = %d,  my_steering_ok_ "
                      "= %d",
                      ref_traj_.size(), my_position_ok_, my_velocity_ok_, my_steering_ok_);
    publishControlCommands(0.0, 0.0, steer_cmd_prev_);  // publish brake
    return;
  }

  /* control command */
  double vel_cmd = 0.0;
  double acc_cmd = 0.0;
  double steer_cmd = 0.0;

  /* update state error */
  updateStateError();

  double heading_error_cmd = kp_yaw_error_ * heading_error_;

  double lateral_error_cmd = std::atan2(kp_lateral_error_ * lateral_error_, k_soft_ + vehicle_status_.twist.linear.x);

  // Equation (8), hard to tune
  double steady_state_yaw_cmd = k_ag_ * vehicle_status_.twist.linear.x * (ref_pt_velocity_ * ref_pt_curvature_);
  // double steady_state_yaw_cmd = 0.0;

  // to avoid abrupt change of yaw rate
  double yaw_rate_diff_cmd = kd_yaw_error_ * heading_error_rate_;

  // to avoid abrupt steering angle change
  // double steering_diff_cmd = kd_steer_ * (previous_steer_angle_ - current_steer_angle);
  double steering_diff_cmd = 0.0;

  // Equation (9)
  steer_cmd = -(heading_error_cmd - steady_state_yaw_cmd + lateral_error_cmd + yaw_rate_diff_cmd + steering_diff_cmd);

  // longitudinal control
  // Find a preview refrence point ahead of the current nearest reference point
  double preview_time = nearest_traj_time_ + preview_window_ / update_rate_;
  auto it_low = std::lower_bound(ref_traj_.relative_time.begin(), ref_traj_.relative_time.end(), preview_time);
  int preview_index = static_cast<int>(it_low - ref_traj_.relative_time.begin());

  // Find distance in between the nearest reference point to the preview reference point
  // better factor it into MPCTrajector class later.
  double s = 0.0;
  for (int i = nearest_traj_index_; i < preview_index; i++)
  {
    double dx = ref_traj_.x[i + 1] - ref_traj_.x[i];
    double dy = ref_traj_.y[i + 1] - ref_traj_.y[i];
    s += std::sqrt(dx * dx + dy * dy);
  }

  // assume constant accleration
  vel_cmd = ref_traj_.vx[preview_index];

  // todo: define a PID controller for acceleration command.
  acc_cmd = (vel_cmd * vel_cmd - vehicle_status_.twist.linear.x * vehicle_status_.twist.linear.x) / (2.0 * s);

  steer_cmd_prev_ = steer_cmd;           //< @brief steering command calculated in previous period
  lateral_error_prev_ = lateral_error_;  //< @brief previous lateral error for derivative
  heading_error_prev_ = heading_error_;  //< @brief previous lateral error for derivative

  publishControlCommands(vel_cmd, acc_cmd, steer_cmd);
};

// Given current pose, heading, velocity, and yaw rate,
// compute lateral_error, lateral_error_rate, heading_error, heading_error_rate
bool StanleyController::updateStateError()
{
  geometry_msgs::Pose nearest_pose;
  double dist_err = 0.0;
  if (!MPCUtils::calcNearestPoseInterp(ref_traj_, vehicle_status_.pose, &nearest_pose, &nearest_traj_index_, &dist_err,
                                       &heading_error_, &nearest_traj_time_))
  {
    ROS_WARN_THROTTLE(2, "Error in calculating nearest pose.");
    return false;
  };

  double dx = vehicle_status_.pose.position.x - nearest_pose.position.x;
  double dy = vehicle_status_.pose.position.y - nearest_pose.position.y;
  double sp_yaw = tf2::getYaw(nearest_pose.orientation);
  lateral_error_ = -sin(sp_yaw) * dx + cos(sp_yaw) * dy;

  lateral_error_rate_ = vehicle_status_.twist.linear.x * std::sin(heading_error_ - vehicle_status_.steering_angle_rad);

  heading_error_rate_ =
      vehicle_status_.twist.angular.x - ref_traj_.k[nearest_traj_index_] * ref_traj_.vx[nearest_traj_index_];

  ref_pt_velocity_ = ref_traj_.vx[nearest_traj_index_];
  ref_pt_curvature_ = ref_traj_.k[nearest_traj_index_];

  // Publish lateral tracking error
  std_msgs::Float32 lat_err_msg;
  lat_err_msg.data = lateral_error_;
  pub_lat_err_.publish(lat_err_msg);

  return true;
}

void StanleyController::callbackRefPath(const autoware_msgs::Lane::ConstPtr& msg)
{
  current_waypoints_ = *msg;

  MPCUtils::MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(current_waypoints_, &relative_time);

  /* resampling */
  MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(current_waypoints_, relative_time, traj_resample_dist_,
                                                          &traj);
  MPCUtils::convertEulerAngleToMonotonic(&traj.yaw);

  /* path smoothing */
  if (enable_path_smoothing_)
  {
    for (int i = 0; i < path_smoothing_times_; ++i)
    {
      if (!MPCUtils::filt_vector(path_filter_moving_ave_num_, &traj.x) ||
          !MPCUtils::filt_vector(path_filter_moving_ave_num_, &traj.y) ||
          !MPCUtils::filt_vector(path_filter_moving_ave_num_, &traj.yaw) ||
          !MPCUtils::filt_vector(path_filter_moving_ave_num_, &traj.vx))
      {
        ROS_WARN_THROTTLE(2, "Path callback: filtering error. stop filtering");
        return;
      }
    }
  }

  /* calculate yaw angle */
  if (enable_yaw_recalculation_)
  {
    MPCUtils::calcTrajectoryYawFromXY(&traj);
    MPCUtils::convertEulerAngleToMonotonic(&traj.yaw);
  }

  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(curvature_smoothing_num_, &traj);

  if (!traj.size())
  {
    ROS_ERROR_THROTTLE(1, "Path callback: trajectory size is undesired.");
    ROS_ERROR_THROTTLE(1, "size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu",
        traj.x.size(), traj.y.size(), traj.z.size(),
        traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
    return;
  }

  ref_traj_ = traj;

  /* publish trajectory for visualize */
  visualization_msgs::Marker markers;
  convertTrajToMarker(ref_traj_, &markers, "ref_traj", 0.0, 0.5, 1.0, 0.05);
  pub_ref_traj_marker_.publish(markers);
};

void StanleyController::convertTrajToMarker(const MPCUtils::MPCTrajectory& traj, visualization_msgs::Marker* marker,
                                            std::string ns, double r, double g, double b, double z)
{
  marker->points.clear();
  marker->header.frame_id = current_waypoints_.header.frame_id;
  marker->header.stamp = ros::Time();
  marker->ns = ns;
  marker->id = 0;
  marker->type = visualization_msgs::Marker::LINE_STRIP;
  marker->action = visualization_msgs::Marker::ADD;
  marker->scale.x = 0.15;
  marker->scale.y = 0.3;
  marker->scale.z = 0.3;
  marker->color.a = 0.9;
  marker->color.r = r;
  marker->color.g = g;
  marker->color.b = b;
  // display a waypoint at least every meter apart
  int step = static_cast<int>(1.0 / traj_resample_dist_);
  step = step < 1 ? 1 : step;
  for (unsigned int i = 0; i < traj.x.size(); i += step)
  {
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i) + z;
    marker->points.push_back(p);
  }
}

void StanleyController::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  vehicle_status_.header = msg->header;

  // center of front axle in relative to center of rear axle
  Eigen::Vector3d l_front(wheelbase_, 0.0, 0.0);
  Eigen::Vector3d g_rear(msg->pose.position.x, msg->pose.position.y, 0.0);
  Eigen::Quaternion<double> quaternion(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y,
                                       msg->pose.orientation.z);
  Eigen::Vector3d g_front = quaternion.toRotationMatrix() * l_front + g_rear;

  // find the position of center of front axle.
  vehicle_status_.pose = msg->pose;
  vehicle_status_.pose.position.x = g_front[0];
  vehicle_status_.pose.position.y = g_front[1];
  my_position_ok_ = true;
};

void StanleyController::callbackVehicleStatus(const autoware_msgs::VehicleStatus& msg)
{
  vehicle_status_.steering_angle_rad = msg.angle;

  // find the velocity of car along the orientation of front wheels which is needed
  // by stanley controller. Usually msg.speed is the car's longitudinal velocity.
  // We may need to define a parameter to tell the node whether the given speed is
  // longitudinal velocity or front wheel's speed.
  double v_r = amathutils::kmph2mps(msg.speed);
  vehicle_status_.twist.linear.x = v_r / cos(msg.angle);
  vehicle_status_.twist.angular.z = tan(msg.angle) * v_r / wheelbase_;

  my_steering_ok_ = true;
  my_velocity_ok_ = true;
};

void StanleyController::publishControlCommands(const double& vel_cmd, const double& acc_cmd,
                                               const double& steer_cmd) const
{
  const double omega_cmd = vehicle_status_.twist.linear.x * std::tan(steer_cmd) / wheelbase_;
  if (output_interface_ == "twist")
  {
    publishTwist(vel_cmd, omega_cmd);
  }
  else if (output_interface_ == "ctrl_cmd")
  {
    publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd);
  }
  else if (output_interface_ == "all")
  {
    publishTwist(vel_cmd, omega_cmd);
    publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd);
  }
  else
  {
    ROS_WARN_THROTTLE(2, "Control command interface is not appropriate");
  }
}

void StanleyController::publishTwist(const double& vel_cmd, const double& omega_cmd) const
{
  /* convert steering to twist */
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = "/base_link";
  twist.header.stamp = ros::Time::now();
  twist.twist.linear.x = vel_cmd;
  twist.twist.linear.y = 0.0;
  twist.twist.linear.z = 0.0;
  twist.twist.angular.x = 0.0;
  twist.twist.angular.y = 0.0;
  twist.twist.angular.z = omega_cmd;
  pub_twist_cmd_.publish(twist);
}

void StanleyController::publishCtrlCmd(const double& vel_cmd, const double& acc_cmd, const double& steer_cmd) const
{
  autoware_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "/base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.cmd.linear_velocity = vel_cmd;
  cmd.cmd.linear_acceleration = acc_cmd;
  cmd.cmd.steering_angle = steer_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}
}  // namespace stanley_controller
