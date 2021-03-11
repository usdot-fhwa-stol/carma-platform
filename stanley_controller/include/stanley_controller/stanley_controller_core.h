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

/**
 * @file stanley_controller.h
 * @brief stanley controller class
 * @author Jilin Zhou
 * @date 2019.09.01
 */

#ifndef STANLEY_CONTROLLER_STANLEY_CONTROLLER_CORE_H
#define STANLEY_CONTROLLER_STANLEY_CONTROLLER_CORE_H

#include <limits>
#include <chrono>
#include <unistd.h>
#include <deque>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/Geometry>

#include <autoware_msgs/ControlCommandStamped.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/VehicleStatus.h>

#include <dynamic_reconfigure/server.h>
#include <stanley_controller/StanleyDynamicParamsConfig.h>

#include "stanley_controller/mpc_utils.h"
#include "stanley_controller/mpc_trajectory.h"

namespace stanley_controller
{
/**
 * @class
 * @brief calculate control command to follow reference waypoints
 */
class StanleyController
{
public:
  /**
   * @brief constructor
   */
  StanleyController();

  /**
   * @brief destructor
   */
  ~StanleyController() = default;

private:
  ros::NodeHandle nh_;                     //!< @brief ros node handle
  ros::NodeHandle pnh_;                    //!< @brief private ros node handle
  ros::Publisher pub_steer_vel_ctrl_cmd_;  //!< @brief topic publisher for control command
  ros::Publisher pub_twist_cmd_;           //!< @brief topic publisher for twist command
  ros::Publisher pub_lat_err_;             //!< @brief topic publisher for lateral error
  ros::Subscriber sub_ref_path_;           //!< @brief topic subscriber for reference waypoints
  ros::Subscriber sub_pose_;               //!< @brief subscriber for current pose
  ros::Subscriber sub_vehicle_status_;     //!< @brief subscriber for currrent vehicle status
  ros::Timer timer_control_;               //!< @brief timer for control command computation
  ros::Publisher pub_ref_traj_marker_;

  MPCUtils::MPCTrajectory ref_traj_;       //!< @brief reference trajectory to be followed
  autoware_msgs::Lane current_waypoints_;  //!< @brief current waypoints to be followed
  std::string output_interface_;           //!< @brief output command type

  /* parameters for control*/
  double update_rate_;  //!< @brief control frequency [s]
  double wheelbase_;    //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity

  /* parameters for path smoothing */
  bool enable_path_smoothing_;      //< @brief flag for path smoothing
  bool enable_yaw_recalculation_;   //< @brief flag for recalculation of yaw angle after resampling
  int path_filter_moving_ave_num_;  //< @brief param of moving average filter for path smoothing
  int path_smoothing_times_;        //< @brief number of times of applying path smoothing filter
  int curvature_smoothing_num_;     //< @brief point-to-point index distance used in curvature calculation
  double traj_resample_dist_;       //< @brief path resampling interval [m]

  /* Stanley controller tuning parameters */
  double kp_yaw_error_;
  double kd_yaw_error_;
  double kp_lateral_error_;
  double kd_steer_;
  double k_soft_;       //< @brief minimum speed during steering
  double k_ag_;         //< @brief for steady state yaw
  int preview_window_;  //< @brief lookahead window size for longitudinal control
  double yaw_offset_points_ = 0.0;

  struct VehicleStatus
  {
    std_msgs::Header header;     //< @brief header
    geometry_msgs::Pose pose;    //< @brief vehicle pose at the center of front axle
    geometry_msgs::Twist twist;  //< @brief vehicle velocity measured at the center of front axle
    double steering_angle_rad;   //< @brief vehicle steering angle
  };
  VehicleStatus vehicle_status_;  //< @brief vehicle status

  // state variables for lateral steering control
  double lateral_error_;       //< @brief current lateral error in between the ego vehicle to reference trajectory
  double lateral_error_rate_;  //< @brief current lateral error rate
  double heading_error_;       //< @brief current heading (yaw) error in between the ego vehicle's heading
                               //         to the heading of the nearest point on reference trajectory
  double heading_error_rate_;  //< @brief current heading error rate
  double nearest_traj_time_;
  uint32_t nearest_traj_index_;

  double steer_cmd_prev_;      //< @brief steering command calculated in previous period
  double lateral_error_prev_;  //< @brief previous lateral error for derivative
  double heading_error_prev_;  //< @brief previous lateral error for derivative

  /* flags */
  bool my_position_ok_;  //< @brief flag for validity of current pose
  bool my_velocity_ok_;  //< @brief flag for validity of current velocity
  bool my_steering_ok_;  //< @brief flag for validity of steering angle

  double ref_pt_velocity_;   //< @brief velocity of nearest point on reference trajectory
  double ref_pt_curvature_;  //< @brief curvature of nearest point on reference trajectory

  dynamic_reconfigure::Server<stanley_controller::StanleyDynamicParamsConfig> dynamic_param_server_;
  

  bool updateStateError();

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void controlTimerCallback(const ros::TimerEvent&);

  /**
   * @brief set current_waypoints_ with receved message
   */
  void callbackRefPath(const autoware_msgs::Lane::ConstPtr& msg);

  /**
   * @brief set vehicle_status_.pose with receved message
   */
  void callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg);

  /**
   * @brief set vehicle_status_.twist and vehicle_status_.tire_angle_rad with receved message
   */
  void callbackVehicleStatus(const autoware_msgs::VehicleStatus& msg);

  /**
   * @brief publish control command calculated by MPC
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   */
  void publishControlCommands(const double& vel_cmd, const double& acc_cmd, const double& steer_cmd) const;

  /**
   * @brief publish control command as geometry_msgs/TwistStamped type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] omega_cmd angular velocity command [rad/s] for vehicle control
   */
  void publishTwist(const double& vel_cmd, const double& omega_cmd) const;

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   */
  void publishCtrlCmd(const double& vel_cmd, const double& acc_cmd, const double& steer_cmd) const;

  /**
   * @brief convert MPCTraj to visualizaton marker for visualization
   */
  void convertTrajToMarker(const MPCUtils::MPCTrajectory& traj, visualization_msgs::Marker* markers, std::string ns,
                           double r, double g, double b, double z);


  /**
   * @brief Compensate for lag in the yaw response by forward shifting target yaw values
   * by a fixed nubmer of points
   * 
   * @param traj The MPCUtils::MPCTrajectory object to adjust, non-destructively
   * @param offset The number of points to shift by
   * 
   * @return A new MPCUtils::MPCTrajectory object containing the same data as 
   * traj with the yaw values shifted forward by offset points. The trajectory's
   * yaw values are backfilled with the last yaw value from the original traj.
   */
  MPCUtils::MPCTrajectory apply_response_lag(const MPCUtils::MPCTrajectory& traj, const int offset) const;
  void param_callback(stanley_controller::StanleyDynamicParamsConfig &config, uint32_t level);
};
}  // namespace stanley_controller
#endif  // STANLEY_CONTROLLER_STANLEY_CONTROLLER_CORE_H
