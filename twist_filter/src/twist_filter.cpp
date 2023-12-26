/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 * Modifications:
 * - Added limiting for longitudinal velocity per CARMA specifications. Refactored
 *   namespacing and header as needed to support unit testing.
 *   - Kyle Rush
 *   - 9/11/2020
 */

#include "twist_filter.hpp"
#include "velocity_limit.hpp"


namespace twist_filter
{

namespace std_ph = std::placeholders;

TwistFilter::TwistFilter(const rclcpp::NodeOptions &options)
    : carma_ros2_utils::CarmaLifecycleNode(options)
{

  // Declare parameters
  wheel_base_ = declare_parameter<double>("vehicle_wheel_base", wheel_base_);
  longitudinal_velocity_limit_ = declare_parameter<double>("config_speed_limit", longitudinal_velocity_limit_);
  longitudinal_accel_limit_ = declare_parameter<double>("vehicle_acceleration_limit", longitudinal_accel_limit_);
  lateral_accel_limit_ = declare_parameter<double>("vehicle_lateral_accel_limit", lateral_accel_limit_);
  lateral_jerk_limit_ = declare_parameter<double>("vehicle_lateral_jerk_limit", lateral_jerk_limit_);
  lowpass_gain_linear_x_ = declare_parameter<double>("lowpass_gain_linear_x", lowpass_gain_linear_x_);
  lowpass_gain_angular_z_ = declare_parameter<double>("lowpass_gain_angular_x", lowpass_gain_angular_z_);
  lowpass_gain_steering_angle_ = declare_parameter<double>("lowpass_gain_steering_angle", lowpass_gain_steering_angle_);
}

carma_ros2_utils::CallbackReturn TwistFilter::handle_on_configure(const rclcpp_lifecycle::State &prev_state)
{
  // get parameters
  get_parameter<double>("vehicle_wheel_base", wheel_base_);
  // parameters on private handles
  get_parameter<double>("config_speed_limit", longitudinal_velocity_limit_);
  longitudinal_velocity_limit_ = longitudinal_velocity_limit_ * 0.44704;

  get_parameter<double>("vehicle_acceleration_limit", longitudinal_accel_limit_);
  _lon_accel_limiter = LongitudinalAccelLimiter{
    std::min(longitudinal_accel_limit_, MAX_LONGITUDINAL_ACCEL_HARDCODED_LIMIT_M_S_2)};

  get_parameter<double>("vehicle_lateral_accel_limit", lateral_accel_limit_);
  get_parameter<double>("vehicle_lateral_jerk_limit", lateral_jerk_limit_);
  get_parameter<double>("lowpass_gain_linear_x", lowpass_gain_linear_x_);
  get_parameter<double>("lowpass_gain_angular_x", lowpass_gain_angular_z_);
  get_parameter<double>("lowpass_gain_steering_angle", lowpass_gain_steering_angle_);


  //Setup subscribers
  twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("twist_raw", 1, std::bind(&TwistFilter::TwistCmdCallback, this, std_ph::_1));
  ctrl_sub_ = create_subscription<autoware_msgs::msg::ControlCommandStamped>("ctrl_raw", 1, std::bind(&TwistFilter::CtrlCmdCallback, this, std_ph::_1));
  config_sub_ = create_subscription<autoware_config_msgs::msg::ConfigTwistFilter>("config/twist_filter", 10, std::bind(&TwistFilter::configCallback, this, std_ph::_1));

  //Setup publishers
  twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_cmd", 5);
  ctrl_pub_ = create_publisher<autoware_msgs::msg::ControlCommandStamped>("ctrl_cmd", 5);

  // Publishers on private handles
  twist_lacc_limit_debug_pub_ = create_publisher<std_msgs::msg::Float32>("~/limitation_debug/twist/lateral_accel", 5);
  twist_ljerk_limit_debug_pub_ = create_publisher<std_msgs::msg::Float32>("~/limitation_debug/twist/lateral_jerk", 5);
  ctrl_lacc_limit_debug_pub_ = create_publisher<std_msgs::msg::Float32>("~/limitation_debug/ctrl/lateral_accel", 5);
  ctrl_ljerk_limit_debug_pub_ = create_publisher<std_msgs::msg::Float32>("~/limitation_debug/ctrl/lateral_jerk", 5);
  twist_lacc_result_pub_ = create_publisher<std_msgs::msg::Float32>("~/result/twist/lateral_accel", 5);
  twist_ljerk_result_pub_ = create_publisher<std_msgs::msg::Float32>("~/result/twist/lateral_jerk", 5);
  ctrl_lacc_result_pub_ = create_publisher<std_msgs::msg::Float32>("~/result/ctrl/lateral_accel", 5);
  ctrl_ljerk_result_pub_ = create_publisher<std_msgs::msg::Float32>("~/result/ctrl/lateral_jerk", 5);


  return CallbackReturn::SUCCESS;

}


auto lowpass_filter()
{
  double y = 0.0;
  return [y](const double& x, const double& gain) mutable -> double
  {
    y = gain * y + (1 - gain) * x;
    return y;
  };
}

boost::optional<double> TwistFilter::calcLaccWithAngularZ(
  const double& lv, const double& az) const
{
  return az * lv;
}

boost::optional<double> TwistFilter::calcLjerkWithAngularZ(
  const double& lv, const double& az) const
{
  if (std::fabs(az_prev_.dt) < MIN_DURATION)
  {
    return boost::none;
  }
  return (az - az_prev_.val) * lv / az_prev_.dt;
}

boost::optional<double> TwistFilter::calcLaccWithSteeringAngle(
  const double& lv, const double& sa) const
{
  if (std::fabs(wheel_base_) < MIN_LENGTH)
  {
    return boost::none;
  }
  return lv * lv * std::tan(sa) / wheel_base_;
}

boost::optional<double> TwistFilter::calcLjerkWithSteeringAngle(
  const double& lv, const double& sa) const
{
  if (std::fabs(sa_prev_.dt) < MIN_DURATION ||
    std::fabs(wheel_base_) < MIN_LENGTH)
  {
    return boost::none;
  }
  return lv * lv *
    ((std::tan(sa) - std::tan(sa_prev_.val)) / sa_prev_.dt) / wheel_base_;
}

void TwistFilter::publishLateralResultsWithTwist(
  const geometry_msgs::msg::TwistStamped& msg) const
{
  const double lv = msg.twist.linear.x;
  const double az = msg.twist.angular.z;
  const auto lacc = calcLaccWithAngularZ(lv, az);
  const auto ljerk = calcLjerkWithAngularZ(lv, az);
  if (!lacc || !ljerk)
  {
    return;
  }
  std_msgs::msg::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc.get();
  ljerk_msg.data = ljerk.get();
  twist_lacc_result_pub_->publish(lacc_msg);
  twist_ljerk_result_pub_->publish(ljerk_msg);
}

void TwistFilter::publishLateralResultsWithCtrl(
  const autoware_msgs::msg::ControlCommandStamped& msg) const
{
  const double lv = msg.cmd.linear_velocity;
  const double sa = msg.cmd.steering_angle;
  const auto lacc = calcLaccWithSteeringAngle(lv, sa);
  const auto ljerk = calcLjerkWithSteeringAngle(lv, sa);
  if (!lacc || !ljerk)
  {
    return;
  }
  std_msgs::msg::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc.get();
  ljerk_msg.data = ljerk.get();
  ctrl_lacc_result_pub_->publish(lacc_msg);
  ctrl_ljerk_result_pub_->publish(ljerk_msg);
}

void TwistFilter::checkTwist(const geometry_msgs::msg::TwistStamped& msg)
{
  const double lv = msg.twist.linear.x;
  const double az = msg.twist.angular.z;
  const auto lacc = calcLaccWithAngularZ(lv, az);
  const auto ljerk = calcLjerkWithAngularZ(lv, az);

}

void TwistFilter::checkCtrl(const autoware_msgs::msg::ControlCommandStamped& msg)
{
  const double lv = msg.cmd.linear_velocity;
  const double sa = msg.cmd.steering_angle;
  const auto lacc = calcLaccWithSteeringAngle(lv, sa);
  const auto ljerk = calcLjerkWithSteeringAngle(lv, sa);

}


geometry_msgs::msg::TwistStamped
  TwistFilter::lateralLimitTwist(const geometry_msgs::msg::TwistStamped& msg)
{
  static bool init = false;

  geometry_msgs::msg::TwistStamped ts;
  ts = msg;

  rclcpp::Time t = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  az_prev_.dt = (t - az_prev_.time).seconds();
  const double lv = msg.twist.linear.x;
  double az = msg.twist.angular.z;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ts;
  }

  // lateral acceleration
  double lacc = calcLaccWithAngularZ(lv, az).get();
  // limit lateral acceleration
  if (std::fabs(lacc) > lateral_accel_limit_ && !is_stopping)
  {
    double sgn = lacc / std::fabs(lacc);
    double az_max = sgn * (lateral_accel_limit_) / lv;
    auto& clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 1, "Limit angular velocity by lateral acceleration: %f -> %f", az, az_max);
    az = az_max;
  }

  // lateral jerk
  double ljerk = calcLjerkWithAngularZ(lv, az).get();
  // limit lateral jerk
  if (std::fabs(ljerk) > lateral_jerk_limit_ && !is_stopping)
  {
    double sgn = ljerk / std::fabs(ljerk);
    double az_max =
      az_prev_.val + (sgn * lateral_jerk_limit_ / lv) * az_prev_.dt;
    auto& clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 1,"Limit angular velocity by lateral jerk: %f -> %f", az, az_max);
    az = az_max;
  }

  // update by lateral limitaion
  ts.twist.angular.z = az;

  // update lateral acceleration/jerk
  lacc = calcLaccWithAngularZ(lv, az).get();
  ljerk = calcLjerkWithAngularZ(lv, az).get();

  // for debug
  std_msgs::msg::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc;
  ljerk_msg.data = ljerk;
  twist_lacc_limit_debug_pub_->publish(lacc_msg);
  twist_ljerk_limit_debug_pub_->publish(ljerk_msg);

  return ts;
}

geometry_msgs::msg::TwistStamped
  TwistFilter::smoothTwist(const geometry_msgs::msg::TwistStamped& msg)
{
  static auto lp_lx = lowpass_filter();
  static auto lp_az = lowpass_filter();

  geometry_msgs::msg::TwistStamped ts;
  ts = msg;

  // apply lowpass filter to linear_x / angular_z
  ts.twist.linear.x = lp_lx(ts.twist.linear.x, lowpass_gain_linear_x_);
  ts.twist.angular.z = lp_az(ts.twist.angular.z, lowpass_gain_angular_z_);

  return ts;
}

autoware_msgs::msg::ControlCommandStamped
  TwistFilter::lateralLimitCtrl(const autoware_msgs::msg::ControlCommandStamped& msg)
{
  static bool init = false;

  autoware_msgs::msg::ControlCommandStamped ccs;
  ccs = msg;

  rclcpp::Time t = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  sa_prev_.dt = (t - sa_prev_.time).seconds();
  const double lv = msg.cmd.linear_velocity;
  double sa = msg.cmd.steering_angle;

  // skip first msg, check linear_velocity
  const bool is_stopping = (std::fabs(lv) < MIN_LINEAR_X);
  if (!init || is_stopping)
  {
    init = true;
    return ccs;
  }

  // lateral acceleration
  double lacc = calcLaccWithSteeringAngle(lv, sa).get();
  // limit lateral acceleration
  if (std::fabs(lacc) > lateral_accel_limit_ && !is_stopping)
  {
    double sgn = lacc / std::fabs(lacc);
    double sa_max =
      std::atan(sgn * lateral_accel_limit_ * wheel_base_ / (lv * lv));
    auto& clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 1,
        "Limit steering angle by lateral acceleration: %f -> %f", sa, sa_max);
    sa = sa_max;
  }

  // lateral jerk
  double ljerk = calcLjerkWithSteeringAngle(lv, sa).get();
  // limit lateral jerk
  if (std::fabs(ljerk) > lateral_jerk_limit_ && !is_stopping)
  {
    double sgn = ljerk / std::fabs(ljerk);
    double sa_max = std::atan(std::tan(sa_prev_.val) +
      sgn * (lateral_jerk_limit_ * wheel_base_ / (lv * lv)) * sa_prev_.dt);
    auto& clk = *this->get_clock();
    RCLCPP_WARN_THROTTLE(get_logger(), clk, 1,
     "Limit steering angle by lateral jerk: %f -> %f",sa, sa_max);
    sa = sa_max;
  }

  // update by lateral limitation
  ccs.cmd.steering_angle = sa;

  // update lateral acceleration/jerk
  lacc = calcLaccWithSteeringAngle(lv, sa).get();
  ljerk = calcLjerkWithSteeringAngle(lv, sa).get();

  // for debug
  std_msgs::msg::Float32 lacc_msg, ljerk_msg;
  lacc_msg.data = lacc;
  ljerk_msg.data = ljerk;
  ctrl_lacc_limit_debug_pub_->publish(lacc_msg);
  ctrl_ljerk_limit_debug_pub_->publish(ljerk_msg);

  return ccs;
}

autoware_msgs::msg::ControlCommandStamped
  TwistFilter::smoothCtrl(const autoware_msgs::msg::ControlCommandStamped& msg)
{
  static auto lp_lx = lowpass_filter();
  static auto lp_sa = lowpass_filter();

  autoware_msgs::msg::ControlCommandStamped ccs;
  ccs = msg;

  // apply lowpass filter to linear_x / steering_angle
  ccs.cmd.linear_velocity =
    lp_lx(ccs.cmd.linear_velocity, lowpass_gain_linear_x_);
  ccs.cmd.steering_angle =
    lp_sa(ccs.cmd.steering_angle, lowpass_gain_steering_angle_);

  return ccs;
}

void TwistFilter::configCallback(
  const autoware_config_msgs::msg::ConfigTwistFilter::UniquePtr config)
{
  lateral_accel_limit_ = config->lateral_accel_limit;
  lateral_jerk_limit_ = config->lateral_jerk_limit;
  lowpass_gain_linear_x_ = config->lowpass_gain_linear_x;
  lowpass_gain_angular_z_ = config->lowpass_gain_angular_z;
  lowpass_gain_steering_angle_ = config->lowpass_gain_steering_angle;
}

void TwistFilter::updatePrevTwist(const geometry_msgs::msg::TwistStamped& msg)
{
  az_prev_.time = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  az_prev_.val = msg.twist.angular.z;
}

void TwistFilter::updatePrevCtrl(
  const autoware_msgs::msg::ControlCommandStamped& msg)
{
  sa_prev_.time = rclcpp::Time(msg.header.stamp.sec, msg.header.stamp.nanosec);
  sa_prev_.val = msg.cmd.steering_angle;
}

void TwistFilter::TwistCmdCallback(
  const geometry_msgs::msg::TwistStamped::UniquePtr msg)
{
  checkTwist(*msg);
  geometry_msgs::msg::TwistStamped ts;
  ts = twist_filter::longitudinalLimitTwist(*msg, longitudinal_velocity_limit_);
  ts = _lon_accel_limiter.longitudinalAccelLimitTwist(ts);
  ts = lateralLimitTwist(ts);
  ts = smoothTwist(ts);
  twist_pub_->publish(ts);
  publishLateralResultsWithTwist(ts);
  updatePrevTwist(ts);
}

void TwistFilter::CtrlCmdCallback(
  const autoware_msgs::msg::ControlCommandStamped::UniquePtr msg)
{
  checkCtrl(*msg);
  autoware_msgs::msg::ControlCommandStamped ccs;
  ccs = twist_filter::longitudinalLimitCtrl(*msg, longitudinal_velocity_limit_);
  ccs = _lon_accel_limiter.longitudinalAccelLimitCtrl(ccs);
  ccs = lateralLimitCtrl(ccs);
  ccs = smoothCtrl(ccs);
  ctrl_pub_->publish(ccs);
  publishLateralResultsWithCtrl(ccs);
  updatePrevCtrl(ccs);
}


} // namespace twist_filter

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(twist_filter::TwistFilter)