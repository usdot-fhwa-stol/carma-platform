/*
 * Copyright (C) 2024 LEIDOS.
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
#include "platoon_control/platoon_control.hpp"

namespace platoon_control
{
  namespace std_ph = std::placeholders;

  PlatoonControlPlugin::PlatoonControlPlugin(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::ControlPlugin(options)
  {
    // Create initial config
    config_ = PlatooningControlPluginConfig();

    // Declare parameters
    config_.stand_still_headway = declare_parameter<double>("stand_still_headway", config_.stand_still_headway);
    config_.max_accel = declare_parameter<double>("max_accel", config_.max_accel);
    config_.kp = declare_parameter<double>("kp", config_.kp);
    config_.kd = declare_parameter<double>("kd", config_.kd);
    config_.ki = declare_parameter<double>("ki", config_.ki);
    config_.max_value = declare_parameter<double>("max_value", config_.max_value);
    config_.min_value = declare_parameter<double>("min_value", config_.min_value);
    config_.dt = declare_parameter<double>("dt", config_.dt);
    config_.adjustment_cap = declare_parameter<double>("adjustment_cap", config_.adjustment_cap);
    config_.cmd_timestamp = declare_parameter<int>("cmd_timestamp", config_.cmd_timestamp);
    config_.integrator_max = declare_parameter<double>("integrator_max", config_.integrator_max);
    config_.integrator_min = declare_parameter<double>("integrator_min", config_.integrator_min);
    config_.lowpass_gain = declare_parameter<double>("lowpass_gain", config_.lowpass_gain);
    config_.lookahead_ratio = declare_parameter<double>("lookahead_ratio", config_.lookahead_ratio);
    config_.min_lookahead_dist = declare_parameter<double>("min_lookahead_dist", config_.min_lookahead_dist);
    config_.correction_angle = declare_parameter<double>("correction_angle", config_.correction_angle);
    config_.integrator_max_pp = declare_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
    config_.integrator_min_pp = declare_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
    config_.ki_pp = declare_parameter<double>("ki_pp", config_.ki_pp);

    //Global params (from vehicle config)
    config_.kdd = declare_parameter<double>("kdd", config_.kdd);
    config_.wheel_base = declare_parameter<double>("wheel_base", config_.wheel_base);

    config_.vehicle_id  = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
    config_.shutdown_timeout = declare_parameter<int>("control_plugin_shutdown_timeout", config_.shutdown_timeout);
    config_.ignore_initial_inputs = declare_parameter<int>("control_plugin_ignore_initial_inputs", config_.ignore_initial_inputs);

    pcw_ = PlatoonControlWorker();
    pcw_.ctrl_config_ = std::make_shared<PlatooningControlPluginConfig>(config_);
    pcw_.current_pose_ = std::make_shared<geometry_msgs::msg::Pose>(current_pose_.get().pose);

  }

  rcl_interfaces::msg::SetParametersResult PlatoonControlPlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_double = update_params<double>({
      {"stand_still_headway", config_.stand_still_headway},
      {"max_accel", config_.max_accel},
      {"kp", config_.kp},
      {"kd", config_.kd},
      {"ki", config_.ki},
      {"max_value", config_.max_value},
      {"min_value", config_.min_value},
      {"dt", config_.dt},
      {"adjustment_cap", config_.adjustment_cap},
      {"integrator_max", config_.integrator_max},
      {"integrator_min", config_.integrator_min},
      {"kdd", config_.kdd},
      {"wheel_base", config_.wheel_base},
      {"lowpass_gain", config_.lowpass_gain},
      {"lookahead_ratio", config_.lookahead_ratio},
      {"min_lookahead_dist", config_.min_lookahead_dist},
      {"correction_angle", config_.correction_angle},
      {"integrator_max_pp", config_.integrator_max_pp},
      {"integrator_min_pp", config_.integrator_min_pp},
      {"ki_pp", config_.ki_pp},
    }, parameters);

    auto error_int = update_params<int>({
      {"cmd_timestamp", config_.cmd_timestamp},
    }, parameters);

    // vehicle_id, control_plugin_shutdown_timeout and control_plugin_ignore_initial_inputs are not updated as they are global params
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_double && !error_int;

    return result;

  }

  carma_ros2_utils::CallbackReturn PlatoonControlPlugin::on_configure_plugin()
  {
    // Reset config
    config_ = PlatooningControlPluginConfig();

    // Load parameters
    get_parameter<double>("stand_still_headway", config_.stand_still_headway);
    get_parameter<double>("max_accel", config_.max_accel);
    get_parameter<double>("kp", config_.kp);
    get_parameter<double>("kd", config_.kd);
    get_parameter<double>("ki", config_.ki);
    get_parameter<double>("max_value", config_.max_value);
    get_parameter<double>("min_value", config_.min_value);
    get_parameter<double>("dt", config_.dt);
    get_parameter<double>("adjustment_cap", config_.adjustment_cap);
    get_parameter<int>("cmd_timestamp", config_.cmd_timestamp);
    get_parameter<double>("integrator_max", config_.integrator_max);
    get_parameter<double>("integrator_min", config_.integrator_min);
    get_parameter<double>("kdd", config_.kdd);
    get_parameter<double>("wheel_base", config_.wheel_base);
    get_parameter<double>("lowpass_gain", config_.lowpass_gain);
    get_parameter<double>("lookahead_ratio", config_.lookahead_ratio);
    get_parameter<double>("min_lookahead_dist", config_.min_lookahead_dist);
    get_parameter<double>("correction_angle", config_.correction_angle);
    get_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
    get_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
    get_parameter<double>("ki_pp", config_.ki_pp);


    get_parameter<std::string>("vehicle_id", config_.vehicle_id);
    get_parameter<int>("control_plugin_shutdown_timeout", config_.shutdown_timeout);
    get_parameter<int>("control_plugin_ignore_initial_inputs", config_.ignore_initial_inputs);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded Params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&PlatoonControlPlugin::parameter_update_callback, this, std_ph::_1));


    // Trajectory Plan Subscriber
    trajectory_plan_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("platoon_control/plan_trajectory", 1,
                                                                                            std::bind(&PlatoonControlPlugin::current_trajectory_callback, this, std_ph::_1));

    // Platoon Info Subscriber
    platoon_info_sub_ = create_subscription<carma_planning_msgs::msg::PlatooningInfo>("platoon_info", 1, std::bind(&PlatoonControlPlugin::platoonInfo_cb, this, std_ph::_1));


    //Control Publishers
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("twist_raw", 5); //TODO car5188: Should this be transient_local?
    platoon_info_pub_ = create_publisher<carma_planning_msgs::msg::PlatooningInfo>("platooning_info", 1); //TODO car5188: Should this be transient_local?


    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }


  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::generate_command()
  {

    autoware_msgs::msg::ControlCommandStamped ctrl_msg;

    // If it has been a long time since input data has arrived then reset the input counter and return
    // Note: this quiets the controller after its input stream stops, which is necessary to allow
    // the replacement controller to publish on the same output topic after this one is done.
    long current_time = this->now().nanoseconds() / 1e6;
    RCLCPP_DEBUG_STREAM(get_logger(), "current_time = " << current_time << ", prev_input_time_ = " << prev_input_time_ << ", input counter = " << consecutive_input_counter_);

    if(current_time - prev_input_time_ > config_.shutdown_timeout)
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "returning due to timeout.");
        consecutive_input_counter_ = 0;
        return ctrl_msg;
    }

    // If there have not been enough consecutive timely inputs then return (waiting for
    // previous control plugin to time out and stop publishing, since it uses same output topic)
    if (consecutive_input_counter_ <= config_.ignore_initial_inputs)
    {
        RCLCPP_DEBUG_STREAM(get_logger(), "returning due to first data input");
        return ctrl_msg;
    }

    carma_planning_msgs::msg::TrajectoryPlanPoint second_trajectory_point = latest_trajectory_.trajectory_points[1];
    carma_planning_msgs::msg::TrajectoryPlanPoint lookahead_point = getLookaheadTrajectoryPoint(latest_trajectory_);

    trajectory_speed_ = getTrajectorySpeed(latest_trajectory_.trajectory_points);

    ctrl_msg = generateControlSignals(second_trajectory_point, lookahead_point);

    return ctrl_msg;

  }

  carma_planning_msgs::msg::TrajectoryPlanPoint PlatoonControlPlugin::getLookaheadTrajectoryPoint(carma_planning_msgs::msg::TrajectoryPlan trajectory_plan)
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint lookahead_point;

    double lookahead_dist = config_.lookahead_ratio * current_twist_.get().twist.linear.x;
    RCLCPP_DEBUG_STREAM(get_logger(), "lookahead based on speed: " << lookahead_dist);

    lookahead_dist = std::max(config_.min_lookahead_dist, lookahead_dist);
    RCLCPP_DEBUG_STREAM(get_logger(), "final lookahead: " << lookahead_dist);

    double traveled_dist = 0.0;
    bool found_point = false;

    for (size_t i = 1; i<trajectory_plan.trajectory_points.size() - 1; i++)
    {
        double dx =  current_pose_.get().pose.position.x - trajectory_plan.trajectory_points[i].x;
        double dy =  current_pose_.get().pose.position.y - trajectory_plan.trajectory_points[i].y;

        double dx1 = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
        double dy1 = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
        double dist1 = std::sqrt(dx1*dx1 + dy1*dy1);
        RCLCPP_DEBUG_STREAM(get_logger(), "trajectory spacing: " << dist1);

        double dist = std::sqrt(dx*dx + dy*dy);

        traveled_dist = dist;

        if ((lookahead_dist - traveled_dist) < 1.0)
        {
            lookahead_point =  trajectory_plan.trajectory_points[i];
            found_point = true;
            RCLCPP_DEBUG_STREAM(get_logger(), "found lookahead point at index: " << i);
            break;
        }
    }

    if (!found_point)
    {
        lookahead_point = trajectory_plan.trajectory_points.back();
        RCLCPP_DEBUG_STREAM(get_logger(), "lookahead point set as the last trajectory point");
    }


    return lookahead_point;
  }

  void PlatoonControlPlugin::platoonInfo_cb(const carma_planning_msgs::msg::PlatooningInfo::SharedPtr msg)
  {

    platoon_leader_.staticId = msg->leader_id;
    platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
    platoon_leader_.commandSpeed = msg->leader_cmd_speed;
    // TODO: index is 0 temp to test the leader state
    platoon_leader_.NumberOfVehicleInFront = msg->host_platoon_position;
    platoon_leader_.leaderIndex = 0;

    RCLCPP_DEBUG_STREAM(get_logger(), "Platoon leader leader id:  " << platoon_leader_.staticId);
    RCLCPP_DEBUG_STREAM(get_logger(), "Platoon leader leader pose:  " << platoon_leader_.vehiclePosition);
    RCLCPP_DEBUG_STREAM(get_logger(), "Platoon leader leader cmd speed:  " << platoon_leader_.commandSpeed);

    carma_planning_msgs::msg::PlatooningInfo platooing_info_msg = *msg;

    RCLCPP_DEBUG_STREAM(get_logger(), "platooing_info_msg.actual_gap:  " << platooing_info_msg.actual_gap);

    if (platooing_info_msg.actual_gap > 5.0)
    {
        platooing_info_msg.actual_gap -= 5.0; // TODO: temporary: should be vehicle length
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "platooing_info_msg.actual_gap:  " << platooing_info_msg.actual_gap);
    // platooing_info_msg.desired_gap = pcw_.desired_gap_;
    // platooing_info_msg.actual_gap = pcw_.actual_gap_;
    pcw_.actual_gap_ = platooing_info_msg.actual_gap;
    pcw_.desired_gap_ = platooing_info_msg.desired_gap;

    platooing_info_msg.host_cmd_speed = pcw_.speedCmd_;
    platoon_info_pub_->publish(platooing_info_msg);
  }

  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::generateControlSignals(const carma_planning_msgs::msg::TrajectoryPlanPoint& first_trajectory_point, const carma_planning_msgs::msg::TrajectoryPlanPoint& lookahead_point)
  {
    pcw_.setCurrentSpeed(trajectory_speed_); //TODO why this and not the actual vehicle speed?  Method name suggests different use than this.
    // pcw_.setCurrentSpeed(current_twist_.get());
    pcw_.setLeader(platoon_leader_);
    pcw_.generateSpeed(first_trajectory_point);
    pcw_.generateSteer(lookahead_point);

    geometry_msgs::msg::TwistStamped twist_msg = composeTwistCmd(pcw_.speedCmd_, pcw_.angVelCmd_);
    twist_pub_->publish(twist_msg);

    autoware_msgs::msg::ControlCommandStamped ctrl_msg = composeCtrlCmd(pcw_.speedCmd_, pcw_.steerCmd_);

    return ctrl_msg;
  }

  void PlatoonControlPlugin::current_trajectory_callback(const carma_planning_msgs::msg::TrajectoryPlan::UniquePtr tp)
  {
    if (tp->trajectory_points.size() < 2) {
            RCLCPP_WARN_STREAM(get_logger(), "PlatoonControlPlugin cannot execute trajectory as only 1 point was provided");
            return;
        }

        latest_trajectory_ = *tp;
        prev_input_time_ = this->now().nanoseconds() / 1000000;
        ++consecutive_input_counter_;
        RCLCPP_DEBUG_STREAM(get_logger(), "New trajectory plan #" << consecutive_input_counter_ << " at time " << prev_input_time_);
        rclcpp::Time tp_time(tp->header.stamp);
        RCLCPP_DEBUG_STREAM(get_logger(), "tp header time =                " << tp_time.nanoseconds() / 1000000);
  }

  geometry_msgs::msg::TwistStamped PlatoonControlPlugin::composeTwistCmd(double linear_vel, double angular_vel)
  {
    geometry_msgs::msg::TwistStamped cmd_twist;
    cmd_twist.twist.linear.x = linear_vel;
    cmd_twist.twist.angular.z = angular_vel;
    cmd_twist.header.stamp = this->now();
    return cmd_twist;
  }

  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::composeCtrlCmd(double linear_vel, double steering_angle)
  {
    autoware_msgs::msg::ControlCommandStamped cmd_ctrl;
    cmd_ctrl.header.stamp = this->now();
    cmd_ctrl.cmd.linear_velocity = linear_vel;
    RCLCPP_DEBUG_STREAM(get_logger(), "ctrl command speed " << cmd_ctrl.cmd.linear_velocity);
    cmd_ctrl.cmd.steering_angle = steering_angle;
    RCLCPP_DEBUG_STREAM(get_logger(), "ctrl command steering " << cmd_ctrl.cmd.steering_angle);

    return cmd_ctrl;
  }

  bool PlatoonControlPlugin::get_availability() {
    return true; // TODO for user implement actual check on availability if applicable to plugin
  }

  std::string PlatoonControlPlugin::get_version_id() {
    return "v1.0";
  }

  // extract maximum speed of trajectory
  double PlatoonControlPlugin::getTrajectorySpeed(std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points)
  {
    double trajectory_speed = 0;

    double dx1 = trajectory_points[trajectory_points.size()-1].x - trajectory_points[0].x;
    double dy1 = trajectory_points[trajectory_points.size()-1].y - trajectory_points[0].y;
    double d1 = sqrt(dx1*dx1 + dy1*dy1);
    double t1 = (rclcpp::Time((trajectory_points[trajectory_points.size()-1].target_time)).nanoseconds() - rclcpp::Time(trajectory_points[0].target_time).nanoseconds())/1e9;

    double avg_speed = d1/t1;
    RCLCPP_DEBUG_STREAM(get_logger(), "trajectory_points size = " << trajectory_points.size() << ", d1 = " << d1 << ", t1 = " << t1 << ", avg_speed = " << avg_speed);

    for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
    {
        double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
        double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
        double d = sqrt(dx*dx + dy*dy);
        double t = (rclcpp::Time((trajectory_points[i + 1].target_time)).nanoseconds() - rclcpp::Time(trajectory_points[i].target_time).nanoseconds())/1e9;
        double v = d/t;
        if(v > trajectory_speed)
        {
            trajectory_speed = v;
        }
    }

    RCLCPP_DEBUG_STREAM(get_logger(), "trajectory speed: " << trajectory_speed);
    RCLCPP_DEBUG_STREAM(get_logger(), "avg trajectory speed: " << avg_speed);

    return avg_speed; //TODO: why are 2 speeds being calculated? Which should be returned?

  }


} // platoon_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(platoon_control::PlatoonControlPlugin)
