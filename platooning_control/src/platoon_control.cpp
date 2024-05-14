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
    config_.stand_still_headway_m = declare_parameter<double>("stand_still_headway_m", config_.stand_still_headway_m);
    config_.max_accel_mps2 = declare_parameter<double>("max_accel_mps2", config_.max_accel_mps2);
    config_.kp = declare_parameter<double>("kp", config_.kp);
    config_.kd = declare_parameter<double>("kd", config_.kd);
    config_.ki = declare_parameter<double>("ki", config_.ki);
    config_.max_delta_speed_per_timestep = declare_parameter<double>("max_delta_speed_per_timestep", config_.max_delta_speed_per_timestep);
    config_.min_delta_speed_per_timestep = declare_parameter<double>("min_delta_speed_per_timestep", config_.min_delta_speed_per_timestep);
    config_.adjustment_cap_mps = declare_parameter<double>("adjustment_cap_mps", config_.adjustment_cap_mps);
    config_.cmd_timestamp_ms = declare_parameter<int>("cmd_timestamp_ms", config_.cmd_timestamp_ms);
    config_.integrator_max = declare_parameter<double>("integrator_max", config_.integrator_max);
    config_.integrator_min = declare_parameter<double>("integrator_min", config_.integrator_min);

    config_.vehicle_response_lag = declare_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
    config_.max_lookahead_dist = declare_parameter<double>("maximum_lookahead_distance", config_.max_lookahead_dist);
    config_.min_lookahead_dist = declare_parameter<double>("minimum_lookahead_distance", config_.min_lookahead_dist);
    config_.speed_to_lookahead_ratio = declare_parameter<double>("speed_to_lookahead_ratio", config_.speed_to_lookahead_ratio);
    config_.is_interpolate_lookahead_point = declare_parameter<bool>("is_interpolate_lookahead_point", config_.is_interpolate_lookahead_point);
    config_.is_delay_compensation = declare_parameter<bool>("is_delay_compensation", config_.is_delay_compensation);
    config_.emergency_stop_distance = declare_parameter<double>("emergency_stop_distance", config_.emergency_stop_distance);
    config_.speed_thres_traveling_direction = declare_parameter<double>("speed_thres_traveling_direction", config_.speed_thres_traveling_direction);
    config_.dist_front_rear_wheels = declare_parameter<double>("dist_front_rear_wheels", config_.dist_front_rear_wheels);

    config_.dt = declare_parameter<double>("dt", config_.dt);
    config_.integrator_max_pp = declare_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
    config_.integrator_min_pp = declare_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
    config_.ki_pp = declare_parameter<double>("Ki_pp", config_.ki_pp);
    config_.is_integrator_enabled = declare_parameter<bool>("is_integrator_enabled", config_.is_integrator_enabled);

    //Global params (from vehicle config)
    config_.vehicle_id  = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
    config_.shutdown_timeout = declare_parameter<int>("control_plugin_shutdown_timeout", config_.shutdown_timeout);
    config_.ignore_initial_inputs = declare_parameter<int>("control_plugin_ignore_initial_inputs", config_.ignore_initial_inputs);

    pcw_ = PlatoonControlWorker();
    pcw_.ctrl_config_ = std::make_shared<PlatooningControlPluginConfig>(config_);

  }

  rcl_interfaces::msg::SetParametersResult PlatoonControlPlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_double = update_params<double>({
      {"stand_still_headway_m", config_.stand_still_headway_m},
      {"max_accel_mps2", config_.max_accel_mps2},
      {"kp", config_.kp},
      {"kd", config_.kd},
      {"ki", config_.ki},
      {"max_delta_speed_per_timestep", config_.max_delta_speed_per_timestep},
      {"min_delta_speed_per_timestep", config_.min_delta_speed_per_timestep},
      {"adjustment_cap_mps", config_.adjustment_cap_mps},
      {"integrator_max", config_.integrator_max},
      {"integrator_min", config_.integrator_min},

      {"vehicle_response_lag", config_.vehicle_response_lag},
      {"max_lookahead_dist", config_.max_lookahead_dist},
      {"min_lookahead_dist", config_.min_lookahead_dist},
      {"speed_to_lookahead_ratio", config_.speed_to_lookahead_ratio},
      {"emergency_stop_distance",config_.emergency_stop_distance},
      {"speed_thres_traveling_direction", config_.speed_thres_traveling_direction},
      {"dist_front_rear_wheels", config_.dist_front_rear_wheels},
      {"dt", config_.dt},
      {"integrator_max_pp", config_.integrator_max_pp},
      {"integrator_min_pp", config_.integrator_min_pp},
      {"Ki_pp", config_.ki_pp},
    }, parameters);

    auto error_int = update_params<int>({
      {"cmd_timestamp_ms", config_.cmd_timestamp_ms},
    }, parameters);

    auto error_bool = update_params<bool>({
      {"is_interpolate_lookahead_point", config_.is_interpolate_lookahead_point},
      {"is_delay_compensation",config_.is_delay_compensation},
      {"is_integrator_enabled", config_.is_integrator_enabled},
  }, parameters);

    // vehicle_id, control_plugin_shutdown_timeout and control_plugin_ignore_initial_inputs are not updated as they are global params
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_double && !error_int && !error_bool;

    return result;

  }

  carma_ros2_utils::CallbackReturn PlatoonControlPlugin::on_configure_plugin()
  {
    // Reset config
    config_ = PlatooningControlPluginConfig();

    // Load parameters
    get_parameter<double>("stand_still_headway_m", config_.stand_still_headway_m);
    get_parameter<double>("max_accel_mps2", config_.max_accel_mps2);
    get_parameter<double>("kp", config_.kp);
    get_parameter<double>("kd", config_.kd);
    get_parameter<double>("ki", config_.ki);
    get_parameter<double>("max_delta_speed_per_timestep", config_.max_delta_speed_per_timestep);
    get_parameter<double>("min_delta_speed_per_timestep", config_.min_delta_speed_per_timestep);
    get_parameter<double>("adjustment_cap_mps", config_.adjustment_cap_mps);
    get_parameter<int>("cmd_timestamp_ms", config_.cmd_timestamp_ms);
    get_parameter<double>("integrator_max", config_.integrator_max);
    get_parameter<double>("integrator_min", config_.integrator_min);

    get_parameter<std::string>("vehicle_id", config_.vehicle_id);
    get_parameter<int>("control_plugin_shutdown_timeout", config_.shutdown_timeout);
    get_parameter<int>("control_plugin_ignore_initial_inputs", config_.ignore_initial_inputs);

   //Pure Pursuit params
    get_parameter<double>("vehicle_response_lag", config_.vehicle_response_lag);
    get_parameter<double>("maximum_lookahead_distance", config_.max_lookahead_dist);
    get_parameter<double>("minimum_lookahead_distance", config_.min_lookahead_dist);
    get_parameter<double>("speed_to_lookahead_ratio", config_.speed_to_lookahead_ratio);
    get_parameter<bool>("is_interpolate_lookahead_point", config_.is_interpolate_lookahead_point);
    get_parameter<bool>("is_delay_compensation", config_.is_delay_compensation);
    get_parameter<double>("emergency_stop_distance", config_.emergency_stop_distance);
    get_parameter<double>("speed_thres_traveling_direction", config_.speed_thres_traveling_direction);
    get_parameter<double>("dist_front_rear_wheels", config_.dist_front_rear_wheels);

    get_parameter<double>("dt", config_.dt);
    get_parameter<double>("integrator_max_pp", config_.integrator_max_pp);
    get_parameter<double>("integrator_min_pp", config_.integrator_min_pp);
    get_parameter<double>("Ki_pp", config_.ki_pp);
    get_parameter<bool>("is_integrator_enabled", config_.is_integrator_enabled);


    RCLCPP_INFO_STREAM(rclcpp::get_logger("platoon_control"), "Loaded Params: " << config_);

    // create config for pure_pursuit worker
    pure_pursuit::Config cfg{
      config_.min_lookahead_dist,
      config_.max_lookahead_dist,
      config_.speed_to_lookahead_ratio,
      config_.is_interpolate_lookahead_point,
      config_.is_delay_compensation,
      config_.emergency_stop_distance,
      config_.speed_thres_traveling_direction,
      config_.dist_front_rear_wheels,
    };

    pure_pursuit::IntegratorConfig i_cfg;
    i_cfg.dt = config_.dt;
    i_cfg.integrator_max_pp = config_.integrator_max_pp;
    i_cfg.integrator_min_pp = config_.integrator_min_pp;
    i_cfg.Ki_pp = config_.ki_pp;
    i_cfg.integral = 0.0; // accumulator of integral starts from 0
    i_cfg.is_integrator_enabled = config_.is_integrator_enabled;

    pp_ = std::make_shared<pure_pursuit::PurePursuit>(cfg, i_cfg);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&PlatoonControlPlugin::parameter_update_callback, this, std_ph::_1));


    // Trajectory Plan Subscriber
    trajectory_plan_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("platoon_control/plan_trajectory", 1,
                                                                                            std::bind(&PlatoonControlPlugin::current_trajectory_callback, this, std_ph::_1));

    // Platoon Info Subscriber
    platoon_info_sub_ = create_subscription<carma_planning_msgs::msg::PlatooningInfo>("platoon_info", 1, std::bind(&PlatoonControlPlugin::platoon_info_cb, this, std_ph::_1));


    //Control Publishers
    platoon_info_pub_ = create_publisher<carma_planning_msgs::msg::PlatooningInfo>("platooning_info", 1);


    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }


  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::generate_command()
  {

    autoware_msgs::msg::ControlCommandStamped ctrl_msg;
    if (!current_trajectory_ || !current_pose_ || !current_twist_)
      return ctrl_msg;

    // If it has been a long time since input data has arrived then reset the input counter and return
    // Note: this quiets the controller after its input stream stops, which is necessary to allow
    // the replacement controller to publish on the same output topic after this one is done.
    long current_time_ms = this->now().nanoseconds() / 1e6;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "current_time_ms = " << current_time_ms << ", prev_input_time_ = " << prev_input_time_ << ", input counter = " << consecutive_input_counter_);

    if(current_time_ms - prev_input_time_ > config_.shutdown_timeout)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "returning due to timeout.");
        consecutive_input_counter_ = 0;
        return ctrl_msg;
    }

    // If there have not been enough consecutive timely inputs then return (waiting for
    // previous control plugin to time out and stop publishing, since it uses same output topic)
    if (consecutive_input_counter_ <= config_.ignore_initial_inputs)
    {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "returning due to first data input");
        return ctrl_msg;
    }

    carma_planning_msgs::msg::TrajectoryPlanPoint second_trajectory_point = latest_trajectory_.trajectory_points[1];
    carma_planning_msgs::msg::TrajectoryPlanPoint lookahead_point = get_lookahead_trajectory_point(latest_trajectory_, current_pose_.get(), current_twist_.get());

    trajectory_speed_ = get_trajectory_speed(latest_trajectory_.trajectory_points);

    ctrl_msg = generate_control_signals(second_trajectory_point, lookahead_point, current_pose_.get(), current_twist_.get());

    return ctrl_msg;

  }

  carma_planning_msgs::msg::TrajectoryPlanPoint PlatoonControlPlugin::get_lookahead_trajectory_point(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan, const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::TwistStamped& current_twist)
  {
    carma_planning_msgs::msg::TrajectoryPlanPoint lookahead_point;

    double lookahead_dist = config_.speed_to_lookahead_ratio * current_twist.twist.linear.x;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "lookahead based on speed: " << lookahead_dist);

    lookahead_dist = std::max(config_.min_lookahead_dist, lookahead_dist);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "final lookahead: " << lookahead_dist);

    double traveled_dist = 0.0;
    bool found_point = false;

    for (size_t i = 1; i<trajectory_plan.trajectory_points.size() - 1; i++)
    {
        double dx =  current_pose.pose.position.x - trajectory_plan.trajectory_points[i].x;
        double dy =  current_pose.pose.position.y - trajectory_plan.trajectory_points[i].y;

        double dx1 = trajectory_plan.trajectory_points[i].x - trajectory_plan.trajectory_points[i-1].x;
        double dy1 = trajectory_plan.trajectory_points[i].y - trajectory_plan.trajectory_points[i-1].y;
        double dist1 = std::sqrt(dx1*dx1 + dy1*dy1);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "trajectory spacing: " << dist1);

        double dist = std::sqrt(dx*dx + dy*dy);

        traveled_dist = dist;

        if ((lookahead_dist - traveled_dist) < 1.0)
        {
            lookahead_point =  trajectory_plan.trajectory_points[i];
            found_point = true;
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "found lookahead point at index: " << i);
            break;
        }
    }

    if (!found_point)
    {
        lookahead_point = trajectory_plan.trajectory_points.back();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "lookahead point set as the last trajectory point");
    }


    return lookahead_point;
  }

  void PlatoonControlPlugin::platoon_info_cb(const carma_planning_msgs::msg::PlatooningInfo::SharedPtr msg)
  {

    platoon_leader_.staticId = msg->leader_id;
    platoon_leader_.vehiclePosition = msg->leader_downtrack_distance;
    platoon_leader_.commandSpeed = msg->leader_cmd_speed;
    // TODO: index is 0 temp to test the leader state
    platoon_leader_.NumberOfVehicleInFront = msg->host_platoon_position;
    platoon_leader_.leaderIndex = 0;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Platoon leader leader id:  " << platoon_leader_.staticId);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Platoon leader leader pose:  " << platoon_leader_.vehiclePosition);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Platoon leader leader cmd speed:  " << platoon_leader_.commandSpeed);

    carma_planning_msgs::msg::PlatooningInfo platooning_info_msg = *msg;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "platooning_info_msg.actual_gap:  " << platooning_info_msg.actual_gap);

    if (platooning_info_msg.actual_gap > 5.0)
    {
        platooning_info_msg.actual_gap -= 5.0; // TODO: temporary: should be vehicle length
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "platooning_info_msg.actual_gap:  " << platooning_info_msg.actual_gap);
    // platooing_info_msg.desired_gap = pcw_.desired_gap_;
    // platooing_info_msg.actual_gap = pcw_.actual_gap_;
    pcw_.actual_gap_ = platooning_info_msg.actual_gap;
    pcw_.desired_gap_ = platooning_info_msg.desired_gap;

    platooning_info_msg.host_cmd_speed = pcw_.speedCmd_;
    platoon_info_pub_->publish(platooning_info_msg);
  }

  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::generate_control_signals(const carma_planning_msgs::msg::TrajectoryPlanPoint& first_trajectory_point, const carma_planning_msgs::msg::TrajectoryPlanPoint& lookahead_point, const geometry_msgs::msg::PoseStamped& current_pose, const geometry_msgs::msg::TwistStamped& current_twist)
  {
    pcw_.set_current_speed(trajectory_speed_); //TODO why this and not the actual vehicle speed?  Method name suggests different use than this.
    // pcw_.set_current_speed(current_twist_.get());
    pcw_.set_leader(platoon_leader_);
    pcw_.generate_speed(first_trajectory_point);

    motion::control::controller_common::State state_tf = convert_state(current_pose_.get(), current_twist_.get());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "Forced from frame_id: " << state_tf.header.frame_id << ", into: " << current_trajectory_.get().header.frame_id);

    current_trajectory_.get().header.frame_id = state_tf.header.frame_id;

    auto autoware_traj_plan = basic_autonomy::waypoint_generation::process_trajectory_plan(current_trajectory_.get(), config_.vehicle_response_lag);

    pp_->set_trajectory(autoware_traj_plan);
    const auto cmd{pp_->compute_command(state_tf)};

    auto steer_cmd = cmd.front_wheel_angle_rad; //autoware sets the front wheel angle as the calculated steer. https://github.com/usdot-fhwa-stol/autoware.auto/blob/3450f94fa694f51b00de272d412722d65a2c2d3e/AutowareAuto/src/control/pure_pursuit/src/pure_pursuit.cpp#L88

    autoware_msgs::msg::ControlCommandStamped ctrl_msg = compose_ctrl_cmd(pcw_.speedCmd_, steer_cmd);

    return ctrl_msg;
  }

  motion::motion_common::State PlatoonControlPlugin::convert_state(const geometry_msgs::msg::PoseStamped& pose, const geometry_msgs::msg::TwistStamped& twist)
  {
    motion::motion_common::State state;
    state.header = pose.header;
    state.state.x = pose.pose.position.x;
    state.state.y = pose.pose.position.y;
    state.state.z = pose.pose.position.z;
    state.state.heading.real = pose.pose.orientation.w;
    state.state.heading.imag = pose.pose.orientation.z;

    state.state.longitudinal_velocity_mps = twist.twist.linear.x;
    return state;
  }

  void PlatoonControlPlugin::current_trajectory_callback(const carma_planning_msgs::msg::TrajectoryPlan::UniquePtr tp)
  {
    if (tp->trajectory_points.size() < 2) {
            RCLCPP_WARN_STREAM(rclcpp::get_logger("platoon_control"), "PlatoonControlPlugin cannot execute trajectory as only 1 point was provided");
            return;
        }

        latest_trajectory_ = *tp;
        prev_input_time_ = this->now().nanoseconds() / 1000000;
        ++consecutive_input_counter_;
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "New trajectory plan #" << consecutive_input_counter_ << " at time " << prev_input_time_);
        rclcpp::Time tp_time(tp->header.stamp);
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "tp header time =                " << tp_time.nanoseconds() / 1000000);
  }

  geometry_msgs::msg::TwistStamped PlatoonControlPlugin::compose_twist_cmd(double linear_vel, double angular_vel)
  {
    geometry_msgs::msg::TwistStamped cmd_twist;
    cmd_twist.twist.linear.x = linear_vel;
    cmd_twist.twist.angular.z = angular_vel;
    cmd_twist.header.stamp = this->now();
    return cmd_twist;
  }

  autoware_msgs::msg::ControlCommandStamped PlatoonControlPlugin::compose_ctrl_cmd(double linear_vel, double steering_angle)
  {
    autoware_msgs::msg::ControlCommandStamped cmd_ctrl;
    cmd_ctrl.header.stamp = this->now();
    cmd_ctrl.cmd.linear_velocity = linear_vel;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "ctrl command speed " << cmd_ctrl.cmd.linear_velocity);
    cmd_ctrl.cmd.steering_angle = steering_angle;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "ctrl command steering " << cmd_ctrl.cmd.steering_angle);

    return cmd_ctrl;
  }

  bool PlatoonControlPlugin::get_availability() {
    return true; // TODO for user implement actual check on availability if applicable to plugin
  }

  std::string PlatoonControlPlugin::get_version_id() {
    return "v1.0";
  }

  // extract maximum speed of trajectory
  double PlatoonControlPlugin::get_trajectory_speed(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points)
  {
    double trajectory_speed = 0;

    double dx1 = trajectory_points[trajectory_points.size()-1].x - trajectory_points[0].x;
    double dy1 = trajectory_points[trajectory_points.size()-1].y - trajectory_points[0].y;
    double d1 = sqrt(dx1*dx1 + dy1*dy1);
    double t1 = rclcpp::Time((trajectory_points[trajectory_points.size()-1].target_time)).seconds() - rclcpp::Time(trajectory_points[0].target_time).nanoseconds();

    double avg_speed = d1/t1;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "trajectory_points size = " << trajectory_points.size() << ", d1 = " << d1 << ", t1 = " << t1 << ", avg_speed = " << avg_speed);

    for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
    {
        double dx = trajectory_points[i + 1].x - trajectory_points[i].x;
        double dy = trajectory_points[i + 1].y - trajectory_points[i].y;
        double d = sqrt(dx*dx + dy*dy);
        double t = rclcpp::Time((trajectory_points[i + 1].target_time)).seconds() - rclcpp::Time(trajectory_points[i].target_time).seconds();
        double v = d/t;
        if(v > trajectory_speed)
        {
            trajectory_speed = v;
        }
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "trajectory speed: " << trajectory_speed);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("platoon_control"), "avg trajectory speed: " << avg_speed);

    return avg_speed; //TODO: why are 2 speeds being calculated? Which should be returned?

  }


} // platoon_control

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(platoon_control::PlatoonControlPlugin)
