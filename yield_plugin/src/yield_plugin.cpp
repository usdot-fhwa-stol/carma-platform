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

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <algorithm>
#include <memory>
#include <limits>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <lanelet2_core/geometry/Point.h>
#include <trajectory_utils/trajectory_utils.hpp>
#include <trajectory_utils/conversions/conversions.hpp>
#include <sstream>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>
#include <yield_plugin/yield_plugin.hpp>
#include <carma_v2x_msgs/msg/location_ecef.hpp>
#include <carma_v2x_msgs/msg/trajectory.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <basic_autonomy/smoothing/filters.hpp>

using oss = std::ostringstream;
constexpr auto EPSILON {0.01}; //small value to compare doubles

namespace yield_plugin
{
  YieldPlugin::YieldPlugin(std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> nh, carma_wm::WorldModelConstPtr wm, YieldPluginConfig config,
                                            MobilityResponseCB mobility_response_publisher,
                                            LaneChangeStatusCB lc_status_publisher)
    : nh_(nh), wm_(wm), config_(config),mobility_response_publisher_(mobility_response_publisher), lc_status_publisher_(lc_status_publisher)
  {

  }

  double get_trajectory_end_time(const carma_planning_msgs::msg::TrajectoryPlan& trajectory)
  {
    return rclcpp::Time(trajectory.trajectory_points.back().target_time).seconds();
  }

  double get_trajectory_start_time(const carma_planning_msgs::msg::TrajectoryPlan& trajectory)
  {
    return rclcpp::Time(trajectory.trajectory_points.front().target_time).seconds();
  }

  double get_trajectory_duration(const carma_planning_msgs::msg::TrajectoryPlan& trajectory)
  {
    return fabs(get_trajectory_end_time(trajectory) - get_trajectory_start_time(trajectory));
  }

  std::vector<std::pair<int, lanelet::BasicPoint2d>> YieldPlugin::detect_trajectories_intersection(std::vector<lanelet::BasicPoint2d> self_trajectory, std::vector<lanelet::BasicPoint2d> incoming_trajectory) const
  {
    std::vector<std::pair<int, lanelet::BasicPoint2d>> intersection_points;
    boost::geometry::model::linestring<lanelet::BasicPoint2d> self_traj;
    for (auto tpp:self_trajectory)
    {
      boost::geometry::append(self_traj, tpp);
    }
    // distance to consider trajectories colliding (chosen based on lane width and vehicle size)
    for (size_t i=0; i<incoming_trajectory.size(); i++)
    {
      double res = boost::geometry::distance(incoming_trajectory.at(i), self_traj);

      if (fabs(res) <= config_.intervehicle_collision_distance_in_m)
      {
         intersection_points.push_back(std::make_pair(i, incoming_trajectory.at(i)));
      }
    }
    return intersection_points;
  }

  std::vector<lanelet::BasicPoint2d> YieldPlugin::convert_eceftrajectory_to_mappoints(const carma_v2x_msgs::msg::Trajectory& ecef_trajectory) const
  {
    carma_planning_msgs::msg::TrajectoryPlan trajectory_plan;
    std::vector<lanelet::BasicPoint2d> map_points;

    lanelet::BasicPoint2d first_point = ecef_to_map_point(ecef_trajectory.location);

    map_points.push_back(first_point);
    auto curr_point = ecef_trajectory.location;

    for (size_t i = 0; i<ecef_trajectory.offsets.size(); i++)
    {
      lanelet::BasicPoint2d offset_point;
      curr_point.ecef_x += ecef_trajectory.offsets.at(i).offset_x;
      curr_point.ecef_y += ecef_trajectory.offsets.at(i).offset_y;
      curr_point.ecef_z += ecef_trajectory.offsets.at(i).offset_z;

      offset_point = ecef_to_map_point(curr_point);

      map_points.push_back(offset_point);
    }

    return map_points;
  }

  lanelet::BasicPoint2d YieldPlugin::ecef_to_map_point(const carma_v2x_msgs::msg::LocationECEF& ecef_point) const
  {

    if (!map_projector_) {
        throw std::invalid_argument("No map projector available for ecef conversion");
    }

    lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { static_cast<double>(ecef_point.ecef_x)/100.0, static_cast<double>(ecef_point.ecef_y)/100.0, static_cast<double>(ecef_point.ecef_z)/100.0 } , 1);

    return lanelet::traits::to2D(map_point);
  }



  carma_v2x_msgs::msg::MobilityResponse YieldPlugin::compose_mobility_response(const std::string& resp_recipient_id, const std::string& req_plan_id, bool response) const
  {
    carma_v2x_msgs::msg::MobilityResponse out_mobility_response;
    out_mobility_response.m_header.sender_id = config_.vehicle_id;
    out_mobility_response.m_header.recipient_id = resp_recipient_id;
    out_mobility_response.m_header.sender_bsm_id = host_bsm_id_;
    out_mobility_response.m_header.plan_id = req_plan_id;
    out_mobility_response.m_header.timestamp = nh_->now().seconds()*1000;


    if (config_.always_accept_mobility_request && response)
    {
      out_mobility_response.is_accepted = true;
    }
    else out_mobility_response.is_accepted = false;

    return out_mobility_response;
  }


  void YieldPlugin::mobilityrequest_cb(const carma_v2x_msgs::msg::MobilityRequest::UniquePtr msg)
  {
    carma_v2x_msgs::msg::MobilityRequest incoming_request = *msg;
    carma_planning_msgs::msg::LaneChangeStatus lc_status_msg;
    if (incoming_request.strategy == "carma/cooperative-lane-change")
    {
      if (!map_projector_) {
        RCLCPP_ERROR(nh_->get_logger(),"Cannot process mobility request as map projection is not yet set!");
        return;
      }
      if (incoming_request.plan_type.type == carma_v2x_msgs::msg::PlanType::CHANGE_LANE_LEFT || incoming_request.plan_type.type == carma_v2x_msgs::msg::PlanType::CHANGE_LANE_RIGHT)
      {
        RCLCPP_DEBUG(nh_->get_logger(),"Cooperative Lane Change Request Received");
        lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::REQUEST_RECEIVED;
        lc_status_msg.description = "Received lane merge request";

        if (incoming_request.m_header.recipient_id == config_.vehicle_id)
        {
          RCLCPP_DEBUG(nh_->get_logger(),"CLC Request correctly received");
        }

        // extract mobility header
        std::string req_sender_id = incoming_request.m_header.sender_id;
        std::string req_plan_id = incoming_request.m_header.plan_id;
        // extract mobility request
        carma_v2x_msgs::msg::LocationECEF ecef_location = incoming_request.location;
        carma_v2x_msgs::msg::Trajectory incoming_trajectory = incoming_request.trajectory;
        std::string req_strategy_params = incoming_request.strategy_params;
        clc_urgency_ = incoming_request.urgency;
        RCLCPP_DEBUG_STREAM(nh_->get_logger(),"received urgency: " << clc_urgency_);

        // Parse strategy parameters
        using boost::property_tree::ptree;
        ptree pt;
        std::istringstream strstream(req_strategy_params);
        boost::property_tree::json_parser::read_json(strstream, pt);
        int req_traj_speed_full = pt.get<int>("s");
        int req_traj_fractional = pt.get<int>("f");
        int start_lanelet_id = pt.get<int>("sl");
        int end_lanelet_id = pt.get<int>("el");
        double req_traj_speed = static_cast<double>(req_traj_speed_full) + static_cast<double>(req_traj_fractional)/10.0;
        RCLCPP_DEBUG_STREAM(nh_->get_logger(),"req_traj_speed" << req_traj_speed);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(),"start_lanelet_id" << start_lanelet_id);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(),"end_lanelet_id" << end_lanelet_id);

        std::vector<lanelet::BasicPoint2d> req_traj_plan = {};

        req_traj_plan = convert_eceftrajectory_to_mappoints(incoming_trajectory);

        double req_expiration_sec = static_cast<double>(incoming_request.expiration);
        double current_time_sec = nh_->now().seconds();

        bool response_to_clc_req = false;
        // ensure there is enough time for the yield
        double req_plan_time = req_expiration_sec - current_time_sec;
        double req_timestamp = static_cast<double>(incoming_request.m_header.timestamp) / 1000.0 - current_time_sec;
        set_incoming_request_info(req_traj_plan, req_traj_speed, req_plan_time, req_timestamp);


        if (req_expiration_sec - current_time_sec >= config_.min_obj_avoidance_plan_time_in_s && cooperative_request_acceptable_)
        {
          timesteps_since_last_req_ = 0;
          lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::REQUEST_ACCEPTED;
          lc_status_msg.description = "Accepted lane merge request";
          response_to_clc_req = true;
          RCLCPP_DEBUG(nh_->get_logger(),"CLC accepted");
        }
        else
        {
          lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::REQUEST_REJECTED;
          lc_status_msg.description = "Rejected lane merge request";
          response_to_clc_req = false;
          RCLCPP_DEBUG(nh_->get_logger(),"CLC rejected");
        }
        carma_v2x_msgs::msg::MobilityResponse outgoing_response = compose_mobility_response(req_sender_id, req_plan_id, response_to_clc_req);
        mobility_response_publisher_(outgoing_response);
        lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::RESPONSE_SENT;
        RCLCPP_DEBUG(nh_->get_logger(),"response sent");
      }
    }
    lc_status_publisher_(lc_status_msg);

  }

  void YieldPlugin::set_incoming_request_info(std::vector <lanelet::BasicPoint2d> req_trajectory, double req_speed, double req_planning_time, double req_timestamp)
  {
    req_trajectory_points_ = req_trajectory;
    req_target_speed_ = req_speed;
    req_target_plan_time_ = req_planning_time;
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"req_target_plan_time_" << req_target_plan_time_);
    req_timestamp_ = req_timestamp;
  }

  void YieldPlugin::bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg)
  {
    carma_v2x_msgs::msg::BSMCoreData bsm_core_ = msg->core_data;
    host_bsm_id_ = bsmIDtoString(bsm_core_);
  }

 void YieldPlugin::plan_trajectory_callback(
  carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req,
  carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
{
    RCLCPP_DEBUG(nh_->get_logger(),"Yield_plugin was called!");
    if (req->initial_trajectory_plan.trajectory_points.size() < 2){
      throw std::invalid_argument("Empty Trajectory received by Yield");
    }
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time start_time = system_clock.now();  // Start timing the execution time for planning so it can be logged

    carma_planning_msgs::msg::TrajectoryPlan original_trajectory = req->initial_trajectory_plan;
    carma_planning_msgs::msg::TrajectoryPlan yield_trajectory;

    // seperating cooperative yield with regular object detection for better performance.
    if (config_.enable_cooperative_behavior && clc_urgency_ > config_.acceptable_urgency)
    {
      RCLCPP_DEBUG(nh_->get_logger(),"Only consider high urgency clc");
      if (timesteps_since_last_req_ < config_.acceptable_passed_timesteps)
      {
        RCLCPP_DEBUG(nh_->get_logger(),"Yield for CLC. We haven't received an updated negotiation this timestep");
        yield_trajectory = update_traj_for_cooperative_behavior(original_trajectory, req->vehicle_state.longitudinal_vel);
        timesteps_since_last_req_++;
      }
      else
      {
        RCLCPP_DEBUG(nh_->get_logger(),"unreliable CLC communication, switching to object avoidance");
        yield_trajectory = update_traj_for_object(original_trajectory, external_objects_, req->vehicle_state.longitudinal_vel); // Compute the trajectory
      }
    }
    else
    {
      RCLCPP_DEBUG(nh_->get_logger(),"Yield for object avoidance");
      yield_trajectory = update_traj_for_object(original_trajectory, external_objects_, req->vehicle_state.longitudinal_vel); // Compute the trajectory
    }

    // return original trajectory if no difference in trajectory points a.k.a no collision
    if (fabs(get_trajectory_end_time(original_trajectory) - get_trajectory_end_time(yield_trajectory)) < EPSILON)
    {
      resp->trajectory_plan = original_trajectory;
    }
    else
    {
      yield_trajectory.header.frame_id = "map";
      yield_trajectory.header.stamp = nh_->now();
      yield_trajectory.trajectory_id = original_trajectory.trajectory_id;
      resp->trajectory_plan = yield_trajectory;
    }

    rclcpp::Time end_time = system_clock.now();  // Planning complete

    auto duration = end_time - start_time;
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ExecutionTime: " << std::to_string(duration.seconds()));

  }

  carma_planning_msgs::msg::TrajectoryPlan YieldPlugin::update_traj_for_cooperative_behavior(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double current_speed)
  {
    carma_planning_msgs::msg::TrajectoryPlan cooperative_trajectory;

    double initial_pos = 0;
    double goal_pos;
    double initial_velocity = current_speed;
    double goal_velocity = req_target_speed_;
    double planning_time = req_target_plan_time_;

    std::vector<lanelet::BasicPoint2d> host_traj_points = {};
    for (size_t i=0; i<original_tp.trajectory_points.size(); i++)
    {
      lanelet::BasicPoint2d traj_point;
      traj_point.x() = original_tp.trajectory_points.at(i).x;
      traj_point.y() = original_tp.trajectory_points.at(i).y;
      host_traj_points.push_back(traj_point);
    }

    std::vector<std::pair<int, lanelet::BasicPoint2d>> intersection_points = detect_trajectories_intersection(host_traj_points, req_trajectory_points_);
    if (!intersection_points.empty())
    {
      lanelet::BasicPoint2d intersection_point = intersection_points[0].second;
      double dx = original_tp.trajectory_points[0].x - intersection_point.x();
      double dy = original_tp.trajectory_points[0].y - intersection_point.y();
      // check if a digital_gap is available
      double digital_gap = check_traj_for_digital_min_gap(original_tp);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"digital_gap: " << digital_gap);
      goal_pos = sqrt(dx*dx + dy*dy) - config_.minimum_safety_gap_in_meters;
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Goal position (goal_pos): " << goal_pos);
      double collision_time = req_timestamp_ + (intersection_points[0].first * ecef_traj_timestep_) - config_.safety_collision_time_gap_in_s;
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"req time stamp: " << req_timestamp_);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Collision time: " << collision_time);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"intersection num: " << intersection_points[0].first);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Planning time: " << planning_time);
      // calculate distance traveled from beginning of trajectory to collision point
      double dx2 = intersection_point.x() - req_trajectory_points_[0].x();
      double dy2 = intersection_point.y() - req_trajectory_points_[0].y();
      // calculate incoming trajectory speed from time and distance between trajectory points
      double incoming_trajectory_speed = sqrt(dx2*dx2 + dy2*dy2)/(intersection_points[0].first * ecef_traj_timestep_);
      // calculate goal velocity from request trajectory
      goal_velocity = std::min(goal_velocity, incoming_trajectory_speed);
      double min_time = (initial_velocity - goal_velocity)/config_.yield_max_deceleration_in_ms2;

      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"goal_velocity: " << goal_velocity);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"incoming_trajectory_speed: " << incoming_trajectory_speed);

      if (planning_time > min_time)
      {
        cooperative_request_acceptable_ = true;
        double original_max_speed = max_trajectory_speed(original_tp.trajectory_points, get_trajectory_end_time(original_tp));
        cooperative_trajectory = generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, planning_time, original_max_speed);
      }
      else
      {
        cooperative_request_acceptable_ = false;
        RCLCPP_DEBUG(nh_->get_logger(),"The incoming requested trajectory is rejected, due to insufficient gap");
        cooperative_trajectory = original_tp;
      }

    }
    else
    {
      cooperative_request_acceptable_ = true;
      RCLCPP_DEBUG(nh_->get_logger(),"The incoming requested trajectory does not overlap with host vehicle's trajectory");
      cooperative_trajectory = original_tp;
    }

    return cooperative_trajectory;
  }

  double get_smallest_time_step_of_traj(const carma_planning_msgs::msg::TrajectoryPlan& original_tp)
  {
    double smallest_time_step = std::numeric_limits<double>::max();
    for (size_t i = 0; i < original_tp.trajectory_points.size() - 1; i ++)
    {
      smallest_time_step = std::min(smallest_time_step,
        (rclcpp::Time(original_tp.trajectory_points.at(i + 1).target_time)
        - rclcpp::Time(original_tp.trajectory_points.at(i).target_time)).seconds());
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"smallest_time_step: " << smallest_time_step);

    return smallest_time_step;
  }

  carma_planning_msgs::msg::TrajectoryPlan YieldPlugin::generate_JMT_trajectory(const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double initial_pos, double goal_pos,
    double initial_velocity, double goal_velocity, double planning_time, double original_max_speed)
  {
    carma_planning_msgs::msg::TrajectoryPlan jmt_trajectory;
    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> jmt_trajectory_points;
    jmt_trajectory_points.push_back(original_tp.trajectory_points[0]);

    std::vector<double> original_traj_relative_downtracks = get_relative_downtracks(original_tp);
    std::vector<double> calculated_speeds = {};
    std::vector<double> new_relative_downtracks = {};
    new_relative_downtracks.push_back(0.0);
    calculated_speeds.push_back(initial_velocity);
    double new_traj_accumulated_downtrack = 0.0;
    double original_traj_accumulated_downtrack = original_traj_relative_downtracks.at(1);

    // Up until goal_pos (which also can be until end of the entire original trajectory), generate new speeds at
    // or near original trajectory points by generating them at a fixed time interval using the JMT polynomial equation
    const double initial_time = 0;
    const double initial_accel = 0.0;
    const double goal_accel = 0.0;
    int new_traj_idx = 1;
    int original_traj_idx = 1;

    // Get the polynomial solutions used to generate the trajectory
    std::vector<double> polynomial_coefficients = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos,
                                                                                                goal_pos,
                                                                                                initial_velocity,
                                                                                                goal_velocity,
                                                                                                initial_accel,
                                                                                                goal_accel,
                                                                                                initial_time,
                                                                                                planning_time);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Used original_max_speed: " << original_max_speed);
    const auto smallest_time_step = get_smallest_time_step_of_traj(original_tp);
    while (new_traj_accumulated_downtrack < goal_pos && original_traj_idx < original_traj_relative_downtracks.size())
    {
      const double target_time = new_traj_idx * smallest_time_step;
      const double downtrack_at_target_time = polynomial_calc(polynomial_coefficients, target_time);
      double velocity_at_target_time = polynomial_calc_d(polynomial_coefficients, target_time);

      RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Calculated speed velocity_at_target_time: " << velocity_at_target_time
        << ", downtrack_at_target_time: "<< downtrack_at_target_time << ", target_time: " << target_time);

      // Cannot have a negative speed or have a higher speed than that of the original trajectory
      velocity_at_target_time = std::clamp(velocity_at_target_time, 0.0, original_max_speed);

      // Pick the speed if it matches with the original downtracks
      if (downtrack_at_target_time >= original_traj_accumulated_downtrack)
      {
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Picked calculated speed velocity_at_target_time: " << velocity_at_target_time
          << ", downtrack_at_target_time: "<< downtrack_at_target_time << ", target_time: " << target_time);
        // velocity_at_target_time doesn't exactly correspond to original_traj_accumulated_downtrack but does for new_traj_accumulated_downtrack.
        // however, the logic is assuming they are close enough that the speed is usable
        calculated_speeds.push_back(velocity_at_target_time);
        original_traj_accumulated_downtrack += original_traj_relative_downtracks.at(original_traj_idx);
        original_traj_idx ++;
      }
      new_traj_accumulated_downtrack = downtrack_at_target_time;
      new_traj_idx++;
    }

    // Moving average filter to smoothen the speeds
    std::vector<double> filtered_speeds = basic_autonomy::smoothing::moving_average_filter(calculated_speeds, config_.speed_moving_average_window_size);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "filtered_speeds size: " << filtered_speeds.size());

    // Replace the original trajectory's associated timestamps based on the newly calculated speeds
    double prev_speed = filtered_speeds.at(0);
    for(size_t i = 1; i < original_tp.trajectory_points.size(); i++)
    {
      carma_planning_msgs::msg::TrajectoryPlanPoint jmt_tpp = original_tp.trajectory_points.at(i);

      // In case only subset of original trajectory needs modification,
      // the rest of the points should keep the last speed to cruise
      double current_speed = goal_velocity;

      if (i < filtered_speeds.size())
      {
        current_speed = filtered_speeds.at(i);
      }

      //Force the speed to 0 if below configured value for more control over stopping behavior
      if (current_speed < config_.max_stop_speed_in_ms)
      {
        current_speed = 0;
      }

      // Derived from constant accelaration kinematic equation: (vi + vf) / 2 * dt = d_dist
      // This also handles a case correctly when current_speed is 0, but prev_speed is not 0 yet
      const double dt = (2 * original_traj_relative_downtracks.at(i)) / (current_speed + prev_speed);
      jmt_tpp.target_time =  rclcpp::Time(jmt_trajectory_points.back().target_time) + rclcpp::Duration(dt*1e9);

      if (prev_speed < EPSILON) // Handle a special case if prev_speed (thus current_speed too) is 0
      {
        // NOTE: Assigning arbitrary 100 mins dt between points where normally dt is only 1 sec to model a stopping behavior.
        // Another way to model it is to keep the trajectory point at a same location and increment time slightly. However,
        // if the vehicle goes past the point, it may cruise toward undesirable location (for example into the intersection).
        // Keeping the points help the controller steer the vehicle toward direction of travel even when stopping.
        // Only downside is the trajectory plan is huge where only 15 sec is expected, but since this is stopping case, it shouldn't matter.
        jmt_tpp.target_time = rclcpp::Time(jmt_trajectory_points.back().target_time) + rclcpp::Duration(6000 * 1e9);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Zero speed = x: " << jmt_tpp.x << ", y:" << jmt_tpp.y
          << ", t:" << std::to_string(rclcpp::Time(jmt_tpp.target_time).seconds())
          << ", prev_speed: " << prev_speed << ", current_speed: " << current_speed);
      }
      else
      {
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Non-zero speed = x: " << jmt_tpp.x << ", y:" << jmt_tpp.y
          << ", t:" << std::to_string(rclcpp::Time(jmt_tpp.target_time).seconds())
          << ", prev_speed: " << prev_speed << ", current_speed: " << current_speed);
      }

      jmt_trajectory_points.push_back(jmt_tpp);
      prev_speed = current_speed;
    }

    jmt_trajectory.header = original_tp.header;
    jmt_trajectory.trajectory_id = original_tp.trajectory_id;
    jmt_trajectory.trajectory_points = jmt_trajectory_points;
    jmt_trajectory.initial_longitudinal_velocity = initial_velocity;
    return jmt_trajectory;
  }

  std::optional<rclcpp::Time> YieldPlugin::get_collision_time(const carma_planning_msgs::msg::TrajectoryPlan& trajectory1,
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2, double collision_radius)
  {

    // Iterate through each pair of consecutive points in the trajectories

    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Starting a new collision detection, trajectory size: "
      << trajectory1.trajectory_points.size() << ". prediction size: " << trajectory2.size());

    // Iterate through the object to check if it's on the route
    bool on_route = false;
    int on_route_idx = 0;

    // A flag to stop searching more than one lanelet if the object has no velocity
    double traj2_velocity = fabs(sqrt(pow(trajectory2.front().predicted_velocity.linear.x, 2) + pow(trajectory2.front().predicted_velocity.linear.y, 2)));
    bool traj2_has_zero_velocity = traj2_velocity < config_.min_obstacle_speed_in_ms;
    // For loop's number of predicted states to skip that is required to check every 1.5 meter of the traj2
    // 1.5 meter is experimental number which results in the best performance without significantly reducing precision
    if (trajectory2.size() < 2)
    {
      throw std::invalid_argument("Object on ther road doesn't have enough predicted states! Please check motion_computation is correctly applying predicted states");
    }
    const double predict_step_duration = fabs((rclcpp::Time(trajectory2.at(1).header.stamp) - rclcpp::Time(trajectory2.front().header.stamp)).seconds());
    const int steps_to_take = std::max(1, static_cast<int>(1.5 / (traj2_velocity * predict_step_duration)));

    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Determined steps_to_take: " << steps_to_take << ", with traj2_velocity: " << traj2_velocity << ", predict_step_duration: " << predict_step_duration);

    for (size_t j = 0; j < trajectory2.size(); j += steps_to_take) // Saving computation time aiming for 1.0 meter interval
    {
      lanelet::BasicPoint2d curr_point;
      curr_point.x() = trajectory2.at(j).predicted_position.position.x;
      curr_point.y() = trajectory2.at(j).predicted_position.position.y;

      auto corresponding_lanelets = wm_->getLaneletsFromPoint(curr_point, 8); // some intersection can have 8 overlapping lanelets

      for (const auto& llt: corresponding_lanelets)
      {
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Checking llt: " << llt.id());

        if (remaining_route_llt_ids_.find(llt.id()) != remaining_route_llt_ids_.end())
        {
          on_route = true;
          on_route_idx = j;
          break;
        }
      }
      if (on_route || traj2_has_zero_velocity)
        break;
    }

    if (!on_route)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Predicted states are not on the route! ignoring");
      return std::nullopt;
    }

    double smallest_dist = std::numeric_limits<double>::max();
    for (size_t i = 0; i < trajectory1.trajectory_points.size() - 1; ++i)
    {
      auto p1a = trajectory1.trajectory_points.at(i);
      auto p1b = trajectory1.trajectory_points.at(i + 1);
      double previous_distance_between_predictions = std::numeric_limits<double>::max();
      for (size_t j = on_route_idx; j < trajectory2.size() - 1; j += steps_to_take)
      {
        auto p2a = trajectory2.at(j);
        auto p2b = trajectory2.at(j + 1);
        double p1a_t = rclcpp::Time(p1a.target_time).seconds();
        double p1b_t = rclcpp::Time(p1b.target_time).seconds();
        double p2a_t = rclcpp::Time(p2a.header.stamp).seconds();
        double p2b_t = rclcpp::Time(p2b.header.stamp).seconds();

        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p1a.target_time: " << std::to_string(p1a_t) << ", p1b.target_time: " << std::to_string(p1b_t));
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p2a.target_time: " << std::to_string(p2a_t) << ", p2b.target_time: " << std::to_string(p2b_t));
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p1a.x: " << p1a.x << ", p1a.y: " << p1a.y);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p1b.x: " << p1b.x << ", p1b.y: " << p1b.y);

        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p2a.x: " << p2a.predicted_position.position.x << ", p2a.y: " << p2a.predicted_position.position.y);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "p2b.x: " << p2b.predicted_position.position.x << ", p2b.y: " << p2b.predicted_position.position.y);

        // Linearly interpolate positions at a common timestamp for both trajectories
        double dt = (p2a_t - p1a_t) / (p1b_t - p1a_t);
        double x1 = p1a.x + dt * (p1b.x - p1a.x);
        double y1 = p1a.y + dt * (p1b.y - p1a.y);
        double x2 = p2a.predicted_position.position.x;
        double y2 = p2a.predicted_position.position.y;

        // Calculate the distance between the two interpolated points
        double distance = std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));

        smallest_dist = std::min(distance, smallest_dist);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Smallest_dist: " << smallest_dist << ", distance: " << distance << ", dt: " << dt
          << ", x1: " << x1 << ", y1: " << y1
          << ", x2: " << x2 << ", y2: " << y2
          << ", p2a_t:" << std::to_string(p2a_t));

        // Following "if logic" assumes the traj2 is a simple cv model, aka, traj2 point is a straight line over time.
        // And current traj1 point is fixed in this iteration.
        // Then once the distance between the two start to increase over traj2 iteration,
        // the distance will always increase and it's unnecessary to continue the logic to find the smallest_dist
        if (previous_distance_between_predictions < distance)
        {
          RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Stopping search here because the distance between predictions started to increase");
          break;
        }
        previous_distance_between_predictions = distance;

        if (i == 0 && j == 0 && distance > config_.collision_check_radius_in_m)
        {
          RCLCPP_DEBUG(nh_->get_logger(), "Too far away" );
          return std::nullopt;
        }

        if (distance > collision_radius)
        {
          if (traj2_has_zero_velocity)
          {
            // If no collision is detect by now, stop searching since traj2 is not moving
            return std::nullopt;
          }
          // continue searching for collision
          continue;
        }

        // if within collision radius
        lanelet::BasicPoint2d vehicle_point(x1,y1);
        lanelet::BasicPoint2d object_point(x2,y2);
        double vehicle_downtrack = wm_->routeTrackPos(vehicle_point).downtrack;
        double object_downtrack = wm_->routeTrackPos(object_point).downtrack;

        if (vehicle_downtrack > object_downtrack + config_.vehicle_length / 2)  // if half a length of the vehicle past, it is considered behind
        {
          RCLCPP_INFO_STREAM(nh_->get_logger(), "Detected an object nearby behind the vehicle at timestamp " << std::to_string(p2a_t));
          return std::nullopt;
        }
        else
        {
          RCLCPP_WARN_STREAM(nh_->get_logger(), "Collision detected at timestamp " << std::to_string(p2a_t) << ", x: " << x1 << ", y: " << y1 <<
            ", within actual downtrack distance: " << object_downtrack - vehicle_downtrack <<
            ", and collision distance: " << distance);
          return rclcpp::Time(p2a.header.stamp);
        }
      }
    }

    // No collision detected
    return std::nullopt;
  }

  std::optional<rclcpp::Time> YieldPlugin::get_collision_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const carma_perception_msgs::msg::ExternalObject& curr_obstacle)
  {
    auto plan_start_time = get_trajectory_start_time(original_tp);

    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Object's back time: " << std::to_string(rclcpp::Time(curr_obstacle.predictions.back().header.stamp).seconds())
      << ", plan_start_time: " << std::to_string(plan_start_time));

    // do not process outdated objects
    if (rclcpp::Time(curr_obstacle.predictions.back().header.stamp).seconds() <= plan_start_time)
    {
      return std::nullopt;
    }

    std::vector<carma_perception_msgs::msg::PredictedState> new_list;
    carma_perception_msgs::msg::PredictedState curr_state;
    // artificially include current position as one of the predicted states
    curr_state.header.stamp = curr_obstacle.header.stamp;
    curr_state.predicted_position.position.x = curr_obstacle.pose.pose.position.x;
    curr_state.predicted_position.position.y = curr_obstacle.pose.pose.position.y;
    // NOTE: predicted_velocity is not used for collision calculation, but timestamps
    curr_state.predicted_velocity.linear.x = curr_obstacle.velocity.twist.linear.x;
    curr_state.predicted_velocity.linear.y = curr_obstacle.velocity.twist.linear.y;
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Object: " << curr_obstacle.id <<", type: " << static_cast<int>(curr_obstacle.object_type)
      << ", speed_x: " << curr_obstacle.velocity.twist.linear.x  << ", speed_y: " << curr_obstacle.velocity.twist.linear.y);
    new_list.push_back(curr_state);
    new_list.insert(new_list.end(), curr_obstacle.predictions.cbegin(), curr_obstacle.predictions.cend());

    return get_collision_time(original_tp, new_list, config_.intervehicle_collision_distance_in_m);
  }

  std::optional<std::pair<carma_perception_msgs::msg::ExternalObject, double>> YieldPlugin::get_earliest_collision_object_and_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ExternalObjects size: " << external_objects.size());

    if (!wm_->getRoute())
    {
      RCLCPP_WARN(nh_->get_logger(), "Yield plugin was not able to analyze collision since route is not available! Please check if route is set");
      return std::nullopt;
    }

    // save route Ids for faster access
    for (const auto& llt: wm_->getRoute()->shortestPath())
    {
      remaining_route_llt_ids_.insert(llt.id());
    }

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"External Object List (external_objects) size: " << external_objects.size());

    std::map<std::uint32_t, rclcpp::Time> collision_times;
    for (const auto & object : external_objects) {
      if (const auto collision_time { get_collision_time(original_tp, object) } ) {
        collision_times[object.id] = collision_time.value();
      }
    }

    if (collision_times.empty()) { return std::nullopt; }

    const auto earliest_colliding_object_id{std::min_element(
      std::cbegin(collision_times), std::cend(collision_times),
      [](const auto & a, const auto & b){ return a.second < b.second; })->first};

    const auto earliest_colliding_object{std::find_if(
      std::cbegin(external_objects), std::cend(external_objects),
      [&earliest_colliding_object_id](const auto & object) { return object.id == earliest_colliding_object_id; })};

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"earliest object x: " << earliest_colliding_object->velocity.twist.linear.x
        << ", y: " << earliest_colliding_object->velocity.twist.linear.y);
    return std::make_pair(*earliest_colliding_object, collision_times.at(earliest_colliding_object_id).seconds());

  }

  double YieldPlugin::get_predicted_velocity_at_time(const geometry_msgs::msg::Twist& object_velocity_in_map_frame,
    const carma_planning_msgs::msg::TrajectoryPlan& original_tp, double timestamp_in_sec_to_predict)
  {
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "timestamp_in_sec_to_predict: " << std::to_string(timestamp_in_sec_to_predict) <<
      ", trajectory_end_time: " << std::to_string(get_trajectory_end_time(original_tp)));

    double point_b_time = 0.0;
    carma_planning_msgs::msg::TrajectoryPlanPoint point_a;
    carma_planning_msgs::msg::TrajectoryPlanPoint point_b;

    // trajectory points' time is guaranteed to be increasing
    // then find the corresponding point at timestamp_in_sec_to_predict
    for (size_t i = 0; i < original_tp.trajectory_points.size() - 1; ++i)
    {
      point_a = original_tp.trajectory_points.at(i);
      point_b = original_tp.trajectory_points.at(i + 1);
      point_b_time = rclcpp::Time(point_b.target_time).seconds();
      if (point_b_time >= timestamp_in_sec_to_predict)
      {
        break;
      }
    }

    auto dx = point_b.x - point_a.x;
    auto dy = point_b.y - point_a.y;
    const tf2::Vector3 trajectory_direction(dx, dy, 0);

    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "timestamp_in_sec_to_predict: " << std::to_string(timestamp_in_sec_to_predict)
      << ", point_b_time: " << std::to_string(point_b_time)
      << ", dx: " << dx << ", dy: " << dy << ", "
      << ", object_velocity_in_map_frame.x: " << object_velocity_in_map_frame.linear.x
      << ", object_velocity_in_map_frame.y: " << object_velocity_in_map_frame.linear.y);

    if (trajectory_direction.length() < 0.001) //EPSILON
    {
      return 0.0;
    }

    const tf2::Vector3 object_direction(object_velocity_in_map_frame.linear.x, object_velocity_in_map_frame.linear.y, 0);

    return tf2::tf2Dot(object_direction, trajectory_direction) / trajectory_direction.length();
  }

  carma_planning_msgs::msg::TrajectoryPlan YieldPlugin::update_traj_for_object(const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects, double initial_velocity)
  {
    if (original_tp.trajectory_points.size() < 2)
    {
      RCLCPP_WARN(nh_->get_logger(), "Yield plugin received less than 2 points in update_traj_for_object, returning unchanged...");
      return original_tp;
    }

    // Get earliest collision object
    const auto earliest_collision_obj_pair = get_earliest_collision_object_and_time(original_tp, external_objects);

    if (!earliest_collision_obj_pair)
    {
      RCLCPP_DEBUG(nh_->get_logger(),"No collision detected, so trajectory not modified.");
      return original_tp;
    }

    carma_perception_msgs::msg::ExternalObject earliest_collision_obj = earliest_collision_obj_pair.value().first;
    double earliest_collision_time_in_seconds = earliest_collision_obj_pair.value().second;

    // Issue (https://github.com/usdot-fhwa-stol/carma-platform/issues/2155): If the yield_plugin can detect if the roadway object is moving along the route,
    // it is able to plan yielding much earlier and smoother using on_route_vehicle_collision_horizon_in_s.

    const lanelet::BasicPoint2d vehicle_point(original_tp.trajectory_points[0].x,original_tp.trajectory_points[0].y);
    const double vehicle_downtrack = wm_->routeTrackPos(vehicle_point).downtrack;

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"vehicle_downtrack: " << vehicle_downtrack);

    RCLCPP_WARN_STREAM(nh_->get_logger(),"Collision Detected!");

    const lanelet::BasicPoint2d object_point(earliest_collision_obj.pose.pose.position.x, earliest_collision_obj.pose.pose.position.y);
    const double object_downtrack = wm_->routeTrackPos(object_point).downtrack;

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object_downtrack: " << object_downtrack);

    const double object_downtrack_lead = std::max(0.0, object_downtrack - vehicle_downtrack);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object_downtrack_lead: " << object_downtrack_lead);

    // The vehicle's goal velocity of the yielding behavior is to match the velocity of the object along the trajectory.
    double goal_velocity = get_predicted_velocity_at_time(earliest_collision_obj.velocity.twist, original_tp, earliest_collision_time_in_seconds);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object's speed along trajectory at collision: " << goal_velocity);

    // roadway object position
    const double gap_time_until_min_gap_distance = std::max(0.0, object_downtrack_lead - config_.minimum_safety_gap_in_meters)/initial_velocity;

    if (goal_velocity <= config_.min_obstacle_speed_in_ms){
      RCLCPP_WARN_STREAM(nh_->get_logger(),"The obstacle is not moving, goal velocity is set to 0 from: " << goal_velocity);
      goal_velocity = 0.0;
    }

    // determine the safety inter-vehicle gap based on speed
    double safety_gap = std::max(goal_velocity * gap_time_until_min_gap_distance, config_.minimum_safety_gap_in_meters);
    if (!std::isnormal(safety_gap))
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("yield_plugin"),"Detected non-normal (nan, inf, etc.) safety_gap."
        "Making it desired safety gap configured at config_.minimum_safety_gap_in_meters: " << config_.minimum_safety_gap_in_meters);
      safety_gap = config_.minimum_safety_gap_in_meters;
    }
    if (config_.enable_adjustable_gap)
    {
      // externally_commanded_safety_gap is desired distance gap commanded from external sources
      // such as different plugin, map, or infrastructure depending on the use case
      double externally_commanded_safety_gap = check_traj_for_digital_min_gap(original_tp);
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"externally_commanded_safety_gap: " << externally_commanded_safety_gap);
      // if a digital gap is available, it is replaced as safety gap
      safety_gap = std::max(safety_gap, externally_commanded_safety_gap);
    }

    const double goal_pos = std::max(0.0, object_downtrack_lead - safety_gap);
    const double initial_pos = 0.0; //relative initial position (first trajectory point)
    const double original_max_speed = max_trajectory_speed(original_tp.trajectory_points, earliest_collision_time_in_seconds);
    const double delta_v_max = fabs(goal_velocity - original_max_speed);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"delta_v_max: " << delta_v_max << ", safety_gap: " << safety_gap);

    // reference time, is the maximum time available to perform object avoidance (length of a trajectory)
    const double max_allowed_trajectory_duration_in_s = get_trajectory_duration(original_tp);
    // time required for comfortable deceleration
    const double time_required_for_comfortable_decel_in_s = config_.acceleration_adjustment_factor * delta_v_max / config_.yield_max_deceleration_in_ms2;

    // planning time for object avoidance
    double planning_time_in_s = 0;

    if (config_.min_obj_avoidance_plan_time_in_s < time_required_for_comfortable_decel_in_s
      && time_required_for_comfortable_decel_in_s < max_allowed_trajectory_duration_in_s)
    {
      planning_time_in_s = time_required_for_comfortable_decel_in_s;
    }
    else if(time_required_for_comfortable_decel_in_s < config_.min_obj_avoidance_plan_time_in_s)
    {
      planning_time_in_s = config_.min_obj_avoidance_plan_time_in_s;
    }
    else
    {
      planning_time_in_s = max_allowed_trajectory_duration_in_s;
    }

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Object avoidance planning time: " << planning_time_in_s);

    return generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, planning_time_in_s, original_max_speed);;
  }


  std::vector<double> YieldPlugin::get_relative_downtracks(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan) const
  {
    std::vector<double> downtracks;
    downtracks.reserve(trajectory_plan.trajectory_points.size());
    // relative downtrack distance of the fist Point is 0.0
    downtracks.push_back(0.0);
    for (size_t i=1; i < trajectory_plan.trajectory_points.size(); i++){

      double dx = trajectory_plan.trajectory_points.at(i).x - trajectory_plan.trajectory_points.at(i-1).x;
      double dy = trajectory_plan.trajectory_points.at(i).y - trajectory_plan.trajectory_points.at(i-1).y;
      downtracks.push_back(sqrt(dx*dx + dy*dy));
    }
    return downtracks;
  }

  double YieldPlugin::polynomial_calc(std::vector<double> coeff, double x) const
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size(); i++)
    {
      double value = coeff.at(i) * pow(x, static_cast<int>(coeff.size() - 1 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::polynomial_calc_d(std::vector<double> coeff, double x) const
  {
    double result = 0;
    for (size_t i = 0; i < coeff.size()-1; i++)
    {
      double value = static_cast<int>(coeff.size() - 1 - i) * coeff.at(i) * pow(x, static_cast<int>(coeff.size() - 2 - i));
      result = result + value;
    }
    return result;
  }

  double YieldPlugin::max_trajectory_speed(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_points, double timestamp_in_sec_to_search_until) const
  {
    double max_speed = 0;
    for(size_t i = 0; i < trajectory_points.size() - 2; i++ )
    {
      double dx = trajectory_points.at(i + 1).x - trajectory_points.at(i).x;
      double dy = trajectory_points.at(i + 1).y - trajectory_points.at(i).y;
      double d = sqrt(dx*dx + dy*dy);
      double t = (rclcpp::Time(trajectory_points.at(i + 1).target_time).seconds() - rclcpp::Time(trajectory_points.at(i).target_time).seconds());
      double v = d/t;
      if(v > max_speed)
      {
        max_speed = v;
      }
      if (rclcpp::Time(trajectory_points.at(i + 1).target_time).seconds() >= timestamp_in_sec_to_search_until)
      {
        break;
      }

    }
    return max_speed;
  }

  double YieldPlugin::check_traj_for_digital_min_gap(const carma_planning_msgs::msg::TrajectoryPlan& original_tp) const
  {
    double desired_gap = 0;

    for (size_t i = 0; i < original_tp.trajectory_points.size(); i++)
    {
      lanelet::BasicPoint2d veh_pos(original_tp.trajectory_points.at(i).x, original_tp.trajectory_points.at(i).y);
      auto llts = wm_->getLaneletsFromPoint(veh_pos, 1);
      if (llts.empty())
      {
       RCLCPP_WARN_STREAM(nh_->get_logger(),"Trajectory point: x= " << original_tp.trajectory_points.at(i).x << "y="<< original_tp.trajectory_points.at(i).y);

        throw std::invalid_argument("Trajectory Point is not on a valid lanelet.");
      }
      auto digital_min_gap = llts[0].regulatoryElementsAs<lanelet::DigitalMinimumGap>(); //Returns a list of these elements)
      if (!digital_min_gap.empty())
      {
        double digital_gap = digital_min_gap[0]->getMinimumGap(); // Provided gap is in meters
        RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Digital Gap found with value: " << digital_gap);
        desired_gap = std::max(desired_gap, digital_gap);
      }
    }
    return desired_gap;
  }

  void YieldPlugin::set_georeference_string(const std::string& georeference)
  {
    map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(georeference.c_str());  // Build projector from proj string
  }

  void YieldPlugin::set_external_objects(const std::vector<carma_perception_msgs::msg::ExternalObject>& object_list)
  {
    external_objects_ = object_list;
  }

}  // namespace yield_plugin
