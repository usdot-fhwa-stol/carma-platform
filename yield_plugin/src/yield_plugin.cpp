/*
 * Copyright (C) 2022-2026 LEIDOS.
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
#include <future>
#include <thread>
#include <unordered_set>
#include <basic_autonomy/helper_functions.hpp>
#include <yield_plugin/yield_plugin_cuda.cuh>

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

  double get_trajectory_duration(const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory)
  {
    return (rclcpp::Time(trajectory.back().header.stamp) - rclcpp::Time(trajectory.front().header.stamp)).seconds();
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

    // if ego is not stopped and we committed to stopping, use the last committed trajectory
    if (req->vehicle_state.longitudinal_vel > EPSILON &&
      last_traj_plan_committed_to_stopping_.has_value())
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Using last committed trajectory to stopping");
      lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global,
            req->vehicle_state.y_pos_global);
      auto updated_trajectory = last_traj_plan_committed_to_stopping_.value();

      // Find closest point in last trajectory to current vehicle position
      size_t idx_to_start_new_traj =
          basic_autonomy::waypoint_generation::get_nearest_point_index(
              updated_trajectory.trajectory_points,
              veh_pos);

      // Update last trajectory to start from closest point (remove passed points)
      if (!updated_trajectory.trajectory_points.empty()) {
          updated_trajectory.trajectory_points =
              std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>
              (updated_trajectory.trajectory_points.begin() + idx_to_start_new_traj,
              updated_trajectory.trajectory_points.end());
      }
      last_traj_plan_committed_to_stopping_ = updated_trajectory;
      resp->trajectory_plan = last_traj_plan_committed_to_stopping_.value();

      rclcpp::Time end_time = system_clock.now();  // Planning complete

      auto duration = end_time - start_time;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
        "ExecutionTime Yield: " << std::to_string(duration.seconds()));
      return;
    }

    // Otherwise, we are planning a new trajectory by checking collision
    try
    {
      // NOTE: Wrapping entire plan_trajectory logic with try catch because there is intermittent
      // open issue of which cause is uncertain:
      // https://github.com/usdot-fhwa-stol/carma-platform/issues/2501

      double initial_velocity = req->vehicle_state.longitudinal_vel;
      // If vehicle_state is stopped, non-zero velocity from the trajectory
      // should be used. Otherwise, vehicle will not move.
      if (initial_velocity < EPSILON)
      {
        initial_velocity = original_trajectory.initial_longitudinal_velocity;
        // Record the time when vehicle was stopped first due to collision avoidance
        if (last_traj_plan_committed_to_stopping_.has_value() &&
          !first_time_stopped_to_prevent_collision_.has_value())
        {
          first_time_stopped_to_prevent_collision_ = nh_->now();
          RCLCPP_DEBUG(nh_->get_logger(), "First time stopped to prevent collision: %f",
            first_time_stopped_to_prevent_collision_.value().seconds());
        }
      }

      // seperating cooperative yield with regular object detection for better performance.
      if (config_.enable_cooperative_behavior && clc_urgency_ > config_.acceptable_urgency)
      {
        RCLCPP_DEBUG(nh_->get_logger(),"Only consider high urgency clc");
        if (timesteps_since_last_req_ < config_.acceptable_passed_timesteps)
        {
          RCLCPP_DEBUG(nh_->get_logger(),"Yield for CLC. We haven't received an updated negotiation this timestep");
          yield_trajectory = update_traj_for_cooperative_behavior(original_trajectory, initial_velocity);
          timesteps_since_last_req_++;
        }
        else
        {
          RCLCPP_DEBUG(nh_->get_logger(),"unreliable CLC communication, switching to object avoidance");
          yield_trajectory = update_traj_for_object(original_trajectory, external_objects_, initial_velocity); // Compute the trajectory
        }
      }
      else
      {
        RCLCPP_DEBUG(nh_->get_logger(),"Yield for object avoidance");
        auto _t0_upd = std::chrono::steady_clock::now();
        yield_trajectory = update_traj_for_object(original_trajectory, external_objects_, initial_velocity); // Compute the trajectory
        const double _upd_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_upd).count();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
          "[timing] update_traj_for_object: " << _upd_ms << " ms");
      }


      // return original trajectory if no difference in trajectory points a.k.a no collision
      if (fabs(get_trajectory_end_time(original_trajectory) - get_trajectory_end_time(yield_trajectory)) < EPSILON)
      {

        resp->trajectory_plan = original_trajectory;
        // Reset the collision prevention variables
        first_time_stopped_to_prevent_collision_ = std::nullopt;
        last_traj_plan_committed_to_stopping_ = std::nullopt;

      }
      else
      {
        yield_trajectory.header.frame_id = "map";
        yield_trajectory.header.stamp = nh_->now();
        yield_trajectory.trajectory_id = original_trajectory.trajectory_id;
        resp->trajectory_plan = yield_trajectory;
      }
    }
    catch(const std::runtime_error& e) {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Yield Plugin failed to plan trajectory due to known negative time issue: " << e.what());
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Returning the original trajectory, and retrying at the next call.");
      resp->trajectory_plan = original_trajectory;
    }

    rclcpp::Time end_time = system_clock.now();  // Planning complete

    auto duration = end_time - start_time;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "ExecutionTime Yield: " << std::to_string(duration.seconds()));
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
      goal_pos = sqrt(dx*dx + dy*dy) - std::max(config_.minimum_safety_gap_in_meters, digital_gap);
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
    double smallest_time_step = std::numeric_limits<double>::infinity();
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
    double initial_accel = 0;
    if (last_speed_ && last_speed_time_)
    {
      initial_accel = (initial_velocity - last_speed_.value()) /
        (nh_->now() - last_speed_time_.value()).seconds();

      if (!std::isnormal(initial_accel))
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"Detecting nan initial_accel set to 0");
        initial_accel = 0.0;
      }

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"Detecting initial_accel: " << initial_accel
        << ", initial_velocity:" << initial_velocity
        << ", last_speed_: " << last_speed_.value()
        << ", nh_->now(): " << nh_->now().seconds()
        << ", last_speed_time_.get(): " << last_speed_time_.value().seconds());
    }

    const double goal_accel = 0;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"Following parameters used for JMT: "
      "\ninitial_pos: " << initial_pos <<
      "\ngoal_pos: " << goal_pos <<
      "\ninitial_velocity: " << initial_velocity <<
      "\ngoal_velocity: " << goal_velocity <<
      "\ninitial_accel: " << initial_accel <<
      "\ngoal_accel: " << goal_accel <<
      "\nplanning_time: " << planning_time <<
      "\noriginal_max_speed: " << original_max_speed);

    // Get the polynomial solutions used to generate the trajectory
    std::vector<double> polynomial_coefficients = quintic_coefficient_calculator::quintic_coefficient_calculator(initial_pos,
                                                                                                goal_pos,
                                                                                                initial_velocity,
                                                                                                goal_velocity,
                                                                                                initial_accel,
                                                                                                goal_accel,
                                                                                                initial_time,
                                                                                                planning_time);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"Used original_max_speed: " << original_max_speed);
    for (size_t i = 0; i < polynomial_coefficients.size(); i++) {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),"Coefficient " << i << ": " << polynomial_coefficients[i]);
    }
    // Cap at 0.1 s: finer steps gave no accuracy benefit but caused O(1/dt) loop blow-up
    // when upstream planners produced sub-10 ms trajectory timesteps.
    const double smallest_time_step = std::max(get_smallest_time_step_of_traj(original_tp), 0.1);
    int new_traj_idx = 1;
    int original_traj_idx = 1;
    while (new_traj_accumulated_downtrack < goal_pos - EPSILON && original_traj_idx < original_traj_relative_downtracks.size())
    {
      const double target_time = new_traj_idx * smallest_time_step;
      const double downtrack_at_target_time = polynomial_calc(polynomial_coefficients, target_time);
      double velocity_at_target_time = polynomial_calc_d(polynomial_coefficients, target_time);

      // if the speed becomes negative, the downtrack starts reversing to negative as well
      // which will never reach the goal_pos, so break here.
      if (velocity_at_target_time < 0.0)
      {
        break;
      }

      // Cannot have a negative speed or have a higher speed than that of the original trajectory
      velocity_at_target_time = std::clamp(velocity_at_target_time, 0.0, original_max_speed);

      // Pick the speed if it matches with the original downtracks
      if (downtrack_at_target_time >= original_traj_accumulated_downtrack)
      {
        // velocity_at_target_time doesn't exactly correspond to original_traj_accumulated_downtrack but does for new_traj_accumulated_downtrack.
        // however, the logic is assuming they are close enough that the speed is usable
        calculated_speeds.push_back(velocity_at_target_time);
        original_traj_accumulated_downtrack += original_traj_relative_downtracks.at(original_traj_idx);
        original_traj_idx ++;
      }
      new_traj_accumulated_downtrack = downtrack_at_target_time;
      new_traj_idx++;

    }

    // if the loop above finished prematurely due to negative speed, fill with 0.0 speeds
    // since the speed crossed 0.0 and algorithm indicates stopping
    std::fill_n(std::back_inserter(calculated_speeds),
                std::size(original_traj_relative_downtracks) - std::size(calculated_speeds),
                0.0);

    // Moving average filter to smoothen the speeds
    std::vector<double> filtered_speeds = basic_autonomy::smoothing::moving_average_filter(calculated_speeds, config_.speed_moving_average_window_size);
    // Replace the original trajectory's associated timestamps based on the newly calculated speeds
    double prev_speed = filtered_speeds.at(0);
    last_speed_ = prev_speed;
    last_speed_time_ = nh_->now();

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
      jmt_tpp.target_time =  rclcpp::Time(jmt_trajectory_points.back().target_time) + rclcpp::Duration::from_nanoseconds(dt*1e9);

      if (prev_speed < EPSILON) // Handle a special case if prev_speed (thus current_speed too) is 0
      {
        // NOTE: Assigning arbitrary 100 mins dt between points where normally dt is only 1 sec to model a stopping behavior.
        // Another way to model it is to keep the trajectory point at a same location and increment time slightly. However,
        // if the vehicle goes past the point, it may cruise toward undesirable location (for example into the intersection).
        // Keeping the points help the controller steer the vehicle toward direction of travel even when stopping.
        // Only downside is the trajectory plan is huge where only 15 sec is expected, but since this is stopping case, it shouldn't matter.
        jmt_tpp.target_time = rclcpp::Time(jmt_trajectory_points.back().target_time) + rclcpp::Duration::from_nanoseconds(6000 * 1e9);
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

  std::optional<GetCollisionResult> YieldPlugin::get_collision(const carma_planning_msgs::msg::TrajectoryPlan& ego_trajectory,
    const std::vector<carma_perception_msgs::msg::PredictedState>& object_predictions, double collision_radius, double ego_max_speed)
  {

    // Iterate through each pair of consecutive points in the trajectories
    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Starting a new collision detection, trajectory size: "
      << ego_trajectory.trajectory_points.size() << ". prediction size: " << object_predictions.size());

    // Iterate through the object to check if it's on the route
    bool on_route = false;
    int on_route_idx = 0;

    // A flag to stop searching more than one lanelet if the object has no velocity
    const auto object_speed{std::hypot(object_predictions.front().predicted_velocity.linear.x,
                                  object_predictions.front().predicted_velocity.linear.y)};
    bool object_has_zero_speed = object_speed < config_.obstacle_zero_speed_threshold_in_ms;

    if (object_predictions.size() < 2)
    {
      throw std::invalid_argument("Object on ther road doesn't have enough predicted states! Please check motion_computation is correctly applying predicted states");
    }
    const double object_prediction_step_duration = (rclcpp::Time(object_predictions.at(1).header.stamp) - rclcpp::Time(object_predictions.front().header.stamp)).seconds();
    const double object_prediction_total_duration = get_trajectory_duration(object_predictions);

    if (object_prediction_step_duration < 0.0)
    {
      throw std::invalid_argument("Predicted states of the object is malformed. Detected trajectory going backwards in time!");
    }

    // In order to optimize the for loops for comparing two trajectories, following logic skips every iteration_stride-th points of the object_predictions.
    // Since skipping number of points from the object_predictions may result in ignoring potential collisions, its value is dependent on two
    // trajectories' speeds and intervehicle_collision_distance_in_m radius.
    // Therefore, the derivation first calculates the max time, t, that both actors can move while still being in collision radius:
    // sqrt( (v1 * t / 2)^2 + (v2 * t / 2)^2 ) = collision_radius. Here v1 and v2 are assumed to be perpendicular to each other and
    // intersecting at t/2 to get max possible collision_radius. Solving for t gives following:
    double iteration_stride_max_time_s = 2 * config_.intervehicle_collision_distance_in_m / sqrt(pow(object_speed, 2) + pow(ego_max_speed, 2));
    int iteration_stride = std::max(1, static_cast<int>(iteration_stride_max_time_s / object_prediction_step_duration));

    RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Determined iteration_stride: " << iteration_stride
      << ", with object_speed: " << object_speed
      << ", with ego_max_speed: " << ego_max_speed
      << ", with object_prediction_step_duration: " << object_prediction_step_duration
      << ", iteration_stride_max_time_s: " << iteration_stride_max_time_s);

    for (size_t j = 0; j < object_predictions.size(); j += iteration_stride)
    {
      const lanelet::BasicPoint2d point(object_predictions.at(j).predicted_position.position.x,
                                        object_predictions.at(j).predicted_position.position.y);
      for (const auto& llt : route_llt_polygons_)
      {
        if (boost::geometry::within(point, llt.polygon2d()))
        {
          on_route = true;
          on_route_idx = j;
          break;
        }
      }
      if (on_route || object_has_zero_speed)
        break;
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "[CPU] on_route=" << on_route
      << " on_route_idx=" << on_route_idx
      << " speed=" << object_speed
      << " stride=" << iteration_stride);

    if (!on_route)
    {
      RCLCPP_DEBUG(rclcpp::get_logger("yield_plugin"), "[CPU] Object not on route — skipping");
      return std::nullopt;
    }

    double smallest_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < ego_trajectory.trajectory_points.size() - 1; ++i)
    {
      auto ego_seg_start = ego_trajectory.trajectory_points.at(i);
      auto ego_seg_end = ego_trajectory.trajectory_points.at(i + 1);
      double previous_distance = std::numeric_limits<double>::infinity();
      for (size_t j = on_route_idx; j < object_predictions.size() - 1; j += iteration_stride)
      {
        auto object_seg_start = object_predictions.at(j);
        auto object_seg_end = object_predictions.at(j + 1);
        double ego_seg_start_time = rclcpp::Time(ego_seg_start.target_time).seconds();
        double ego_seg_end_time = rclcpp::Time(ego_seg_end.target_time).seconds();
        double object_seg_start_time = rclcpp::Time(object_seg_start.header.stamp).seconds();
        double object_seg_end_time = rclcpp::Time(object_seg_end.header.stamp).seconds();

        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ego_seg_start.target_time: " << std::to_string(ego_seg_start_time) << ", ego_seg_end.target_time: " << std::to_string(ego_seg_end_time));
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "object_seg_start.target_time: " << std::to_string(object_seg_start_time) << ", object_seg_end.target_time: " << std::to_string(object_seg_end_time));
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ego_seg_start.x: " << ego_seg_start.x << ", ego_seg_start.y: " << ego_seg_start.y);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "ego_seg_end.x: " << ego_seg_end.x << ", ego_seg_end.y: " << ego_seg_end.y);

        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "object_seg_start.x: " << object_seg_start.predicted_position.position.x << ", object_seg_start.y: " << object_seg_start.predicted_position.position.y);
        RCLCPP_DEBUG_STREAM(nh_->get_logger(), "object_seg_end.x: " << object_seg_end.predicted_position.position.x << ", object_seg_end.y: " << object_seg_end.predicted_position.position.y);

        // Linearly interpolate positions at a common timestamp for both trajectories
        double interp_ratio = (object_seg_start_time - ego_seg_start_time) / (ego_seg_end_time - ego_seg_start_time);
        // if negative extrapolation, skip because car wouldn't go backwards
        if (interp_ratio < 0)
        {
          RCLCPP_DEBUG_STREAM(nh_->get_logger(),
            "Negative extrapolation, skipping this pair of points. object_seg_start_time: "
            << std::to_string(object_seg_start_time) << ", ego_seg_start_time: "
            << std::to_string(ego_seg_start_time));
          continue;
        }
        double ego_interp_x = ego_seg_start.x + interp_ratio * (ego_seg_end.x - ego_seg_start.x);
        double ego_interp_y = ego_seg_start.y + interp_ratio * (ego_seg_end.y - ego_seg_start.y);
        double object_x = object_seg_start.predicted_position.position.x;
        double object_y = object_seg_start.predicted_position.position.y;

        // Calculate the distance between the two interpolated points
        const auto distance{std::hypot(ego_interp_x - object_x, ego_interp_y - object_y)};

        smallest_dist = std::min(distance, smallest_dist);

        // Following "if logic" assumes the object_predictions is a simple cv model, aka, object_predictions point is a straight line over time.
        // And current ego_trajectory point is fixed in this iteration.
        // Then once the distance between the two start to increase over object_predictions iteration,
        // the distance will always increase and it's unnecessary to continue the logic to find the smallest_dist
        if (previous_distance < distance)
        {
          RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Stopping search here because the distance between predictions started to increase");
          break;
        }
        previous_distance = distance;

        if (i == 0 && j == 0 && distance > config_.collision_check_radius_in_m)
        {
          RCLCPP_DEBUG(nh_->get_logger(), "Too far away" );
          return std::nullopt;
        }

        if (distance > collision_radius)
        {
          // continue searching for collision
          continue;
        }

        GetCollisionResult collision_result;
        collision_result.ego_point = lanelet::BasicPoint2d(ego_interp_x, ego_interp_y);
        collision_result.object_point = lanelet::BasicPoint2d(object_x, object_y);
        collision_result.collision_time = rclcpp::Time(object_seg_start.header.stamp);
        return collision_result;
      }
    }
    RCLCPP_DEBUG_STREAM(
      rclcpp::get_logger("yield_plugin"),
      "Was not able to find collision: smallest_dist: " << smallest_dist);

    // No collision detected
    return std::nullopt;
  }

  //TODO: Revisit this logic. Further investigation required for this logic since it currently seems like it will never return true
  bool YieldPlugin::is_object_behind_vehicle(uint32_t object_id, const rclcpp::Time& collision_time, double vehicle_downtrack, double object_downtrack)
  {
    const auto previous_clearance_count = consecutive_clearance_count_for_obstacles_[object_id];
    // if the object's location is half a length of the vehicle past its rear-axle, it is considered behind
    // half a length of the vehicle to conservatively estimate the rear axle to rear bumper length
    if (object_downtrack < vehicle_downtrack - config_.vehicle_length / 2)
    {
      consecutive_clearance_count_for_obstacles_[object_id] = std::min(consecutive_clearance_count_for_obstacles_[object_id] + 1, config_.consecutive_clearance_count_for_passed_obstacles_threshold);
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Detected an object nearby might be behind the vehicle at timestamp: " << std::to_string(collision_time.seconds()) <<
        ", and consecutive_clearance_count_for obstacle: " <<  object_id << ", is: " << consecutive_clearance_count_for_obstacles_[object_id]);
    }
    // confirmed false positive for a collision
    if (consecutive_clearance_count_for_obstacles_[object_id] >= config_.consecutive_clearance_count_for_passed_obstacles_threshold)
    {
      return true;
    }
    // if the clearance counter didn't increase by this point, true collision was detected
    // therefore reset the consecutive clearance counter as it is no longer consecutive
    if (consecutive_clearance_count_for_obstacles_[object_id] == previous_clearance_count)
    {
      consecutive_clearance_count_for_obstacles_[object_id] = 0;
    }

    return false;
  }

  std::optional<rclcpp::Time> YieldPlugin::get_collision_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const carma_perception_msgs::msg::ExternalObject& curr_obstacle, double original_tp_max_speed)
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

    const auto collision_result = get_collision(original_tp, new_list, config_.intervehicle_collision_distance_in_m, original_tp_max_speed);

    if (!collision_result)
    {
      // reset the consecutive clearance counter because no collision was detected at this iteration
      consecutive_clearance_count_for_obstacles_[curr_obstacle.id] = 0;
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "[CPU] obj=" << curr_obstacle.id << " no collision detected");
      return std::nullopt;
    }

    // if within collision radius, it is not a collision if obstacle is behind the vehicle despite being in collision radius
    const double vehicle_downtrack = wm_->routeTrackPos(collision_result.value().ego_point).downtrack;
    const double object_downtrack = wm_->routeTrackPos(collision_result.value().object_point).downtrack;

    if (is_object_behind_vehicle(curr_obstacle.id, collision_result.value().collision_time, vehicle_downtrack, object_downtrack))
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Confirmed that the object: " << curr_obstacle.id << " is behind the vehicle at timestamp " << std::to_string(collision_result.value().collision_time.seconds()));
      return std::nullopt;
    }

    const auto distance{std::hypot(
      collision_result.value().ego_point.x() - collision_result.value().object_point.x(),
      collision_result.value().ego_point.y() - collision_result.value().object_point.y()
    )}; //for debug

    RCLCPP_WARN_STREAM(nh_->get_logger(), "Collision detected for object: " << curr_obstacle.id << ", at timestamp " << std::to_string(collision_result.value().collision_time.seconds()) <<
      ", x: " << collision_result.value().ego_point.x() << ", y: " << collision_result.value().ego_point.y() <<
      ", within actual downtrack distance: " << object_downtrack - vehicle_downtrack <<
      ", and collision distance: " << distance);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "[CPU] obj=" << curr_obstacle.id << " collision at t=" << collision_result.value().collision_time.seconds());

    return collision_result.value().collision_time;
  }

  static lanelet::BasicPoint2d interp_trajectory_pt_at_time(
    double query_time,
    const std::vector<CudaPoint>& ego_points,
    int num_ego_points,
    const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan)
  {
    lanelet::BasicPoint2d result(ego_points[0].x, ego_points[0].y);
    for (int i = 0; i < num_ego_points - 1; ++i) {
      const double seg_start_time = rclcpp::Time(trajectory_plan.trajectory_points[i].target_time).seconds();
      const double seg_end_time = rclcpp::Time(trajectory_plan.trajectory_points[i + 1].target_time).seconds();
      if (seg_start_time <= query_time && query_time <= seg_end_time) {
        const double interp_ratio = (seg_end_time > seg_start_time) ? (query_time - seg_start_time) / (seg_end_time - seg_start_time) : 0.0;
        result.x() = trajectory_plan.trajectory_points[i].x + interp_ratio * (trajectory_plan.trajectory_points[i+1].x - trajectory_plan.trajectory_points[i].x);
        result.y() = trajectory_plan.trajectory_points[i].y + interp_ratio * (trajectory_plan.trajectory_points[i+1].y - trajectory_plan.trajectory_points[i].y);
        break;
      }
    }
    return result;
  }

  static lanelet::BasicPoint2d interp_predicted_pt_at_time(
    double query_time,
    const std::vector<carma_perception_msgs::msg::PredictedState>& predictions,
    int start_index)
  {
    lanelet::BasicPoint2d result(
      predictions.front().predicted_position.position.x,
      predictions.front().predicted_position.position.y);
    for (int j = start_index; j < static_cast<int>(predictions.size()) - 1; ++j) {
      const double seg_start_time = rclcpp::Time(predictions[j].header.stamp).seconds();
      const double seg_end_time = rclcpp::Time(predictions[j + 1].header.stamp).seconds();
      if (seg_start_time <= query_time && query_time <= seg_end_time) {
        const double interp_ratio = (seg_end_time > seg_start_time) ? (query_time - seg_start_time) / (seg_end_time - seg_start_time) : 0.0;
        result.x() = predictions[j].predicted_position.position.x +
                     interp_ratio * (predictions[j+1].predicted_position.position.x - predictions[j].predicted_position.position.x);
        result.y() = predictions[j].predicted_position.position.y +
                     interp_ratio * (predictions[j+1].predicted_position.position.y - predictions[j].predicted_position.position.y);
        break;
      }
    }
    return result;
  }

  std::pair<bool, int> YieldPlugin::find_on_route_in_predictions(
    const std::vector<carma_perception_msgs::msg::PredictedState>& predictions,
    int stride, bool object_has_zero_speed) const
  {
    for (size_t j = 0; j < predictions.size(); j += stride) {
      const lanelet::BasicPoint2d point(predictions[j].predicted_position.position.x,
                                        predictions[j].predicted_position.position.y);
      for (const auto& llt : route_llt_polygons_) {
        if (boost::geometry::within(point, llt.polygon2d())) {
          return {true, static_cast<int>(j)};
        }
      }
      if (object_has_zero_speed) break;
    }
    return {false, 0};
  }

  std::unordered_map<uint32_t, rclcpp::Time> YieldPlugin::get_collision_times_concurrently(
    const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects,
    double original_tp_max_speed)
  {
    if (!cuda_is_available())
      return get_collision_times_concurrently_cpu(original_tp, external_objects, original_tp_max_speed);
    return get_collision_times_concurrently_cuda(original_tp, external_objects, original_tp_max_speed);
  }

  std::unordered_map<uint32_t, rclcpp::Time> YieldPlugin::get_collision_times_concurrently_cpu(
    const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects,
    double original_tp_max_speed)
  {
    std::unordered_map<uint32_t, std::future<std::optional<rclcpp::Time>>> futures;
    std::unordered_map<uint32_t, rclcpp::Time> collision_times;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[CPU] Launching " << external_objects.size() << " async get_collision_time tasks");
    std::vector<std::thread> threads;
    threads.reserve(external_objects.size());
    for (const auto& object : external_objects) {
      std::packaged_task<std::optional<rclcpp::Time>()> task(
        [this, &original_tp, &object, &original_tp_max_speed] {
          return get_collision_time(original_tp, object, original_tp_max_speed);
        });
      futures[object.id] = task.get_future();
      threads.emplace_back(std::move(task));
    }
    for (auto& t : threads) t.join();
    for (const auto& object : external_objects) {
      if (const auto collision_time{futures.at(object.id).get()}) {
        collision_times[object.id] = collision_time.value();
      }
    }
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[CPU] Done — " << collision_times.size() << " collision(s) confirmed");
    return collision_times;
  }

  std::unordered_map<uint32_t, rclcpp::Time> YieldPlugin::get_collision_times_concurrently_cuda(
    const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects,
    double original_tp_max_speed)
  {
    std::unordered_map<uint32_t, rclcpp::Time> collision_times;

    if (original_tp.trajectory_points.size() < 2) return collision_times;

    const double plan_start_time = get_trajectory_start_time(original_tp);

    // Timestamps as absolute doubles; reference used to normalise into float32.
    const double ref_time = plan_start_time;

    // Build ego SoA (structure of array) with normalised timestamps.
    const auto num_ego_points = static_cast<int>(original_tp.trajectory_points.size());
    std::vector<CudaPoint> ego_pts;
    ego_pts.reserve(num_ego_points);
    for (const auto& tp : original_tp.trajectory_points) {
      ego_pts.push_back({
        static_cast<float>(tp.x),
        static_cast<float>(tp.y),
        static_cast<float>(rclcpp::Time(tp.target_time).seconds() - ref_time)
      });
    }

    // Per-object data accumulated for the CUDA batch.
    struct ActiveObject {
      uint32_t id;
      // Prediction list with current position prepended (same as get_collision_time builds).
      std::vector<carma_perception_msgs::msg::PredictedState> predictions;
      int on_route_idx;  // first prediction index known to be on the route
    };

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[GPU] Processing " << external_objects.size() << " external objects");

    std::vector<ActiveObject>  active;
    std::vector<CudaPoint>     obs_flat;
    std::vector<int>           obs_offsets;
    std::vector<int>           obs_sizes;

    for (const auto& obj : external_objects) {
      // Skip objects whose entire prediction horizon is before the plan start.
      if (rclcpp::Time(obj.predictions.back().header.stamp).seconds() <= plan_start_time) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
          "[GPU] obj=" << obj.id << " skipped — predictions expired before plan start");
        continue;
      }

      // Build prediction list: prepend current position (mirrors get_collision_time).
      std::vector<carma_perception_msgs::msg::PredictedState> pred_list;
      pred_list.reserve(obj.predictions.size() + 1);
      {
        carma_perception_msgs::msg::PredictedState curr;
        curr.header.stamp = obj.header.stamp;
        curr.predicted_position.position.x = obj.pose.pose.position.x;
        curr.predicted_position.position.y = obj.pose.pose.position.y;
        curr.predicted_velocity.linear.x   = obj.velocity.twist.linear.x;
        curr.predicted_velocity.linear.y   = obj.velocity.twist.linear.y;
        pred_list.push_back(curr);
      }
      pred_list.insert(pred_list.end(), obj.predictions.cbegin(), obj.predictions.cend());

      if (pred_list.size() < 2) continue;

      const double object_prediction_step_duration =
        (rclcpp::Time(pred_list.at(1).header.stamp) - rclcpp::Time(pred_list.front().header.stamp)).seconds();
      if (object_prediction_step_duration < 0.0) continue;

      // Quick spatial pre-filter: if the object starts beyond collision_check_radius_in_m
      // it cannot collide with the ego at the start of the trajectory.
      {
        const double ego_to_object_dx = ego_pts[0].x - obj.pose.pose.position.x;
        const double ego_to_object_dy = ego_pts[0].y - obj.pose.pose.position.y;
        const double ego_to_object_dist = std::hypot(ego_to_object_dx, ego_to_object_dy);
        if (ego_to_object_dist > config_.collision_check_radius_in_m) {
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
            "[GPU] obj=" << obj.id << " skipped — dist_from_ego=" << ego_to_object_dist
            << " > radius=" << config_.collision_check_radius_in_m);
          consecutive_clearance_count_for_obstacles_[obj.id] = 0;
          continue;
        }
      }

      // On-route check — stride logic as in get_collision, but uses pre-computed
      // per-lanelet bounding boxes instead of getLaneletsFromPoint (O(1) vs spatial query).
      const double object_speed = std::hypot(
        pred_list.front().predicted_velocity.linear.x,
        pred_list.front().predicted_velocity.linear.y);
      const bool object_has_zero_speed = object_speed < config_.obstacle_zero_speed_threshold_in_ms;

      const double stride_max_t = 2.0 * config_.intervehicle_collision_distance_in_m /
        std::sqrt(std::pow(object_speed, 2) + std::pow(original_tp_max_speed, 2));
      const int iteration_stride = std::max(1, static_cast<int>(stride_max_t / object_prediction_step_duration));

      const auto [on_route, on_route_idx] = find_on_route_in_predictions(pred_list, iteration_stride, object_has_zero_speed);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
        "[GPU] obj=" << obj.id << " on_route=" << on_route
        << " on_route_idx=" << on_route_idx
        << " speed=" << object_speed
        << " stride=" << iteration_stride);

      if (!on_route) {
        consecutive_clearance_count_for_obstacles_[obj.id] = 0;
        continue;
      }

      // Pack the on-route portion of the prediction into the flat obstacle buffer.
      obs_offsets.push_back(static_cast<int>(obs_flat.size()));
      int count = 0;
      for (int j = on_route_idx; j < static_cast<int>(pred_list.size()); ++j) {
        obs_flat.push_back({
          static_cast<float>(pred_list[j].predicted_position.position.x),
          static_cast<float>(pred_list[j].predicted_position.position.y),
          static_cast<float>(rclcpp::Time(pred_list[j].header.stamp).seconds() - ref_time)
        });
        ++count;
      }
      obs_sizes.push_back(count);
      active.push_back({obj.id, std::move(pred_list), on_route_idx});
    }

    if (active.empty()) return collision_times;

    // -----------------------------------------------------------------------
    // GPU: exact, continuous-time segment-pair collision detection.
    // -----------------------------------------------------------------------
    auto _t0_cuda = std::chrono::steady_clock::now();
    try {
    const auto cuda_results = cuda_check_all_collisions(
      ego_pts, obs_flat, obs_offsets, obs_sizes,
      static_cast<float>(config_.intervehicle_collision_distance_in_m));
    const double _cuda_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_cuda).count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[timing] cuda_check_all_collisions: " << _cuda_ms << " ms");

    // -----------------------------------------------------------------------
    // Post-process: recover collision positions and run behind-vehicle check.
    // -----------------------------------------------------------------------
    for (size_t k = 0; k < active.size(); ++k) {
      const auto& object_result = cuda_results[k];

      if (!object_result.has_collision) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
          "[GPU] obj=" << active[k].id << " no collision detected");
        consecutive_clearance_count_for_obstacles_[active[k].id] = 0;
        continue;
      }

      const double collision_time_abs = static_cast<double>(object_result.collision_t_norm) + ref_time;
      const rclcpp::Time collision_time(static_cast<int64_t>(collision_time_abs * 1e9));

      const lanelet::BasicPoint2d ego_collision_point = interp_trajectory_pt_at_time(collision_time_abs, ego_pts, num_ego_points, original_tp);
      const lanelet::BasicPoint2d object_collision_point = interp_predicted_pt_at_time(collision_time_abs, active[k].predictions, active[k].on_route_idx);

      const double vehicle_downtrack = wm_->routeTrackPos(ego_collision_point).downtrack;
      const double object_downtrack  = wm_->routeTrackPos(object_collision_point).downtrack;

      if (is_object_behind_vehicle(active[k].id, collision_time,
                                   vehicle_downtrack, object_downtrack)) {
        RCLCPP_INFO_STREAM(nh_->get_logger(),
          "Confirmed that the object: " << active[k].id
          << " is behind the vehicle at timestamp "
          << std::to_string(collision_time.seconds()));
        continue;
      }

      RCLCPP_WARN_STREAM(nh_->get_logger(),
        "Collision detected for object: " << active[k].id
        << ", at timestamp " << std::to_string(collision_time.seconds())
        << ", x: " << ego_collision_point.x() << ", y: " << ego_collision_point.y()
        << ", within actual downtrack distance: "
        << object_downtrack - vehicle_downtrack);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
        "[GPU] obj=" << active[k].id
        << " collision at t=" << collision_time_abs
        << " ego=(" << ego_collision_point.x() << "," << ego_collision_point.y() << ")"
        << " obs=(" << object_collision_point.x() << "," << object_collision_point.y() << ")"
        << " downtrack_gap=" << (object_downtrack - vehicle_downtrack));
      collision_times[active[k].id] = collision_time;
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[GPU] Done — " << collision_times.size() << " collision(s) confirmed");

    } catch (const std::runtime_error& e) {
      std::string error_msg(e.what());
      // Detect CUDA-specific errors
      bool is_cuda_error = (error_msg.find("CUDA") != std::string::npos ||
                            error_msg.find("cuda") != std::string::npos ||
                            error_msg.find("driver version") != std::string::npos ||
                            error_msg.find("runtime version") != std::string::npos ||
                            error_msg.find("GPU") != std::string::npos);

      if (is_cuda_error) {
        RCLCPP_WARN_STREAM_ONCE(rclcpp::get_logger("yield_plugin"),
          "[GPU] CUDA unavailable (" << e.what() << "), please make sure GPU is accessible for this node or container. Using CPU fallback");
      } else {
        RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger("yield_plugin"),
          "[GPU] Unexpected error during GPU collision detection: " << e.what() << ", using CPU fallback");
      }
      return get_collision_times_concurrently_cpu(original_tp, external_objects, original_tp_max_speed);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM_ONCE(rclcpp::get_logger("yield_plugin"),
        "[GPU] Unexpected exception during GPU collision detection: " << typeid(e).name() << " - " << e.what() << ", using CPU fallback");
      return get_collision_times_concurrently_cpu(original_tp, external_objects, original_tp_max_speed);
    }
    return collision_times;
  }

  std::optional<std::pair<carma_perception_msgs::msg::ExternalObject, double>> YieldPlugin::get_earliest_collision_object_and_time(const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "ExternalObjects size: " << external_objects.size());

    if (!wm_->getRoute() || route_llt_polygons_.empty())
    {
      RCLCPP_WARN(nh_->get_logger(), "Yield plugin was not able to analyze collision since route is not available! Please check if route is set");
      return std::nullopt;
    }

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"External Object List (external_objects) size: " << external_objects.size());
    const double original_max_speed = max_trajectory_speed(original_tp.trajectory_points, get_trajectory_end_time(original_tp));
    auto _t0_conc = std::chrono::steady_clock::now();
    std::unordered_map<uint32_t, rclcpp::Time> collision_times = get_collision_times_concurrently(original_tp,external_objects, original_max_speed);
    const double _conc_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_conc).count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[timing] get_collision_times_concurrently: " << _conc_ms << " ms");

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
    auto _t0_ect = std::chrono::steady_clock::now();
    const auto earliest_collision_obj_pair = get_earliest_collision_object_and_time(original_tp, external_objects);
    const double _ect_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_ect).count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[timing] get_earliest_collision_object_and_time: " << _ect_ms << " ms");

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
    auto _rtp_t0_upd = std::chrono::steady_clock::now();
    const double vehicle_downtrack = wm_->routeTrackPos(vehicle_point).downtrack;

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"vehicle_downtrack: " << vehicle_downtrack);

    RCLCPP_WARN_STREAM(nh_->get_logger(),"Collision Detected!");

    const lanelet::BasicPoint2d object_point(earliest_collision_obj.pose.pose.position.x, earliest_collision_obj.pose.pose.position.y);
    const double object_downtrack = wm_->routeTrackPos(object_point).downtrack;

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object_downtrack: " << object_downtrack);
    const double _rtp_ms_upd = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - _rtp_t0_upd).count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[update_traj] routeTrackPos x2: " << _rtp_ms_upd << " ms");

    const double object_downtrack_lead = std::max(0.0, object_downtrack - vehicle_downtrack);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object_downtrack_lead: " << object_downtrack_lead);

    // The vehicle's goal velocity of the yielding behavior is to match the velocity of the object along the trajectory.
    double goal_velocity = get_predicted_velocity_at_time(earliest_collision_obj.velocity.twist, original_tp, earliest_collision_time_in_seconds);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"object's speed along trajectory at collision: " << goal_velocity);

    // roadway object position
    const double gap_time_until_min_gap_distance = std::max(0.0, object_downtrack_lead - config_.minimum_safety_gap_in_meters)/initial_velocity;

    if (goal_velocity <= config_.obstacle_zero_speed_threshold_in_ms){
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
      auto _t0_gap = std::chrono::steady_clock::now();
      double externally_commanded_safety_gap = check_traj_for_digital_min_gap(original_tp);
      const double _gap_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_gap).count();
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
        "[timing] check_traj_for_digital_min_gap: " << _gap_ms << " ms");
      RCLCPP_DEBUG_STREAM(nh_->get_logger(),"externally_commanded_safety_gap: " << externally_commanded_safety_gap);
      // if a digital gap is available, it is replaced as safety gap
      safety_gap = std::max(safety_gap, externally_commanded_safety_gap);
    }

    const double goal_pos = std::max(0.0, object_downtrack_lead - safety_gap - config_.vehicle_length);
    const double initial_pos = 0.0; //relative initial position (first trajectory point)
    const double original_max_speed = max_trajectory_speed(original_tp.trajectory_points, earliest_collision_time_in_seconds);
    const double delta_v_max = fabs(goal_velocity - original_max_speed);
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"delta_v_max: " << delta_v_max << ", safety_gap: " << safety_gap);

    const double time_required_for_comfortable_decel_in_s = config_.acceleration_adjustment_factor * 2 * goal_pos / delta_v_max;
    const double min_time_required_for_comfortable_decel_in_s = delta_v_max / config_.yield_max_deceleration_in_ms2;

    // planning time for object avoidance
    double planning_time_in_s = std::max({config_.min_obj_avoidance_plan_time_in_s, time_required_for_comfortable_decel_in_s, min_time_required_for_comfortable_decel_in_s});
    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"time_required_for_comfortable_decel_in_s: " << time_required_for_comfortable_decel_in_s << ", min_time_required_for_comfortable_decel_in_s: " << min_time_required_for_comfortable_decel_in_s);

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"Object avoidance planning time: " << planning_time_in_s);

    auto _t0_jmt = std::chrono::steady_clock::now();
    auto jmt_trajectory = generate_JMT_trajectory(original_tp,
      initial_pos, goal_pos, initial_velocity, goal_velocity,
      planning_time_in_s, original_max_speed);
    const double _jmt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - _t0_jmt).count();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "[timing] generate_JMT_trajectory: " << _jmt_ms << " ms");

    // If expected to stop to prevent collision, we should save this trajectory to commit to it
    if (!last_traj_plan_committed_to_stopping_.has_value() &&
        goal_velocity < EPSILON &&
        earliest_collision_time_in_seconds - nh_->now().seconds()
        <= config_.time_horizon_until_collision_to_commit_to_stop_in_s)
    {
      last_traj_plan_committed_to_stopping_ = jmt_trajectory;
    }
    return jmt_trajectory;
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
    if (!wm_->getRoute()) return desired_gap;

    const lanelet::BasicPoint2d traj_start(original_tp.trajectory_points.front().x,
                                           original_tp.trajectory_points.front().y);
    const lanelet::BasicPoint2d traj_end(original_tp.trajectory_points.back().x,
                                         original_tp.trajectory_points.back().y);

    // Fetch up to 4 candidates per endpoint — a boundary point can lie on
    // overlapping lanelets and a single result may be the wrong one.
    auto start_llts = wm_->getLaneletsFromPoint(traj_start, 4);
    auto end_llts   = wm_->getLaneletsFromPoint(traj_end,   4);

    if (start_llts.empty() || end_llts.empty())
    {
      // Trajectory generation may place a point off-road in rare edge cases
      // (see https://github.com/usdot-fhwa-stol/carma-platform/issues/2503)
      RCLCPP_WARN_STREAM(nh_->get_logger(), "check_traj_for_digital_min_gap: trajectory endpoint "
        "not on a lanelet, skipping digital gap check.");
      return desired_gap;
    }

    std::unordered_set<lanelet::Id> start_ids;
    std::unordered_set<lanelet::Id> end_ids;
    for (const auto& llt : start_llts) start_ids.insert(llt.id());
    for (const auto& llt : end_llts)   end_ids.insert(llt.id());

    // One pass: first index matching any start candidate, last index matching any end candidate.
    // Taking "last" for end covers all overlapping lanelets at the trajectory's back boundary.
    std::optional<size_t> start_pos;
    std::optional<size_t> end_pos;
    size_t pos = 0;
    const auto& path = wm_->getRoute()->shortestPath();
    for (const auto& llt : path)
    {
      if (!start_pos.has_value() && start_ids.count(llt.id())) start_pos = pos;
      if (end_ids.count(llt.id()))                 end_pos   = pos;
      ++pos;
    }

    if (!start_pos || !end_pos || *start_pos > *end_pos)
    {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "check_traj_for_digital_min_gap: trajectory endpoints "
        "not found on route shortest path, skipping digital gap check.");
      return desired_gap;
    }

    pos = 0;
    for (const auto& llt : path)
    {
      if (pos > *end_pos) break;
      if (pos >= *start_pos)
      {
        auto digital_min_gap = llt.regulatoryElementsAs<lanelet::DigitalMinimumGap>();
        if (!digital_min_gap.empty())
        {
          double digital_gap = digital_min_gap[0]->getMinimumGap();
          RCLCPP_DEBUG_STREAM(nh_->get_logger(), "Digital Gap found with value: " << digital_gap);
          desired_gap = std::max(desired_gap, digital_gap);
        }
      }
      ++pos;
    }
    return desired_gap;
  }

  void YieldPlugin::set_georeference_string(const std::string& georeference)
  {
    if (georeference_ != georeference)
    {
      georeference_ = georeference;
      map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(georeference.c_str());  // Build projector from proj string
    }
  }

  void YieldPlugin::set_external_objects(const std::vector<carma_perception_msgs::msg::ExternalObject>& object_list)
  {
    external_objects_ = object_list;
  }

  void YieldPlugin::update_route_llt_cache()
  {
    if (!wm_->getRoute())
    {
      RCLCPP_WARN(nh_->get_logger(), "update_route_llt_cache called but route is not available");
      return;
    }
    route_llt_ids_.clear();
    route_llt_polygons_.clear();
    for (const auto& llt : wm_->getRoute()->shortestPath())
    {
      route_llt_ids_.insert(llt.id());
      route_llt_polygons_.push_back(llt);
    }
  }

}  // namespace yield_plugin
