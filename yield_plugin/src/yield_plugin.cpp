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
#include <fmt/format.h>
#include <yield_plugin/yield_plugin.hpp>
#include <carma_v2x_msgs/msg/location_ecef.hpp>
#include <carma_v2x_msgs/msg/trajectory.hpp>
#include <carma_v2x_msgs/msg/plan_type.hpp>
#include <basic_autonomy/smoothing/filters.hpp>
#include <future>

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
        yield_trajectory = update_traj_for_object(original_trajectory, external_objects_, initial_velocity); // Compute the trajectory
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
    }
    catch(const std::runtime_error& e) {
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Yield Plugin failed to plan trajectory due to known negative time issue: " << e.what());
      RCLCPP_WARN_STREAM(nh_->get_logger(), "Returning the original trajectory, and retrying at the next call.");
      resp->trajectory_plan = original_trajectory;
    }

    rclcpp::Time end_time = system_clock.now();  // Planning complete

    auto duration = end_time - start_time;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "ExecutionTime: " << std::to_string(duration.seconds()));
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
    const auto smallest_time_step = get_smallest_time_step_of_traj(original_tp);
    int new_traj_idx = 1;
    int original_traj_idx = 1;
    while (new_traj_accumulated_downtrack < goal_pos - EPSILON && original_traj_idx < original_traj_relative_downtracks.size())
    {
      const double target_time = new_traj_idx * smallest_time_step;
      const double downtrack_at_target_time = polynomial_calc(polynomial_coefficients, target_time);
      double velocity_at_target_time = polynomial_calc_d(polynomial_coefficients, target_time);

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "Calculated speed velocity_at_target_time: " << velocity_at_target_time
        << ", downtrack_at_target_time: "<< downtrack_at_target_time << ", target_time: " << target_time);

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
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "Picked calculated speed velocity_at_target_time: " << velocity_at_target_time
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

    // if the loop above finished prematurely due to negative speed, fill with 0.0 speeds
    // since the speed crossed 0.0 and algorithm indicates stopping
    std::fill_n(std::back_inserter(calculated_speeds),
                std::size(original_traj_relative_downtracks) - std::size(calculated_speeds),
                0.0);

    // Moving average filter to smoothen the speeds
    std::vector<double> filtered_speeds = basic_autonomy::smoothing::moving_average_filter(calculated_speeds, config_.speed_moving_average_window_size);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "filtered_speeds size: " << filtered_speeds.size());
    for (const auto& speed : filtered_speeds)
    {
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "filtered speed: " << speed);
    }
    // Replace the original trajectory's associated timestamps based on the newly calculated speeds
    double prev_speed = filtered_speeds.at(0);
    last_speed_ = prev_speed;
    last_speed_time_ = nh_->now();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "start speed: " << prev_speed << ", target_time: " << std::to_string(rclcpp::Time(original_tp.trajectory_points[0].target_time).seconds()));

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
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "Zero speed = x: " << jmt_tpp.x << ", y:" << jmt_tpp.y
          << ", t:" << std::to_string(rclcpp::Time(jmt_tpp.target_time).seconds())
          << ", prev_speed: " << prev_speed << ", current_speed: " << current_speed);
      }
      else
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "Non-zero speed = x: " << jmt_tpp.x << ", y:" << jmt_tpp.y
          << ", t:" << std::to_string(rclcpp::Time(jmt_tpp.target_time).seconds())
          << ", prev_speed: " << prev_speed << ", current_speed: " << current_speed);
      }

      jmt_trajectory_points.push_back(jmt_tpp);
      double insta_decel = (current_speed - prev_speed) / (rclcpp::Time(jmt_trajectory_points.at(i).target_time).seconds() - rclcpp::Time(jmt_trajectory_points.at(i - 1).target_time).seconds());
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "insta_decel: " << insta_decel );
      prev_speed = current_speed;
    }

    jmt_trajectory.header = original_tp.header;
    jmt_trajectory.trajectory_id = original_tp.trajectory_id;
    jmt_trajectory.trajectory_points = jmt_trajectory_points;
    jmt_trajectory.initial_longitudinal_velocity = initial_velocity;
    return jmt_trajectory;
  }

  /*
    * @brief Create a spatial grid for the trajectory2 points to speed up collision detection
    *        Spatial grid is created by dividing the space into cells of size 2 * collision_radius
    *        Each cell contains a list of indices of trajectory2 points that fall within that cell
    * @param trajectory2 A predicted state trajectory (often incoming object)
    * @param collision_radius The radius within which to check for collisions
    * @return A map where the key is a grid cell identifier (example: "0,0" or "1,-1")
    *         and the value is a list of indices of trajectory2 points in that cell
  */
 std::unordered_map<std::string, std::vector<size_t>> create_spatial_grid(
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2,
    double collision_radius)
  {
    // Build spatial index for faster geographic lookup
    // For each point in trajectory2, store its position and index
    std::vector<std::tuple<double, double, size_t>> spatial_index;
    for (size_t i = 0; i < trajectory2.size(); ++i) {
        double x = trajectory2[i].predicted_position.position.x;
        double y = trajectory2[i].predicted_position.position.y;
        spatial_index.emplace_back(x, y, i);
    }

    // Sort spatial index (could be replaced with more sophisticated spatial index like R-tree)
    // This simple approach splits the space into a grid of collision_radius-sized cells
    const double cell_size = collision_radius * 2.0;
    std::unordered_map<std::string, std::vector<size_t>> spatial_grid;

    for (size_t i = 0; i < spatial_index.size(); ++i) {
        const auto& [x, y, idx] = spatial_index[i];
        // Convert position to grid cell
        int grid_x = static_cast<int>(x / cell_size);
        int grid_y = static_cast<int>(y / cell_size);

        // Add to current cell and neighboring cells to handle boundary cases
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                std::string cell_key = fmt::format("{},{}", grid_x + dx, grid_y + dy);
                spatial_grid[cell_key].push_back(idx);
            }
        }
    }
    return spatial_grid;
  }

  /*
    * Get collision candidates purely from spatial perspective using spatial grid
    * @param trajectory1 The trajectory of the host vehicle
    * @param trajectory2 The predicted state trajectory (often incoming object)
    * @param traj2_spatial_grid The spatial grid created from trajectory2
    * @param collision_radius The radius within which to check for collisions
    * @return A vector of pairs of indices representing potential collision points
    *         in the form of (trajectory1_index, trajectory2_index)
  */
  std::vector<std::pair<size_t, size_t>> get_spatial_collision_from_spatial_grid(
    const carma_planning_msgs::msg::TrajectoryPlan& trajectory1,
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2,
    const std::unordered_map<std::string, std::vector<size_t>>& traj2_spatial_grid,
    double collision_radius)
  {
    // Check for geographic collisions using the spatial grid
    std::vector<std::pair<size_t, size_t>> candidate_collisions;
    for (size_t i = 0; i < trajectory1.trajectory_points.size(); ++i) {
        double x1 = trajectory1.trajectory_points[i].x;
        double y1 = trajectory1.trajectory_points[i].y;

        // Find grid cell for this point
        int grid_x = static_cast<int>(x1 / (collision_radius * 2.0));
        int grid_y = static_cast<int>(y1 / (collision_radius * 2.0));
        std::string cell_key = fmt::format("{},{}", grid_x, grid_y);

        // Check if cell exists in our grid
        if (traj2_spatial_grid.find(cell_key) != traj2_spatial_grid.end()) {
            // Check all points in this cell
            for (const auto& j : traj2_spatial_grid.at(cell_key)) {
                double x2 = trajectory2[j].predicted_position.position.x;
                double y2 = trajectory2[j].predicted_position.position.y;

                double distance = std::hypot(x1 - x2, y1 - y2);
                if (distance <= collision_radius) {
                    // Geographic collision found - store as candidate
                    candidate_collisions.emplace_back(i, j);
                }
            }
        }
    }
    return candidate_collisions;
  }

  /*
    * @brief Get the best collision point based on temporal checks
             from the candidate collisions filtered from spatial checks
    * @param trajectory1 The trajectory of the host vehicle
    * @param trajectory2 The predicted state trajectory (often incoming object)
    * @param candidate_collisions A vector of pairs of indices representing potential
    *         collision points in the form of (trajectory1_index, trajectory2_index)
    * @param collision_radius The radius within which to check for collisions
    * @return An optional CollisionData containing the best collision candidate
  */
  std::optional<CollisionData> get_temporal_collision(
    const carma_planning_msgs::msg::TrajectoryPlan& trajectory1,
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2,
    const std::vector<std::pair<size_t, size_t>>& candidate_collisions,
    double collision_radius)
  {
    double smallest_time_diff = std::numeric_limits<double>::infinity();
    std::optional<CollisionData> best_collision = std::nullopt;

    const double traj1_speed = 8.99; //TODO fix
    const double traj2_speed = std::hypot(trajectory2.front().predicted_velocity.linear.x,
                                          trajectory2.front().predicted_velocity.linear.y);
    double max_collision_time_diff = 0.0;
    if (traj1_speed > 0.0 || traj2_speed > 0.0) {
        max_collision_time_diff =
          std::max(1.0, collision_radius / std::hypot(traj1_speed, traj2_speed));
    } else {
        // If both objects have zero speed, use a constant time window
        max_collision_time_diff = 1.0; // 1 second
    }

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "Max collision time difference: " << max_collision_time_diff
      << "s, traj1_speed: " << traj1_speed << ", traj2_speed: " << traj2_speed);

    for (const auto& [idx1, idx2] : candidate_collisions) {
        const auto& point1 = trajectory1.trajectory_points[idx1];
        const auto& point2 = trajectory2[idx2];

        double t1 = rclcpp::Time(point1.target_time).seconds();
        double t2 = rclcpp::Time(point2.header.stamp).seconds();
        double time_diff = std::abs(t1 - t2);

        // Skip if time difference is too large
        if (time_diff > max_collision_time_diff) {
            continue;
        }

        // Capture current collision if it has the smallest time difference
        if (time_diff < smallest_time_diff) {
            smallest_time_diff = time_diff;

            CollisionData collision_result;
            collision_result.point1 = lanelet::BasicPoint2d(point1.x, point1.y);
            collision_result.point2 = lanelet::BasicPoint2d(
                point2.predicted_position.position.x,
                point2.predicted_position.position.y
            );

            // Use the earlier timestamp for collision time
            collision_result.collision_time = (t1 < t2) ?
                rclcpp::Time(point1.target_time) :
                rclcpp::Time(point2.header.stamp);

            best_collision = collision_result;
        }
    }
    return best_collision;
  }


  std::optional<CollisionData> YieldPlugin::get_collision(
    const carma_planning_msgs::msg::TrajectoryPlan& trajectory1,
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory2,
    double collision_radius)
  {

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
      "Starting collision detection, trajectory1 size: "
      << trajectory1.trajectory_points.size() << ", trajectory2 size: " << trajectory2.size());

    // Validate input trajectories
    if (trajectory1.trajectory_points.size() < 2 || trajectory2.size() < 2)
    {
        throw std::invalid_argument("Both trajectories must have at least 2 points");
    }

    // Verify the predicted trajectory is on the route
    if (!is_trajectory_on_route(trajectory2)) {
        RCLCPP_DEBUG(rclcpp::get_logger("yield_plugin"),
          "Predicted states are not on the route! Ignoring");
        return std::nullopt;
    }

    // Create a spatial grid for trajectory2 points to speed up collision detection
    // when comparing with trajectory1
    const auto spatial_grid = create_spatial_grid(trajectory2, collision_radius);

    // First pass: Check geographic collisions using spatial grid
    const auto candidate_collisions =
      get_spatial_collision_from_spatial_grid(
          trajectory1, trajectory2, spatial_grid, collision_radius);

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"), "Found " << candidate_collisions.size()
                       << " geographic collision candidates");

    // If no geographic collisions, exit early
    if (candidate_collisions.empty()) {
        auto end_time = nh_->now();
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("yield_plugin"),
          "No geographic collisions found.");
        return std::nullopt;
    }

    // Second pass: Now check temporal constraints for each geographic collision

    // TODO: FIX
    // Calculate maximum time difference to consider for collision
    // This is the time it would take both objects to travel their speeds to cover
    // the collision radius
    // Formula: collision_radius = sqrt((v1*t)^2 + (v2*t)^2), solve for t

    // Extract speeds - we'll use these for temporal calculations later
    return get_temporal_collision(
      trajectory1, trajectory2, candidate_collisions, collision_radius);;
  }

  // Helper method to check if a trajectory is on route
  bool YieldPlugin::is_trajectory_on_route(
    const std::vector<carma_perception_msgs::msg::PredictedState>& trajectory) {

    // Skip every few points for efficiency
    const int stride = std::max(1, static_cast<int>(trajectory.size() / 10));

    for (size_t j = 0; j < trajectory.size(); j += stride) {
        lanelet::BasicPoint2d curr_point;
        curr_point.x() = trajectory.at(j).predicted_position.position.x;
        curr_point.y() = trajectory.at(j).predicted_position.position.y;

        auto corresponding_lanelets = wm_->getLaneletsFromPoint(curr_point, 8);

        for (const auto& llt: corresponding_lanelets) {
            if (route_llt_ids_.find(llt.id()) != route_llt_ids_.end()) {
                return true;
            }
        }
    }

    return false;
  }

  bool YieldPlugin::is_object_behind_vehicle(uint32_t object_id, const rclcpp::Time& collision_time, double vehicle_downtrack, double object_downtrack)
  {
    const auto previous_clearance_count = consecutive_clearance_count_for_obstacles_[object_id];
    // if the object's location is half a length of the vehicle past its rear-axle, it is considered behind
    // half a length of the vehicle to conservatively estimate the rear axle to rear bumper length
    if (object_downtrack < vehicle_downtrack - config_.vehicle_length / 2)
    {
      consecutive_clearance_count_for_obstacles_[object_id] = std::min(consecutive_clearance_count_for_obstacles_[object_id] + 1, config_.consecutive_clearance_count_for_obstacles_threshold);
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Detected an object nearby might be behind the vehicle at timestamp: " << std::to_string(collision_time.seconds()) <<
        ", and consecutive_clearance_count_for obstacle: " <<  object_id << ", is: " << consecutive_clearance_count_for_obstacles_[object_id]);
    }
    // confirmed false positive for a collision
    if (consecutive_clearance_count_for_obstacles_[object_id] >= config_.consecutive_clearance_count_for_obstacles_threshold)
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

    const auto collision_result = get_collision(original_tp, new_list, config_.intervehicle_collision_distance_in_m);

    if (!collision_result)
    {
      // reset the consecutive clearance counter because no collision was detected at this iteration
      consecutive_clearance_count_for_obstacles_[curr_obstacle.id] = 0;
      return std::nullopt;
    }

    // if within collision radius, it is not a collision if obstacle is behind the vehicle despite being in collision radius
    const double vehicle_downtrack = wm_->routeTrackPos(collision_result.value().point1).downtrack;
    const double object_downtrack = wm_->routeTrackPos(collision_result.value().point2).downtrack;

    if (is_object_behind_vehicle(curr_obstacle.id, collision_result.value().collision_time, vehicle_downtrack, object_downtrack))
    {
      RCLCPP_INFO_STREAM(nh_->get_logger(), "Confirmed that the object: " << curr_obstacle.id << " is behind the vehicle at timestamp " << std::to_string(collision_result.value().collision_time.seconds()));
      return std::nullopt;
    }

    const auto distance{std::hypot(
      collision_result.value().point1.x() - collision_result.value().point2.x(),
      collision_result.value().point1.y() - collision_result.value().point2.y()
    )}; //for debug

    RCLCPP_WARN_STREAM(nh_->get_logger(), "Collision detected for object: " << curr_obstacle.id << ", at timestamp " << std::to_string(collision_result.value().collision_time.seconds()) <<
      ", x: " << collision_result.value().point1.x() << ", y: " << collision_result.value().point1.y() <<
      ", within actual downtrack distance: " << object_downtrack - vehicle_downtrack <<
      ", and collision distance: " << distance);

    return collision_result.value().collision_time;
  }

  std::unordered_map<uint32_t, rclcpp::Time> YieldPlugin::get_collision_times_concurrently(
    const carma_planning_msgs::msg::TrajectoryPlan& original_tp,
    const std::vector<carma_perception_msgs::msg::ExternalObject>& external_objects)
  {

    std::unordered_map<uint32_t, std::future<std::optional<rclcpp::Time>>> futures;
    std::unordered_map<uint32_t, rclcpp::Time> collision_times;

    // Launch asynchronous tasks to check for collision times
    for (const auto& object : external_objects) {
      futures[object.id] = std::async(std::launch::async,[this, &original_tp, &object]{
          return get_collision_time(original_tp, object);
        });
    }

    // Collect results from futures and update collision_times
    for (const auto& object : external_objects) {
      if (const auto collision_time{futures.at(object.id).get()}) {
        collision_times[object.id] = collision_time.value();
      }
    }

    return collision_times;
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
      // TODO: Enhancement https://github.com/usdot-fhwa-stol/carma-platform/issues/2316
      route_llt_ids_.insert(llt.id());
    }

    RCLCPP_DEBUG_STREAM(nh_->get_logger(),"External Object List (external_objects) size: " << external_objects.size());
    const double original_max_speed = max_trajectory_speed(original_tp.trajectory_points, get_trajectory_end_time(original_tp));
    std::unordered_map<uint32_t, rclcpp::Time> collision_times = get_collision_times_concurrently(original_tp, external_objects);

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
      double externally_commanded_safety_gap = check_traj_for_digital_min_gap(original_tp);
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

    return generate_JMT_trajectory(original_tp, initial_pos, goal_pos, initial_velocity, goal_velocity, planning_time_in_s, original_max_speed);
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
        // This should technically never happen
        // However, trajectory generation currently may fail due to osm map issue https://github.com/usdot-fhwa-stol/carma-platform/issues/2503
        RCLCPP_WARN_STREAM(nh_->get_logger(),"Trajectory point: x= " << original_tp.trajectory_points.at(i).x << "y="<< original_tp.trajectory_points.at(i).y);
        RCLCPP_WARN_STREAM(nh_->get_logger(),"Trajectory is not on the road, so was unable to get the digital minimum gap. Returning default minimum_safety_gap_in_meters: " << config_.minimum_safety_gap_in_meters);
        return desired_gap;
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

}  // namespace yield_plugin
