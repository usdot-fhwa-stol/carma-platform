/*
 * Copyright (C) 2019-2022 LEIDOS.
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
#include "cooperative_lanechange/cooperative_lanechange_node.hpp"

namespace cooperative_lanechange
{
  namespace std_ph = std::placeholders;

  CooperativeLaneChangePlugin::CooperativeLaneChangePlugin(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::TacticalPlugin(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.trajectory_time_length = declare_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    config_.control_plugin_name = declare_parameter<std::string>("control_plugin_name", config_.control_plugin_name);
    config_.minimum_speed = declare_parameter<double>("minimum_speed", config_.minimum_speed);
    config_.max_accel = declare_parameter<double>("max_accel", config_.max_accel);
    config_.minimum_lookahead_distance = declare_parameter<double>("minimum_lookahead_distance", config_.minimum_lookahead_distance);
    config_.maximum_lookahead_distance = declare_parameter<double>("maximum_lookahead_distance", config_.maximum_lookahead_distance);
    config_.minimum_lookahead_speed = declare_parameter<double>("minimum_lookahead_speed", config_.minimum_lookahead_speed);
    config_.maximum_lookahead_speed = declare_parameter<double>("maximum_lookahead_speed", config_.maximum_lookahead_speed);
    config_.lateral_accel_limit = declare_parameter<double>("lateral_accel_limit", config_.lateral_accel_limit);
    config_.speed_moving_average_window_size = declare_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    config_.curvature_moving_average_window_size = declare_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    config_.curvature_calc_lookahead_count = declare_parameter<int>("curvature_calc_lookahead_count", config_.curvature_calc_lookahead_count);
    config_.downsample_ratio = declare_parameter<int>("downsample_ratio", config_.downsample_ratio);
    config_.destination_range = declare_parameter<double>("destination_range", config_.destination_range);
    config_.lanechange_time_out = declare_parameter<double>("lanechange_time_out", config_.lanechange_time_out);
    config_.min_timestep = declare_parameter<double>("min_timestep", config_.min_timestep);
    config_.starting_downtrack_range = declare_parameter<double>("starting_downtrack_range", config_.starting_downtrack_range);
    config_.starting_fraction = declare_parameter<double>("starting_fraction", config_.starting_fraction);
    config_.mid_fraction = declare_parameter<double>("mid_fraction", config_.mid_fraction);
    config_.min_desired_gap = declare_parameter<double>("min_desired_gap", config_.min_desired_gap);
    config_.desired_time_gap = declare_parameter<double>("desired_time_gap", config_.desired_time_gap);
    config_.turn_downsample_ratio = declare_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    config_.curve_resample_step_size = declare_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    config_.back_distance = declare_parameter<double>("back_distance", config_.back_distance);
    config_.buffer_ending_downtrack = declare_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
  }

  rcl_interfaces::msg::SetParametersResult CooperativeLaneChangePlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<std::string>(
      {{"control_plugin_name", config_.control_plugin_name},
      {"vehicle_id", config_.vehicle_id}}, parameters);

    auto error_2 = update_params<double>(
      {{"trajectory_time_length", config_.trajectory_time_length},
      {"minimum_speed", config_.minimum_speed},
      {"max_accel", config_.max_accel},
      {"minimum_lookahead_distance", config_.minimum_lookahead_distance},
      {"maximum_lookahead_distance", config_.maximum_lookahead_distance},
      {"minimum_lookahead_speed", config_.minimum_lookahead_speed},
      {"maximum_lookahead_speed", config_.maximum_lookahead_speed},
      {"lateral_accel_limit", config_.lateral_accel_limit},
      {"destination_range", config_.destination_range},
      {"lanechange_time_out", config_.lanechange_time_out},
      {"min_timestep", config_.min_timestep},
      {"starting_downtrack_range", config_.starting_downtrack_range},
      {"starting_fraction", config_.starting_fraction},
      {"mid_fraction", config_.mid_fraction},
      {"min_desired_gap", config_.min_desired_gap},
      {"curve_resample_step_size", config_.curve_resample_step_size},
      {"back_distance", config_.back_distance},
      {"buffer_ending_downtrack", config_.buffer_ending_downtrack},
      {"desired_time_gap", config_.desired_time_gap}}, parameters);
      
    auto error_3 = update_params<int>(
      {{"speed_moving_average_window_size", config_.speed_moving_average_window_size},
      {"curvature_moving_average_window_size", config_.curvature_moving_average_window_size},
      {"curvature_calc_lookahead_count", config_.curvature_calc_lookahead_count},
      {"downsample_ratio", config_.downsample_ratio},
      {"turn_downsample_ratio", config_.turn_downsample_ratio}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2 && !error_3;

    return result;
  }

  carma_ros2_utils::CallbackReturn CooperativeLaneChangePlugin::on_configure_plugin()
  {
    RCLCPP_INFO_STREAM(get_logger(), "CooperativeLaneChangePlugin trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("trajectory_time_length", config_.trajectory_time_length);
    get_parameter<std::string>("control_plugin_name", config_.control_plugin_name);
    get_parameter<double>("minimum_speed", config_.minimum_speed);
    get_parameter<double>("max_accel", config_.max_accel);
    get_parameter<double>("minimum_lookahead_distance", config_.minimum_lookahead_distance);
    get_parameter<double>("maximum_lookahead_distance", config_.maximum_lookahead_distance);
    get_parameter<double>("minimum_lookahead_speed", config_.minimum_lookahead_speed);
    get_parameter<double>("maximum_lookahead_speed", config_.maximum_lookahead_speed);
    get_parameter<double>("lateral_accel_limit", config_.lateral_accel_limit);
    get_parameter<int>("speed_moving_average_window_size", config_.speed_moving_average_window_size);
    get_parameter<int>("curvature_moving_average_window_size", config_.curvature_moving_average_window_size);
    get_parameter<int>("curvature_calc_lookahead_count", config_.curvature_calc_lookahead_count);
    get_parameter<int>("downsample_ratio", config_.downsample_ratio);
    get_parameter<double>("destination_range", config_.destination_range);
    get_parameter<double>("lanechange_time_out", config_.lanechange_time_out);
    get_parameter<double>("min_timestep", config_.min_timestep);
    get_parameter<double>("starting_downtrack_range", config_.starting_downtrack_range);
    get_parameter<double>("starting_fraction", config_.starting_fraction);
    get_parameter<double>("mid_fraction", config_.mid_fraction);
    get_parameter<double>("min_desired_gap", config_.min_desired_gap);
    get_parameter<double>("desired_time_gap", config_.desired_time_gap);
    get_parameter<int>("turn_downsample_ratio", config_.turn_downsample_ratio);
    get_parameter<double>("curve_resample_step_size", config_.curve_resample_step_size);
    get_parameter<double>("back_distance", config_.back_distance);
    get_parameter<double>("buffer_ending_downtrack", config_.buffer_ending_downtrack);
    get_parameter<std::string>("vehicle_id", config_.vehicle_id);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&CooperativeLaneChangePlugin::parameter_update_callback, this, std_ph::_1));

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Setup subscribers
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("current_pose", 1,
                                                              std::bind(&CooperativeLaneChangePlugin::pose_cb, this, std_ph::_1));
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 1,
                                                              std::bind(&CooperativeLaneChangePlugin::twist_cb, this, std_ph::_1));
    incoming_mobility_response_sub_  = create_subscription<carma_v2x_msgs::msg::MobilityResponse>("incoming_mobility_response", 1,
                                                              std::bind(&CooperativeLaneChangePlugin::mobilityresponse_cb, this, std_ph::_1));
    georeference_sub_  = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&CooperativeLaneChangePlugin::georeference_cb, this, std_ph::_1));
    bsm_sub_  = create_subscription<carma_v2x_msgs::msg::BSM>("bsm_outbound", 1,
                                                              std::bind(&CooperativeLaneChangePlugin::bsm_cb, this, std_ph::_1));

    // Setup publishers
    outgoing_mobility_request_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityRequest>("outgoing_mobility_request", 5); // Rate from yield plugin
    lanechange_status_pub_ = create_publisher<carma_planning_msgs::msg::LaneChangeStatus>("cooperative_lane_change_status", 10);

    // Initialize World Model
    wm_ = get_world_model();
    
    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void CooperativeLaneChangePlugin::mobilityresponse_cb(const carma_v2x_msgs::msg::MobilityResponse::UniquePtr msg){
    //@SONAR_STOP@
    if (clc_called_ && clc_request_id_ == msg->m_header.plan_id)
    {
      carma_planning_msgs::msg::LaneChangeStatus lc_status_msg;
      if(msg->is_accepted)
      {
        is_lanechange_accepted_ = true;
        lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::ACCEPTANCE_RECEIVED;
        lc_status_msg.description = "Received lane merge acceptance";
      }
      else
      {
        is_lanechange_accepted_ = false;
        lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::REJECTION_RECEIVED;
        lc_status_msg.description = "Received lane merge rejection";
      }
      lanechange_status_pub_->publish(lc_status_msg);
      //@SONAR_START@
    }
    else
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "received mobility response is not related to CLC");
    }
        
  }

  double CooperativeLaneChangePlugin::find_current_gap(long veh2_lanelet_id, double veh2_downtrack, carma_planning_msgs::msg::VehicleState& ego_state) const
  {              
    //find downtrack distance between ego and lag vehicle
    RCLCPP_DEBUG_STREAM(get_logger(), "entered find_current_gap");
    double current_gap = 0.0;
    lanelet::BasicPoint2d ego_pos(ego_state.x_pos_global, ego_state.y_pos_global);
    //double ego_current_downtrack = wm_->routeTrackPos(ego_pos).downtrack;
        
    lanelet::LaneletMapConstPtr const_map(wm_->getMap());
    lanelet::ConstLanelet veh2_lanelet = const_map->laneletLayer.get(veh2_lanelet_id);
    RCLCPP_DEBUG_STREAM(get_logger(), "veh2_lanelet id " << veh2_lanelet.id());

    auto current_lanelets = lanelet::geometry::findNearest(const_map->laneletLayer, ego_pos, 10);       
    if(current_lanelets.size() == 0)
    {
      RCLCPP_WARN_STREAM(get_logger(), "Cannot find any lanelet in map!");
      return true;
    }
    lanelet::ConstLanelet current_lanelet = current_lanelets[0].second;
    RCLCPP_DEBUG_STREAM(get_logger(), "current llt id " << current_lanelet.id());
        
    //Create temporary route between the two vehicles
    lanelet::ConstLanelet start_lanelet = veh2_lanelet;
    lanelet::ConstLanelet end_lanelet = current_lanelet;
        
    auto map_graph = wm_->getMapRoutingGraph();
    RCLCPP_DEBUG_STREAM(get_logger(), "Graph created");

    auto temp_route = map_graph->getRoute(start_lanelet, end_lanelet);
    RCLCPP_DEBUG_STREAM(get_logger(), "Route created");

    //Throw exception if there is no shortest path from veh2 to subject vehicle
    lanelet::routing::LaneletPath shortest_path2;
    if(temp_route)
    {
      shortest_path2 = temp_route.get().shortestPath();
    }
    else{
      RCLCPP_ERROR_STREAM(get_logger(), "No path exists from roadway object to subject");
      throw std::invalid_argument("No path exists from roadway object to subject");
    }
 
    RCLCPP_DEBUG_STREAM(get_logger(), "Shorted path created size: " << shortest_path2.size());
    for (auto llt : shortest_path2)
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "llt id  route: " << llt.id());
    }  

    //To find downtrack- creating temporary route from veh2 to veh1(ego vehicle)
    double veh1_current_downtrack = wm_->routeTrackPos(ego_pos).downtrack;      
    RCLCPP_DEBUG_STREAM(get_logger(), "ego_current_downtrack:" << veh1_current_downtrack);
        
    current_gap = veh1_current_downtrack - veh2_downtrack;
    RCLCPP_DEBUG_STREAM(get_logger(), "Finding current gap");
    RCLCPP_DEBUG_STREAM(get_logger(), "Veh1 current downtrack: " << veh1_current_downtrack << " veh2 downtrack: " << veh2_downtrack);
     
    return current_gap;
  }

  void CooperativeLaneChangePlugin::pose_cb(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    pose_msg_ = *msg;
  }
  
  void CooperativeLaneChangePlugin::twist_cb(const geometry_msgs::msg::TwistStamped::UniquePtr msg)
  {
    current_speed_ = msg->twist.linear.x;
  }
    
  void CooperativeLaneChangePlugin::bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg)
  {
    bsm_core_ = msg->core_data;
  }
  
  void CooperativeLaneChangePlugin::plan_trajectory_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp)
  {
    // Set boolean flag if this is the first time this service has been called
    if (!clc_called_)
    {
      clc_called_ = true;
    }

    //  Only plan the trajectory for the requested LANE_CHANGE maneuver
    std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
    if(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].type != carma_planning_msgs::msg::Maneuver::LANE_CHANGE)
    {
      throw std::invalid_argument ("Cooperative Lane Change Plugin doesn't support this maneuver type");
    }
    maneuver_plan.push_back(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan]);

    // Currently only checking for first lane change maneuver message
    long target_lanelet_id = stol(maneuver_plan[0].lane_change_maneuver.ending_lane_id);
    double target_downtrack = maneuver_plan[0].lane_change_maneuver.end_dist;
    
    // Get subject vehicle info
    lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);
    double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

    RCLCPP_DEBUG_STREAM(get_logger(), "target_lanelet_id: " << target_lanelet_id);
    RCLCPP_DEBUG_STREAM(get_logger(), "target_downtrack: " << target_downtrack);
    RCLCPP_DEBUG_STREAM(get_logger(), "current_downtrack: " << current_downtrack);
    RCLCPP_DEBUG_STREAM(get_logger(), "Starting CLC downtrack: " << maneuver_plan[0].lane_change_maneuver.start_dist);

    if(current_downtrack < maneuver_plan[0].lane_change_maneuver.start_dist - config_.starting_downtrack_range){
      RCLCPP_DEBUG_STREAM(get_logger(), "Lane change trajectory will not be planned. current_downtrack is more than " << config_.starting_downtrack_range << " meters before starting CLC downtrack");
      return;
    }
    auto current_lanelets = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, veh_pos, 10);       
    long current_lanelet_id = current_lanelets[0].second.id();
    if(current_lanelet_id == target_lanelet_id && current_downtrack >= target_downtrack - config_.destination_range){
      carma_planning_msgs::msg::LaneChangeStatus lc_status_msg;
      lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::PLANNING_SUCCESS;
      //No description as per UI documentation
      lanechange_status_pub_->publish(lc_status_msg);
    }

    long veh2_lanelet_id = 0;
    double veh2_downtrack = 0.0, veh2_speed = 0.0;
    bool foundRoadwayObject = false;
    bool negotiate = true;
    std::vector<carma_perception_msgs::msg::RoadwayObstacle> rwol = wm_->getRoadwayObjects();
    //Assuming only one connected vehicle in list 
    for(int i = 0; i < rwol.size(); i++){
      if(rwol[i].connected_vehicle_type.type == carma_perception_msgs::msg::ConnectedVehicleType::NOT_CONNECTED){
        veh2_lanelet_id = rwol[0].lanelet_id;
        veh2_downtrack = rwol[0].down_track; //Returns downtrack
        veh2_speed = rwol[0].object.velocity.twist.linear.x;
        foundRoadwayObject = true;
        break;
      }
    }
    if(foundRoadwayObject){
      RCLCPP_DEBUG_STREAM(get_logger(), "Found Roadway object");
      //get current_gap
      RCLCPP_DEBUG_STREAM(get_logger(), "veh2_lanelet_id: " << veh2_lanelet_id << ", veh2_downtrack: " << veh2_downtrack);
            
      double current_gap = find_current_gap(veh2_lanelet_id, veh2_downtrack, req->vehicle_state);
      RCLCPP_DEBUG_STREAM(get_logger(), "Current gap: " << current_gap);

      //get desired gap - desired time gap (default 3s)* relative velocity
      double relative_velocity = current_speed_ - veh2_speed;
      RCLCPP_DEBUG_STREAM(get_logger(), "Relative velocity: " << relative_velocity);
      double desired_gap = config_.desired_time_gap * relative_velocity;      
      RCLCPP_DEBUG_STREAM(get_logger(), "Desired gap: " << desired_gap);

      if(desired_gap < config_.min_desired_gap){
        desired_gap = config_.min_desired_gap;
      }
      // TODO - this condition needs to be re-enabled after testing
      // if(current_gap > desired_gap){
      //     negotiate = false;  //No need for negotiation
      // }
            
    }
    else{
      RCLCPP_DEBUG_STREAM(get_logger(), "No roadway object");
      negotiate = false;
    }

    //plan lanechange without filling in response
    RCLCPP_DEBUG_STREAM(get_logger(), "Planning lane change trajectory");

    std::string maneuver_id = maneuver_plan[0].lane_change_maneuver.parameters.maneuver_id;
    if (original_lc_maneuver_values_.find(maneuver_id) == original_lc_maneuver_values_.end()) {
      // If this lane change maneuver ID is being received for this first time, store its original start_dist and starting_lane_id locally
      RCLCPP_DEBUG_STREAM(get_logger(), "Received maneuver id " << maneuver_id << " for the first time");
      RCLCPP_DEBUG_STREAM(get_logger(), "Original start dist is " << maneuver_plan[0].lane_change_maneuver.start_dist);
      RCLCPP_DEBUG_STREAM(get_logger(), "Original starting_lane_id is " << maneuver_plan[0].lane_change_maneuver.starting_lane_id);

      // Create LaneChangeManeuverOriginalValues object for this lane change maneuver and add it to original_lc_maneuver_values_
      LaneChangeManeuverOriginalValues original_lc_values;
      original_lc_values.maneuver_id = maneuver_id;
      original_lc_values.original_starting_lane_id = maneuver_plan[0].lane_change_maneuver.starting_lane_id;
      original_lc_values.original_start_dist = maneuver_plan[0].lane_change_maneuver.start_dist;

      original_lc_maneuver_values_[maneuver_id] = original_lc_values;
    }
    else {
      // If the vehicle has just started this lane change, store its initial velocity locally; this velocity will be maintained throughout the lane change
      if (current_downtrack >= (original_lc_maneuver_values_[maneuver_id]).original_start_dist  && !(original_lc_maneuver_values_[maneuver_id]).has_started) {
          original_lc_maneuver_values_[maneuver_id].has_started = true;
          original_lc_maneuver_values_[maneuver_id].original_longitudinal_vel_ms = std::max(req->vehicle_state.longitudinal_vel, config_.minimum_speed);

          RCLCPP_DEBUG_STREAM(get_logger(), "Lane change maneuver " << maneuver_id << " has started, maintaining speed (in m/s): " <<
                          original_lc_maneuver_values_[maneuver_id].original_longitudinal_vel_ms << " throughout lane change");
      }
    }

    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> planned_trajectory_points = plan_lanechange(req);
      
    if(negotiate){
      RCLCPP_DEBUG_STREAM(get_logger(), "Negotiating");
      //send mobility request
      //Planning for first lane change maneuver
      carma_v2x_msgs::msg::MobilityRequest request = create_mobility_request(planned_trajectory_points, maneuver_plan[0]);
      outgoing_mobility_request_pub_->publish(request);
      if(!request_sent_){
        request_sent_time_ = this->now();
        request_sent_ = true;
      }
      carma_planning_msgs::msg::LaneChangeStatus lc_status_msg;
      lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::PLAN_SENT;
      lc_status_msg.description = "Requested lane merge";
      lanechange_status_pub_->publish(lc_status_msg);
    }

    //if ack mobility response, send lanechange response
    if(!negotiate || is_lanechange_accepted_){
      RCLCPP_DEBUG_STREAM(get_logger(), "negotiate:" << negotiate);
      RCLCPP_DEBUG_STREAM(get_logger(), "is_lanechange_accepted:" << is_lanechange_accepted_);

      RCLCPP_DEBUG_STREAM(get_logger(), "Adding to response");
      add_trajectory_to_response(req,resp,planned_trajectory_points);
            
    }
    else{
      if(!negotiate && !request_sent_){
        request_sent_time_ = this->now();
        request_sent_ = true;
      }
      rclcpp::Time planning_end_time = this->now();
      rclcpp::Duration passed_time = planning_end_time - request_sent_time_;
      if(passed_time.seconds() >= config_.lanechange_time_out){
        carma_planning_msgs::msg::LaneChangeStatus lc_status_msg;
        lc_status_msg.status = carma_planning_msgs::msg::LaneChangeStatus::TIMED_OUT;
        lc_status_msg.description = "Request timed out for lane merge";
        lanechange_status_pub_->publish(lc_status_msg);
        request_sent_ = false;  //Reset variable
      }
    }
    
    return;  
  }

  void CooperativeLaneChangePlugin::add_trajectory_to_response(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req, 
                                  carma_planning_msgs::srv::PlanTrajectory::Response::SharedPtr resp, 
                                  const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& planned_trajectory_points)
  {
    carma_planning_msgs::msg::TrajectoryPlan trajectory_plan;
    trajectory_plan.header.frame_id = "map";
    trajectory_plan.header.stamp = this->now();
    trajectory_plan.trajectory_id = boost::uuids::to_string(boost::uuids::random_generator()());

    trajectory_plan.trajectory_points = planned_trajectory_points;
    trajectory_plan.initial_longitudinal_velocity = std::max(req->vehicle_state.longitudinal_vel, config_.minimum_speed);
    resp->trajectory_plan = trajectory_plan;

    resp->related_maneuvers.push_back(req->maneuver_index_to_plan);

    resp->maneuver_status.push_back(carma_planning_msgs::srv::PlanTrajectory::Response::MANEUVER_IN_PROGRESS);
  }

  carma_v2x_msgs::msg::MobilityRequest CooperativeLaneChangePlugin::create_mobility_request(std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& trajectory_plan, carma_planning_msgs::msg::Maneuver& maneuver)
  {
    carma_v2x_msgs::msg::MobilityRequest request_msg;
    carma_v2x_msgs::msg::MobilityHeader header;
    header.sender_id = config_.vehicle_id;
    header.recipient_id = DEFAULT_STRING_;  
    header.sender_bsm_id = bsmIDtoString(bsm_core_);
    header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
    clc_request_id_ = header.plan_id;
    header.timestamp = rclcpp::Time(trajectory_plan.front().target_time).nanoseconds() * 1000000;
    request_msg.m_header = header;

    request_msg.strategy = "carma/cooperative-lane-change";
    request_msg.plan_type.type = carma_v2x_msgs::msg::PlanType::CHANGE_LANE_LEFT;
    
    //Urgency- Currently unassigned
    int urgency;
    if(maneuver_fraction_completed_ <= config_.starting_fraction){
      urgency = 10;
    }
    else if(maneuver_fraction_completed_ <= config_.mid_fraction){
      urgency = 5;
    }
    else{
      urgency = 1;
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "Maneuver fraction completed:"<<maneuver_fraction_completed_);
    request_msg.urgency = urgency;

    //Strategy params
    //Encode JSON with Boost Property Tree
    using boost::property_tree::ptree;
    ptree pt;
    double end_speed_floor = std::floor(maneuver.lane_change_maneuver.end_speed);
    int end_speed_fractional = (maneuver.lane_change_maneuver.end_speed - end_speed_floor) * 10;

    RCLCPP_DEBUG_STREAM(get_logger(), "end_speed_floor: " << end_speed_floor);
    RCLCPP_DEBUG_STREAM(get_logger(), "end_speed_fractional: " << end_speed_fractional);
    RCLCPP_DEBUG_STREAM(get_logger(), "start_lanelet_id: " << maneuver.lane_change_maneuver.starting_lane_id);
    RCLCPP_DEBUG_STREAM(get_logger(), "end_lanelet_id: " << maneuver.lane_change_maneuver.ending_lane_id);

    pt.put("s",(int)end_speed_floor); 
    pt.put("f",end_speed_fractional); 
    pt.put("sl",maneuver.lane_change_maneuver.starting_lane_id);
    pt.put("el", maneuver.lane_change_maneuver.ending_lane_id);

    std::stringstream body_stream;
    boost::property_tree::json_parser::write_json(body_stream,pt);
    request_msg.strategy_params = body_stream.str(); 
    RCLCPP_DEBUG_STREAM(get_logger(), "request_msg.strategy_params: " << request_msg.strategy_params);

    //Trajectory
    carma_v2x_msgs::msg::Trajectory trajectory;
    if (map_projector_) {
      trajectory = trajectory_plan_to_trajectory(trajectory_plan);
      //Location
      carma_planning_msgs::msg::TrajectoryPlanPoint temp_loc_to_convert;
      temp_loc_to_convert.x = pose_msg_.pose.position.x;
      temp_loc_to_convert.y = pose_msg_.pose.position.y;
      carma_v2x_msgs::msg::LocationECEF location = trajectory_point_to_ecef(temp_loc_to_convert);

      //Using trajectory first point time as location timestamp
      location.timestamp = rclcpp::Time(trajectory_plan.front().target_time).nanoseconds() * 1000000;

      request_msg.location = location;
    }
    else 
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Map projection not available to be used with request message");
    }
        
    request_msg.trajectory = trajectory;
    request_msg.expiration = rclcpp::Time(trajectory_plan.back().target_time).seconds();
    RCLCPP_DEBUG_STREAM(get_logger(), "request_msg.expiration: " << request_msg.expiration << " of which string size: " << std::to_string(request_msg.expiration).size());
      
    return request_msg;
  }

  carma_v2x_msgs::msg::Trajectory CooperativeLaneChangePlugin::trajectory_plan_to_trajectory(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points) const
  {
    carma_v2x_msgs::msg::Trajectory traj;
    carma_v2x_msgs::msg::LocationECEF ecef_location = trajectory_point_to_ecef(traj_points[0]);

    if (traj_points.size() < 2){
      RCLCPP_WARN_STREAM(get_logger(), "Received Trajectory Plan is too small");
      traj.offsets = {};
    }
    else{
      carma_v2x_msgs::msg::LocationECEF prev_point = ecef_location;
      for (size_t i = 1; i < traj_points.size(); i++){
             
        carma_v2x_msgs::msg::LocationOffsetECEF offset;
        carma_v2x_msgs::msg::LocationECEF new_point = trajectory_point_to_ecef(traj_points[i]); // m to cm to fit the msg standard
        offset.offset_x = (int16_t)(new_point.ecef_x - prev_point.ecef_x);  
        offset.offset_y = (int16_t)(new_point.ecef_y - prev_point.ecef_y);
        offset.offset_z = (int16_t)(new_point.ecef_z - prev_point.ecef_z);
        prev_point = new_point;
        traj.offsets.push_back(offset);
      }
    }

    traj.location = ecef_location; 

    return traj;
  }

  carma_v2x_msgs::msg::LocationECEF CooperativeLaneChangePlugin::trajectory_point_to_ecef(const carma_planning_msgs::msg::TrajectoryPlanPoint& traj_point) const 
  {
    if (!map_projector_) {
      throw std::invalid_argument("No map projector available for ecef conversion");
    }
    carma_v2x_msgs::msg::LocationECEF location;
        
    lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({traj_point.x, traj_point.y, 0.0}, 1);
    location.ecef_x = ecef_point.x() * 100.0; // Convert cm to m
    location.ecef_y = ecef_point.y() * 100.0;
    location.ecef_z = ecef_point.z() * 100.0;

    return location;
  } 

  std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> CooperativeLaneChangePlugin::plan_lanechange(carma_planning_msgs::srv::PlanTrajectory::Request::SharedPtr req)
  {
    lanelet::BasicPoint2d veh_pos(req->vehicle_state.x_pos_global, req->vehicle_state.y_pos_global);
    double current_downtrack = wm_->routeTrackPos(veh_pos).downtrack;

    // Only plan the trajectory for the requested LANE_CHANGE maneuver
    std::vector<carma_planning_msgs::msg::Maneuver> maneuver_plan;
    if(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan].type != carma_planning_msgs::msg::Maneuver::LANE_CHANGE) {
      throw std::invalid_argument ("Cooperative Lane Change Plugin doesn't support this maneuver type");
    }
    maneuver_plan.push_back(req->maneuver_plan.maneuvers[req->maneuver_index_to_plan]);

    if(current_downtrack >= maneuver_plan.front().lane_change_maneuver.end_dist){
      request_sent_ = false;
    }
    
    basic_autonomy::waypoint_generation::DetailedTrajConfig wpg_detail_config;
    basic_autonomy::waypoint_generation::GeneralTrajConfig wpg_general_config;

    wpg_general_config = basic_autonomy::waypoint_generation::compose_general_trajectory_config("cooperative_lanechange", config_.downsample_ratio, config_.turn_downsample_ratio);
       
    wpg_detail_config = basic_autonomy::waypoint_generation::compose_detailed_trajectory_config(config_.trajectory_time_length, 
                                                                        config_.curve_resample_step_size, config_.minimum_speed, 
                                                                        config_.max_accel, config_.lateral_accel_limit, 
                                                                        config_.speed_moving_average_window_size, 
                                                                        config_.curvature_moving_average_window_size, config_.back_distance,
                                                                        config_.buffer_ending_downtrack);

    RCLCPP_DEBUG_STREAM(get_logger(), "Current downtrack: " << current_downtrack);
        
    std::string maneuver_id = maneuver_plan.front().lane_change_maneuver.parameters.maneuver_id;
    double original_start_dist = current_downtrack; // Initialize so original_start_dist cannot be less than the current downtrack
        
    if (original_lc_maneuver_values_.find(maneuver_id) != original_lc_maneuver_values_.end()) {
      // Obtain the original start_dist associated with this lane change maneuver
      original_start_dist = original_lc_maneuver_values_[maneuver_id].original_start_dist;
      RCLCPP_DEBUG_STREAM(get_logger(), "Maneuver id " << maneuver_id << " original start_dist is " << original_start_dist);

      // Set this maneuver's starting_lane_id to the original starting_lane_id associated with this lane change maneuver
      maneuver_plan.front().lane_change_maneuver.starting_lane_id = original_lc_maneuver_values_[maneuver_id].original_starting_lane_id;
      RCLCPP_DEBUG_STREAM(get_logger(), "Updated maneuver id " << maneuver_id << " starting_lane_id to its original value of " << original_lc_maneuver_values_[maneuver_id].original_starting_lane_id);

      // If the vehicle has started this lane change, set the request's vehicle_state.longitudinal_vel to the velocity that the vehicle began this lane change at
      if(original_lc_maneuver_values_[maneuver_id].has_started) {
        req->vehicle_state.longitudinal_vel = original_lc_maneuver_values_[maneuver_id].original_longitudinal_vel_ms;
        RCLCPP_DEBUG_STREAM(get_logger(), "Updating vehicle_state.longitudinal_vel to the initial lane change value of " << original_lc_maneuver_values_[maneuver_id].original_longitudinal_vel_ms);
      }
    }
    else {
      RCLCPP_WARN_STREAM(get_logger(), "No original values for lane change maneuver were found!");
    }

    double starting_downtrack = std::min(current_downtrack, original_start_dist);

    auto points_and_target_speeds = basic_autonomy::waypoint_generation::create_geometry_profile(maneuver_plan, starting_downtrack, wm_, ending_state_before_buffer_, req->vehicle_state, wpg_general_config, wpg_detail_config);

    // Calculate maneuver fraction completed (current_downtrack/(ending_downtrack-starting_downtrack)
    auto maneuver_end_dist = maneuver_plan.back().lane_change_maneuver.end_dist;
    auto maneuver_start_dist = maneuver_plan.front().lane_change_maneuver.start_dist;
    maneuver_fraction_completed_ = (maneuver_start_dist - current_downtrack)/(maneuver_end_dist - maneuver_start_dist);

    RCLCPP_DEBUG_STREAM(get_logger(), "Maneuvers to points size: " << points_and_target_speeds.size());
    auto downsampled_points = carma_ros2_utils::containers::downsample_vector(points_and_target_speeds, config_.downsample_ratio);

    std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint> trajectory_points = basic_autonomy::waypoint_generation::compose_lanechange_trajectory_from_path(downsampled_points, req->vehicle_state, req->header.stamp,
                                                                                      wm_, ending_state_before_buffer_, wpg_detail_config);
    RCLCPP_DEBUG_STREAM(get_logger(), "Compose Trajectory size: " << trajectory_points.size());
    return trajectory_points;
  }

  void CooperativeLaneChangePlugin::georeference_cb(const std_msgs::msg::String::UniquePtr msg) 
  {
    map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
  }

  std::string CooperativeLaneChangePlugin::bsmIDtoString(carma_v2x_msgs::msg::BSMCoreData bsm_core)
  {
    std::string res = "";
    for (size_t i = 0; i < bsm_core.id.size(); i++){
      res += std::to_string(bsm_core.id[i]);
    }
    return res;
  }

  bool CooperativeLaneChangePlugin::get_availability() {
    return true;
  }

  std::string CooperativeLaneChangePlugin::get_version_id() {
    return "v4.0"; // Version ID matches the value set in this package's package.xml
  }

} // cooperative_lanechange

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(cooperative_lanechange::CooperativeLaneChangePlugin)
