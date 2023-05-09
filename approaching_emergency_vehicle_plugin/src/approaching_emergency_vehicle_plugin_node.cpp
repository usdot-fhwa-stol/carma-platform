/*
 * Copyright (C) 2022-2023 LEIDOS.
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
#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_plugin_node.hpp"

namespace approaching_emergency_vehicle_plugin
{
  namespace std_ph = std::placeholders;

  namespace {
    /**
    * \brief Anonymous function to extract maneuver end speed which cannot be obtained with GET_MANEUVER_PROPERY calls 
    *        due to it missing in stop and wait plugin
    */ 
    double getManeuverEndSpeed(const carma_planning_msgs::msg::Maneuver& mvr) { 
      switch(mvr.type) {
        case carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING:
          return mvr.lane_following_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::LANE_CHANGE:
          return mvr.lane_change_maneuver.end_speed;
        case carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT:
          return 0;
        default:
          return 0;
      }
    }
  }

  ApproachingEmergencyVehiclePlugin::ApproachingEmergencyVehiclePlugin(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::StrategicPlugin(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.passing_threshold = declare_parameter<double>("passing_threshold", config_.passing_threshold);
    config_.approaching_threshold = declare_parameter<double>("approaching_threshold", config_.approaching_threshold);
    config_.finished_passing_threshold = declare_parameter<double>("finished_passing_threshold", config_.finished_passing_threshold);
    config_.min_lane_following_duration_before_lane_change = declare_parameter<double>("min_lane_following_duration_before_lane_change", config_.min_lane_following_duration_before_lane_change);
    config_.bsm_processing_frequency = declare_parameter<double>("bsm_processing_frequency", config_.bsm_processing_frequency);
    config_.speed_limit_reduction_during_passing = declare_parameter<double>("speed_limit_reduction_during_passing", config_.speed_limit_reduction_during_passing);
    config_.minimum_reduced_speed_limit = declare_parameter<double>("minimum_reduced_speed_limit", config_.minimum_reduced_speed_limit);
    config_.default_speed_limit = declare_parameter<double>("default_speed_limit", config_.default_speed_limit);
    config_.reduced_speed_buffer = declare_parameter<double>("reduced_speed_buffer", config_.reduced_speed_buffer);
    config_.timeout_check_frequency = declare_parameter<double>("timeout_check_frequency", config_.timeout_check_frequency);
    config_.timeout_duration = declare_parameter<double>("timeout_duration", config_.timeout_duration);
    config_.minimal_plan_duration = declare_parameter<double>("minimal_plan_duration", config_.minimal_plan_duration);
    config_.buffer_distance_before_stopping = declare_parameter<double>("buffer_distance_before_stopping", config_.buffer_distance_before_stopping);
    config_.stopping_accel_limit_multiplier = declare_parameter<double>("stopping_accel_limit_multiplier", config_.stopping_accel_limit_multiplier);
    config_.vehicle_acceleration_limit = declare_parameter<double>("vehicle_acceleration_limit", config_.vehicle_acceleration_limit);
    config_.route_end_point_buffer = declare_parameter<double>("route_end_point_buffer", config_.route_end_point_buffer);
    config_.approaching_erv_status_publication_frequency = declare_parameter<double>("approaching_erv_status_publication_frequency", config_.approaching_erv_status_publication_frequency);
    config_.warning_broadcast_frequency = declare_parameter<double>("warning_broadcast_frequency", config_.warning_broadcast_frequency);
    config_.max_warning_broadcasts = declare_parameter<int>("max_warning_broadcasts", config_.max_warning_broadcasts);
    config_.vehicle_length = declare_parameter<double>("vehicle_length", config_.vehicle_length);
    config_.lane_following_plugin = declare_parameter<std::string>("lane_following_plugin", config_.lane_following_plugin);
    config_.lane_change_plugin = declare_parameter<std::string>("lane_change_plugin", config_.lane_change_plugin);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);

    erv_world_model_.reset(new carma_wm::CARMAWorldModel);
  }

  rcl_interfaces::msg::SetParametersResult ApproachingEmergencyVehiclePlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_1 = update_params<double>({
        {"passing_threshold", config_.passing_threshold},
        {"approaching_threshold", config_.approaching_threshold},
        {"finished_passing_threshold", config_.finished_passing_threshold},
        {"min_lane_following_duration_before_lane_change", config_.min_lane_following_duration_before_lane_change},
        {"bsm_processing_frequency", config_.bsm_processing_frequency},
        {"speed_limit_reduction_during_passing", config_.speed_limit_reduction_during_passing},
        {"minimum_reduced_speed_limit", config_.minimum_reduced_speed_limit},
        {"default_speed_limit", config_.default_speed_limit},
        {"reduced_speed_buffer", config_.reduced_speed_buffer},
        {"timeout_check_frequency", config_.timeout_check_frequency},
        {"timeout_duration", config_.timeout_duration},
        {"minimal_plan_duration", config_.minimal_plan_duration},
        {"buffer_distance_before_stopping", config_.buffer_distance_before_stopping},
        {"stopping_accel_limit_multiplier", config_.stopping_accel_limit_multiplier},
        {"vehicle_acceleration_limit", config_.vehicle_acceleration_limit},
        {"route_end_point_buffer", config_.route_end_point_buffer},
        {"approaching_erv_status_publication_frequency", config_.approaching_erv_status_publication_frequency},
        {"warning_broadcast_frequency", config_.warning_broadcast_frequency},
        {"vehicle_length", config_.vehicle_length}
    }, parameters);

    auto error_2 = update_params<int>({
        {"max_warning_broadcasts", config_.max_warning_broadcasts}
    }, parameters);

    auto error_3 = update_params<std::string>({
        {"lane_following_plugin", config_.lane_following_plugin},
        {"lane_change_plugin", config_.lane_change_plugin},
        {"stop_and_wait_plugin", config_.stop_and_wait_plugin},
        {"vehicle_id", config_.vehicle_id}
    }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_1 && !error_2 && !error_3;

    return result;
  }

  carma_ros2_utils::CallbackReturn ApproachingEmergencyVehiclePlugin::on_configure_plugin()
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "ApproachingEmergencyVehiclePlugin trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("passing_threshold", config_.passing_threshold);
    get_parameter<double>("approaching_threshold", config_.approaching_threshold);
    get_parameter<double>("finished_passing_threshold", config_.finished_passing_threshold);
    get_parameter<double>("min_lane_following_duration_before_lane_change", config_.min_lane_following_duration_before_lane_change);
    get_parameter<double>("bsm_processing_frequency", config_.bsm_processing_frequency);
    get_parameter<double>("speed_limit_reduction_during_passing", config_.speed_limit_reduction_during_passing);
    get_parameter<double>("minimum_reduced_speed_limit", config_.minimum_reduced_speed_limit);
    get_parameter<double>("default_speed_limit", config_.default_speed_limit);
    get_parameter<double>("reduced_speed_buffer", config_.reduced_speed_buffer);
    get_parameter<double>("timeout_check_frequency", config_.timeout_check_frequency);
    get_parameter<double>("timeout_duration", config_.timeout_duration);
    get_parameter<double>("minimal_plan_duration", config_.minimal_plan_duration);
    get_parameter<double>("buffer_distance_before_stopping", config_.buffer_distance_before_stopping);
    get_parameter<double>("stopping_accel_limit_multiplier", config_.stopping_accel_limit_multiplier);
    get_parameter<double>("vehicle_acceleration_limit", config_.vehicle_acceleration_limit);
    get_parameter<double>("route_end_point_buffer", config_.route_end_point_buffer);
    get_parameter<double>("approaching_erv_status_publication_frequency", config_.approaching_erv_status_publication_frequency);
    get_parameter<double>("warning_broadcast_frequency", config_.warning_broadcast_frequency);
    get_parameter<double>("vehicle_length", config_.vehicle_length);
    get_parameter<int>("max_warning_broadcasts", config_.max_warning_broadcasts);
    get_parameter<std::string>("lane_following_plugin", config_.lane_following_plugin);
    get_parameter<std::string>("lane_change_plugin", config_.lane_change_plugin);
    get_parameter<std::string>("vehicle_id", config_.vehicle_id);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "ApproachingEmergencyVehiclePlugin Config: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&ApproachingEmergencyVehiclePlugin::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    incoming_bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("incoming_bsm", 1,
                                            std::bind(&ApproachingEmergencyVehiclePlugin::incomingBsmCallback, this, std_ph::_1));

    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 10,
                                            std::bind(&ApproachingEmergencyVehiclePlugin::georeferenceCallback, this, std_ph::_1));

    route_state_sub_ = create_subscription<carma_planning_msgs::msg::RouteState>("route_state", 10,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::routeStateCallback, this, std_ph::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 10,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::twistCallback, this, std_ph::_1));

    incoming_emergency_vehicle_ack_sub_ = create_subscription<carma_v2x_msgs::msg::EmergencyVehicleAck>("incoming_emergency_vehicle_ack", 10,
                                                              std::bind(&ApproachingEmergencyVehiclePlugin::incomingEmergencyVehicleAckCallback, this, std_ph::_1));

    guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("state", 1,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::guidanceStateCallback, this, std_ph::_1));

    route_sub_ = create_subscription<carma_planning_msgs::msg::Route>("route", 1,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::routeCallback, this, std_ph::_1));

    // Setup publishers
    outgoing_emergency_vehicle_response_pub_ = create_publisher<carma_v2x_msgs::msg::EmergencyVehicleResponse>("outgoing_emergency_vehicle_response", 10);

    approaching_erv_status_pub_ = create_publisher<carma_msgs::msg::UIInstructions>("approaching_erv_status", 10);

    hazard_light_cmd_pub_ = create_publisher<std_msgs::msg::Bool>("hazard_light_status", 10);

    wm_ = get_world_model();

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }
  
  carma_ros2_utils::CallbackReturn ApproachingEmergencyVehiclePlugin::on_activate_plugin()
  {
    // Timer setup for generating a BSM
    int erv_timeout_check_period_ms = (1 / config_.timeout_check_frequency) * 1000; // Conversion from frequency (Hz) to milliseconds time period
    erv_timeout_timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(erv_timeout_check_period_ms),
                          std::bind(&ApproachingEmergencyVehiclePlugin::checkForErvTimeout, this));

    // Timer setup for broadcasting EmergencyVehicleResponse warning message to an approaching ERV when the ego vehicle cannot change lanes out of the ERV's path
    int emergency_vehicle_response_period_ms = (1 / config_.warning_broadcast_frequency) * 1000; // Conversion from frequency (Hz) to milliseconds time period
    warning_broadcast_timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(emergency_vehicle_response_period_ms),
                          std::bind(&ApproachingEmergencyVehiclePlugin::broadcastWarningToErv, this));

    // Timer setup for publishing approaching ERV status update to the Web UI
    int approaching_erv_status_period_ms = (1 / config_.approaching_erv_status_publication_frequency) * 1000; // Conversion from frequency (Hz) to milliseconds time period
    approaching_emergency_vehicle_status_timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(approaching_erv_status_period_ms),
                          std::bind(&ApproachingEmergencyVehiclePlugin::publishApproachingErvStatus, this));

    // Timer setup for publishing hazard light ON/OFF status boolean
    int hazard_light_status_ms = (1 / 30) * 1000; // Conversion from frequency 30(Hz) to milliseconds time period
    hazard_light_timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(hazard_light_status_ms),
                          std::bind(&ApproachingEmergencyVehiclePlugin::publishHazardLightStatus, this));

    return CallbackReturn::SUCCESS;
  }

  void ApproachingEmergencyVehiclePlugin::publishHazardLightStatus()
  {
    if (transition_table_.getState() == ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV &&
    has_tracked_erv_ &&
    tracked_erv_.lane_index == ego_lane_index_)
    {
      hazard_light_cmd_ = true;
    }
    else
    {
      hazard_light_cmd_ = false;
    }
    std_msgs::msg::Bool msg;
    msg.data = hazard_light_cmd_;
    hazard_light_cmd_pub_->publish(msg);
  }

  void ApproachingEmergencyVehiclePlugin::checkForErvTimeout(){
    // Trigger timeout event if a timeout has occurred for the currently tracked ERV
    if(has_tracked_erv_){
      double seconds_since_prev_update = (this->get_clock()->now() - tracked_erv_.latest_update_time).seconds();

      if(seconds_since_prev_update >= config_.timeout_duration){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Timeout occurred for ERV " << tracked_erv_.vehicle_id);
        has_tracked_erv_ = false;
        has_planned_upcoming_lc_ = false;
        transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT);
      }
    }
  }

  void ApproachingEmergencyVehiclePlugin::broadcastWarningToErv(){
    if(has_tracked_erv_ && should_broadcast_warnings_){
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Sending a warning message to " << tracked_erv_.vehicle_id);

      carma_v2x_msgs::msg::EmergencyVehicleResponse msg;
      msg.m_header.sender_id = config_.vehicle_id;
      msg.m_header.recipient_id = tracked_erv_.vehicle_id;

      msg.can_change_lanes = false;
      msg.reason_exists = true;
      msg.reason = "Vehicle " + config_.vehicle_id + " is unable to change lanes.";

      num_warnings_broadcasted_++;

      outgoing_emergency_vehicle_response_pub_->publish(msg);

      // Reset counter and boolean flag if the maximum number of warnings have been broadcasted
      if(num_warnings_broadcasted_ >= config_.max_warning_broadcasts){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Maximum number of warning messages sent to " << tracked_erv_.vehicle_id);
        num_warnings_broadcasted_ = 0;
        should_broadcast_warnings_ = false;
      }
    }
  }

  void ApproachingEmergencyVehiclePlugin::publishApproachingErvStatus(){
    // Generate the status message and publish it to the applicable ROS topic
    carma_msgs::msg::UIInstructions status_msg = generateApproachingErvStatusMessage();
    approaching_erv_status_pub_->publish(status_msg);
  }
  
  carma_msgs::msg::UIInstructions ApproachingEmergencyVehiclePlugin::generateApproachingErvStatusMessage(){
    // Initialize the message that will be returned by this function
    carma_msgs::msg::UIInstructions status_msg;
    status_msg.type = carma_msgs::msg::UIInstructions::INFO;

    /**
      * Note: APPROACHING_ERV_STATUS_PARAMS format:
      *       "HAS_APPROACHING_ERV:%1%,TIME_UNTIL_PASSING:%2$.1f,EGO_VEHICLE_ACTION:%3%"
      *       |--------0----------------------1----------------------2--------------|
      *       Index 0: (Boolean) Value indicating whether the ego vehicle is tracking an approaching ERV
      *       Index 1: (Double; rounded to first decimal place)  If an approaching ERV exists, this indicates the estimated seconds until the ERV passes the ego vehicle.
      *       Index 2: (String)  If an approaching ERV exists, this describes the current action of the ego vehicle in response to the approaching ERV. 
      *       NOTE: The values of indexes 1 and 2 can be ignored if index 0 indicates that no approaching ERV is being tracked.
      */
    boost::format fmter(APPROACHING_ERV_STATUS_PARAMS);
    
    // Index 0 of formatted string; indicates whether an ERV that is approaching the ego vehicle is being tracked
    if(has_tracked_erv_ && tracked_erv_.seconds_until_passing <= config_.approaching_threshold){
      fmter %true;
    }
    else{
      fmter %false;
    }

    // Index 1 of formatted string; indicates estimated time until tracked ERV passes the ego vehicle
    fmter %tracked_erv_.seconds_until_passing;

    // Add Index 2 of formatted string based on this plugin's current ApproachingEmergencyVehicleState
    switch (transition_table_.getState())
    {
      case ApproachingEmergencyVehicleState::NO_APPROACHING_ERV:
        fmter %"No action.";
        break;

      case ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV:
        // Add ego vehicle action based on the direction of the upcoming lane change
        if(upcoming_lc_params_.is_right_lane_change){
          fmter %"Approaching ERV is in our lane. Attempting to change lanes to the right.";
        }
        else{
          fmter %"Approaching ERV is in our lane. Attempting to change lanes to the left.";
        }

        break;

      case ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV:
        // Add ego vehicle action based on whether the current speed is near the reduced target speed of the latest maneuver plan's first maneuver
        if(!latest_maneuver_plan_.maneuvers.empty()){
          // Extract the target speed in m/s from the latest maneuver plan's first maneuver and convert it to mph
          double target_speed_ms = getManeuverEndSpeed(latest_maneuver_plan_.maneuvers[0]);
          int target_speed_mph = std::round(target_speed_ms * METERS_PER_SEC_TO_MILES_PER_HOUR);

          if(ego_lane_index_ == tracked_erv_.lane_index){
            if(!is_maintaining_non_reduced_speed_){
              if(abs(current_speed_ - target_speed_ms) <= config_.reduced_speed_buffer){
                fmter %("Approaching ERV is in our lane and a lane change is not possible. Remaining in the current lane at a reduced speed of " + std::to_string(target_speed_mph) + " mph.");
              }
              else{
                fmter %("Approaching ERV is in our lane and a lane change is not possible. Remaining in the current lane and slowing down to a reduced speed of " + std::to_string(target_speed_mph) + " mph.");
              }
            }
            else{
              int non_reduced_speed_to_maintain_mph = std::round(non_reduced_speed_to_maintain_ * METERS_PER_SEC_TO_MILES_PER_HOUR);
              fmter %("Approaching ERV is in our lane and a lane change is not possible. Remaining in the current lane and maintaining a speed of " + std::to_string(non_reduced_speed_to_maintain_mph) + " mph.");
            }
          }
          else{
            if(abs(current_speed_ - target_speed_ms) <= config_.reduced_speed_buffer){
              fmter %("Approaching ERV is in adjacent lane. Remaining in the current lane at a reduced speed of " + std::to_string(target_speed_mph) + " mph.");
            }
            else{
              fmter %("Approaching ERV is in adjacent lane. Remaining in the current lane and slowing down to a reduced speed of " + std::to_string(target_speed_mph) + " mph.");
            }
          }
        }
        else{
          throw std::invalid_argument("State is SLOWING_DOWN_FOR_ERV but latest maneuver plan is empty!");
        }
        break;

      default:
        throw std::invalid_argument("Transition table in unsupported state");
    }

    // Add formatted string to status message
    std::string str = fmter.str();
    status_msg.msg = str;

    return status_msg;
  }

  boost::optional<lanelet::BasicPoint2d> ApproachingEmergencyVehiclePlugin::getErvPositionInMap(const double& current_latitude, const double& current_longitude){
    // Check if the map projection is available
    if (!map_projector_) {
      throw std::invalid_argument("Attempting to get ERV's current position in map before map projection was set");
    }
      
    // Build map projector
    lanelet::projection::LocalFrameProjector projector(map_projector_.get().c_str());

    // Create vector to hold ERV's projected current position
    std::vector<lanelet::BasicPoint3d> erv_current_position_projected_vec;

    // Add ERV's current location to the beginning of erv_current_position_projected_vec
    lanelet::GPSPoint current_erv_location;
    current_erv_location.lat = current_latitude;
    current_erv_location.lon = current_longitude;

    // Convert ERV's projected position to its position in the map frame
    erv_current_position_projected_vec.emplace_back(projector.forward(current_erv_location));
    auto erv_current_position_in_map_vec = lanelet::utils::transform(erv_current_position_projected_vec, [](auto a) { return lanelet::traits::to2D(a); });

    // Conduct size check since only the first element is being returned
    if(!erv_current_position_in_map_vec.empty()){
      return erv_current_position_in_map_vec[0];
    }
    else{
      return boost::optional<lanelet::BasicPoint2d>();
    }
  }

  boost::optional<ErvInformation> ApproachingEmergencyVehiclePlugin::getErvInformationFromBsm(carma_v2x_msgs::msg::BSM::UniquePtr msg){
    // Initialize ErvInformation object, which will be populated with information from this BSM if it is from an ERV
    ErvInformation erv_information;

    // Get vehicle_id from the BSM
    std::stringstream ss;
    for(size_t i = 0; i < msg->core_data.id.size(); ++i){
      ss << std::setfill('0') << std::setw(2) << std::hex << (unsigned)msg->core_data.id.at(i);
    }
    erv_information.vehicle_id = ss.str();

    // Check whether vehicle's lights and sirens are active
    bool has_active_lights_and_sirens = false;
    if(msg->presence_vector & carma_v2x_msgs::msg::BSM::HAS_PART_II){

      // Parse BSM Part II Content to determine whether the vehicle's lights and sirens are active
      if(msg->part_ii.size() > 0){
        for(size_t i = 0; i < msg->part_ii.size(); ++i){

          // special_vehicle_extensions contain the status of the vehicle's lights and sirens
          if(msg->part_ii[i].part_ii_id == carma_v2x_msgs::msg::BSMPartIIExtension::SPECIAL_VEHICLE_EXT){
            carma_v2x_msgs::msg::SpecialVehicleExtensions special_vehicle_ext = msg->part_ii[i].special_vehicle_extensions;

            if(special_vehicle_ext.presence_vector & carma_v2x_msgs::msg::SpecialVehicleExtensions::HAS_VEHICLE_ALERTS){
              if(special_vehicle_ext.vehicle_alerts.siren_use.siren_in_use == j2735_v2x_msgs::msg::SirenInUse::IN_USE
                && special_vehicle_ext.vehicle_alerts.lights_use.lightbar_in_use == j2735_v2x_msgs::msg::LightbarInUse::IN_USE){
                  has_active_lights_and_sirens = true;
              }
            }
          }
        }
      }
    }

    if(!has_active_lights_and_sirens){
      // BSM is not a valid ERV BSM since the lights and sirens are not both active; return an empty object
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "BSM is not a valid ERV BSM since the lights and sirens are not both active; return an empty object");

      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current speed from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE){
      erv_information.current_speed = msg->core_data.speed;
    }
    else{
      // BSM is not a valid ERV BSM since current speed is not included; return an empty object
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "BSM is not a valid ERV BSM since current speed is not included; return an empty object");

      return boost::optional<ErvInformation>();
    }

    if(erv_information.current_speed <= current_speed_){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Ignoring received BSM since ERV speed of " << erv_information.current_speed << " is less than ego speed of " << current_speed_);
      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current latitude from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE){
      erv_information.current_latitude = msg->core_data.latitude;
    }
    else{
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "No latitude available");

      // BSM is not a valid ERV BSM since current latitude is not included; return an empty object
      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current longitude from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE){

      erv_information.current_longitude = msg->core_data.longitude;
    }
    else{
      // BSM is not a valid ERV BSM since current longitude is not included; return an empty object
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "No longitude available");

      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current position in the map frame from the BSM
    boost::optional<lanelet::BasicPoint2d> erv_position_in_map = getErvPositionInMap(erv_information.current_latitude, erv_information.current_longitude);
    
    if(erv_position_in_map){
      erv_information.current_position_in_map = *erv_position_in_map;
    }

    // Get vehicle's route destination points from the BSM
    std::vector<carma_v2x_msgs::msg::Position3D> erv_destination_points;
    if(msg->presence_vector & carma_v2x_msgs::msg::BSM::HAS_REGIONAL){

      // Parse BSM Regional Extensions to determine whether BSM includes the ERV's route destination points
      if(msg->regional.size() > 0){
        for(size_t i = 0; i < msg->regional.size(); ++i){
          if(msg->regional[i].regional_extension_id == carma_v2x_msgs::msg::BSMRegionalExtension::ROUTE_DESTINATIONS){
            erv_destination_points = msg->regional[i].route_destination_points;
            break;
          }
        }
      }
    }
    
    if(erv_destination_points.empty()){
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "ERV's BSM does not contain any future destination points");
      
      // BSM is not a valid ERV BSM since it does not include destination points; return an empty object
      return boost::optional<ErvInformation>();
    }

    // Update the latest processing time of this ERV
    latest_erv_update_times_[erv_information.vehicle_id] = this->now();

    // Generate ERV's route based on its current position and its destination points
    lanelet::Optional<lanelet::routing::Route> erv_future_route = generateErvRoute(erv_information.current_latitude, erv_information.current_longitude, erv_destination_points);

    if(!erv_future_route){
      // ERV cannot be tracked since its route could not be generated; return an empty object
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "The ERV's route could not be generated");

      return boost::optional<ErvInformation>();
    }
    
    // Determine the ERV's current lane index
    // Note: For 'lane index', 0 is rightmost lane, 1 is second rightmost, etc.; Only the current travel direction is considered
    if(!erv_future_route.get().shortestPath().empty()){
      lanelet::ConstLanelet erv_current_lanelet = erv_future_route.get().shortestPath()[0];

      // NOTE: this logic checks if the ERV and CMV are on a same direction or not. 
      // Currently this check is sufficient to happen only once due to the use case scenarios
      if (is_same_direction_.find(erv_information.vehicle_id) == is_same_direction_.end()) 
      {
        is_same_direction_[erv_information.vehicle_id] = false;
        for (auto llt: erv_future_route.get().shortestPath()) // checks if ERV is on the same path assuming CMV got all of its planned route when detected
        {
          if (wm_->getRoute()->contains(llt))
          {
            is_same_direction_[erv_information.vehicle_id] = true;

            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Detected that ERV: " << erv_information.vehicle_id << " and CMV are travelling in the SAME direction");
            break;
          }
        }
      }

      if (!is_same_direction_[erv_information.vehicle_id])  // opposite direction
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Detected that ERV and CMV are travelling in DIFFERENT directions");
        return boost::optional<ErvInformation>(); // if opposite direction, do not track
      }

      // Get ERV's lane index
      int lane_index = wm_->getMapRoutingGraph()->rights(erv_current_lanelet).size();

      // A currently-tracked ERV must report the same lane index twice in a row before it is assigned the new lane index
      if(has_tracked_erv_){
        if(lane_index == tracked_erv_.previous_lane_index){
          erv_information.lane_index = lane_index;
        }
        else{
          erv_information.lane_index = tracked_erv_.previous_lane_index;
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Detected first new lane index of " << lane_index << ", ERV's lane index will remain " << tracked_erv_.previous_lane_index);
        }

        erv_information.previous_lane_index = lane_index;
      }
      else{
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "First time detecting this ERV, it's lane index will be " << lane_index);
        erv_information.lane_index = lane_index;
        erv_information.previous_lane_index = lane_index;
      }

     
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "ERV's lane index is " << erv_information.lane_index);

    }
    else{
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "ERV's shortest path is empty!");
    }

    // Get intersecting lanelet between ERV's future route and ego vehicle's future shortest path
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Calling getRouteIntersectingLanelet"); 
    boost::optional<lanelet::ConstLanelet> intersecting_lanelet = getRouteIntersectingLanelet(erv_future_route.get());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Done calling getRouteIntersectingLanelet"); 

    if(intersecting_lanelet){
      erv_information.intersecting_lanelet = *intersecting_lanelet;
    }
    else{
      // No intersecting lanelet between ERV and ego vehicle was found; return an empty object
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "No intersecting lanelet between the ERV's future route and the ego vehicle's future route was found.");

      return boost::optional<ErvInformation>();
    }

    // Get the time (seconds) until the ERV passes the ego vehicle
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Calling getSecondsUntilPassing()"); 
    boost::optional<double> seconds_until_passing = getSecondsUntilPassing(erv_future_route, erv_information.current_position_in_map, erv_information.current_speed, erv_information.intersecting_lanelet);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Done calling getSecondsUntilPassing()"); 

    if(seconds_until_passing){
      erv_information.seconds_until_passing = seconds_until_passing.get();
      RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "Detected approaching ERV that is passing ego vehicle in " << seconds_until_passing.get() << " seconds");
    }
    else{
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Detected ERV is not approaching the ego vehicle");

      // ERV will not be tracked since it is not considered to be approaching the ego vehicle; return an empty object
      return boost::optional<ErvInformation>();
    }

    return erv_information;
  }

  void ApproachingEmergencyVehiclePlugin::georeferenceCallback(const std_msgs::msg::String::UniquePtr msg) 
  {
    // Build projector from proj string
    map_projector_ = msg->data;
  }

  void ApproachingEmergencyVehiclePlugin::incomingEmergencyVehicleAckCallback(const carma_v2x_msgs::msg::EmergencyVehicleAck::UniquePtr msg) 
  {
    // Only process message if it is from the currently-tracked ERV and it is intended for the ego vehicle
    if(has_tracked_erv_ && (msg->m_header.sender_id == tracked_erv_.vehicle_id) && (msg->m_header.recipient_id == config_.vehicle_id)){
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "EmergencyVehicleAck received from ERV " << tracked_erv_.vehicle_id);

      // Reset counter and boolean flag to stop warning messages from being broadcasted
      num_warnings_broadcasted_ = 0;
      should_broadcast_warnings_ = false;
    }
  }


  lanelet::Optional<lanelet::routing::Route> ApproachingEmergencyVehiclePlugin::generateErvRoute(double current_latitude, double current_longitude, 
                                                                             std::vector<carma_v2x_msgs::msg::Position3D> erv_destination_points){
    // Check if the map projection is available
    if (!map_projector_) {
      throw std::invalid_argument("Attempting to generate an ERV's route before map projection was set");
    }
      
    // Build map projector
    lanelet::projection::LocalFrameProjector projector(map_projector_.get().c_str());

    // Create vector to hold ERV's destination points
    std::vector<lanelet::BasicPoint3d> erv_destination_points_projected;

    // Add ERV's current location to the beginning of erv_destination_points
    lanelet::GPSPoint current_erv_location;

    current_erv_location.lat = current_latitude;
    current_erv_location.lon = current_longitude;

   

    // Add ERV's future destination points to erv_destination_points
    if(erv_destination_points.size() > 0){
      for(size_t i = 0; i < erv_destination_points.size(); ++i){
        carma_v2x_msgs::msg::Position3D position_3d_point = erv_destination_points[i];
        
        lanelet::GPSPoint erv_destination_point;
        erv_destination_point.lon = position_3d_point.longitude;
        erv_destination_point.lat = position_3d_point.latitude;

        if(position_3d_point.elevation_exists){
          erv_destination_point.ele = position_3d_point.elevation;
        }

        erv_destination_points_projected.emplace_back(projector.forward(erv_destination_point));
      }
    }

    // Convert ERV destination points to map frame
    auto erv_destination_points_in_map = lanelet::utils::transform(erv_destination_points_projected, [](auto a) { return lanelet::traits::to2D(a); });

    auto cmv_location = lanelet::traits::to2D(projector.forward(current_erv_location));
    auto shortened_erv_destination_points_in_map = filter_points_ahead(cmv_location, erv_destination_points_in_map);

    if(shortened_erv_destination_points_in_map.empty())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "ERV has passed all the destination points!");

        // Return empty route
        return lanelet::Optional<lanelet::routing::Route>();
    }

    // Verify that ERV destination points are geometrically in the map
    for(size_t i = 0; i < shortened_erv_destination_points_in_map.size(); ++i){
      auto pt = shortened_erv_destination_points_in_map[i];

      // Return empty route if a destination point is not contained within the map
      if((wm_->getLaneletsFromPoint(shortened_erv_destination_points_in_map[i])).empty()){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "ERV destination point " << i 
                << " is not contained in a lanelet map; x: " << pt.x() << " y: " << pt.y());

        return lanelet::Optional<lanelet::routing::Route>();
      }
    }

    // Obtain ERV's starting lanelet
    auto starting_lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, cmv_location, 1);
    if(starting_lanelet_vector.empty())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Found no lanelets in the map. ERV routing cannot be completed.");

        // Return empty route
        return lanelet::Optional<lanelet::routing::Route>();
    }
    auto starting_lanelet = lanelet::ConstLanelet(starting_lanelet_vector[0].second.constData());

    // Obtain ERV's ending lanelet
    auto ending_lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, shortened_erv_destination_points_in_map.back(), 1);
    auto ending_lanelet = lanelet::ConstLanelet(ending_lanelet_vector[0].second.constData());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "ending_lanelet: " << ending_lanelet.id());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "starting_lanelet: " << starting_lanelet.id());

    // Obtain ERV's via lanelets
    std::vector<lanelet::BasicPoint2d> via = std::vector<lanelet::BasicPoint2d>(shortened_erv_destination_points_in_map.begin(), shortened_erv_destination_points_in_map.end() - 1);
    lanelet::ConstLanelets via_lanelets_vector;
    for(const auto& point : via){
      auto lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, point, 1);
      auto chosen_lanelet_to_emplace = lanelet::ConstLanelet(lanelet_vector[0].second.constData());
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "chosen_lanelet_to_emplace: " << chosen_lanelet_to_emplace.id());

      if (chosen_lanelet_to_emplace.id() != starting_lanelet.id())  // if id is same, it fails to route
        via_lanelets_vector.emplace_back(chosen_lanelet_to_emplace);
      else
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "======> id was found same: " << chosen_lanelet_to_emplace.id());
      }
    }

    // Generate the ERV's route
    auto erv_route = wm_->getMapRoutingGraph()->getRouteVia(starting_lanelet, via_lanelets_vector, ending_lanelet);

    return erv_route;
  }

  std::vector<lanelet::BasicPoint2d> ApproachingEmergencyVehiclePlugin::filter_points_ahead(const lanelet::BasicPoint2d& reference_point, const std::vector<lanelet::BasicPoint2d>& original_points) const
  {
    if (original_points.size() <= 1)
    {
      return original_points;
    }
    
    // extend the list by extrapolating last two points
    auto extended_points = original_points;
    double last_dx = (original_points.end() - 1 ) ->x() - (original_points.end() - 2 ) ->x();
    double last_dy = (original_points.end() - 1 ) ->y() - (original_points.end() - 2 ) ->y();
    lanelet::BasicPoint2d extended_point = {(original_points.end() - 1 ) ->x() + last_dx, (original_points.end() - 1 ) ->y() + last_dy};
    extended_points.push_back(extended_point);

    size_t i = 0;
    size_t closest_idx;
    double closest_dist = DBL_MAX;
    while (i < extended_points.size() - 1)
    {
      // Calculate vectors
      double v1x = reference_point.x() - extended_points[i].x();

      double v1y = reference_point.y() - extended_points[i].y();
      double v2x = extended_points[i+1].x() - extended_points[i].x();
      double v2y = extended_points[i+1].y() - extended_points[i].y();

      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "reference_point x: " << reference_point.x() << ", y: " << reference_point.y());
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "extended_points x: " << extended_points[i].x() << ", y: " << extended_points[i].y());


      // Calculate dot product
      double dotProduct = v1x * v2x + v1y * v2y;

      // Calculate magnitudes
      double v1Mag = sqrt(v1x * v1x + v1y * v1y);
      double v2Mag = sqrt(v2x * v2x + v2y * v2y);

      // Calculate angle in radians
      double angleRad = acos(dotProduct / (v1Mag * v2Mag));

      // Angle between the vectors above 90 degrees means the point i is ahead
      if (angleRad >= M_PI / 2) 
      {
        double dx = reference_point.x() - extended_points[i].x(); 
        double dy = reference_point.y() - extended_points[i].y();
        double distance = sqrt (dx * dx + dy * dy);
        if (distance < closest_dist) // get closest point to the reference point from the multiple possible points 
        {
          closest_dist = distance;
          closest_idx = i;
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "closest_dist: " << closest_dist << ", closest_idx" << closest_idx);

        }
      }
      i ++;
    }

    double dx = reference_point.x() - original_points.back().x(); 
    double dy = reference_point.y() - original_points.back().y();
    double distance = sqrt (dx * dx + dy * dy);

    // last point is still closer to the reference point, all points have passed
    // note that if the optimal point is the last one, this check fails as intended
    if (distance < closest_dist) 
    {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Returning empty here");
      
      return {};
    }

    return std::vector<lanelet::BasicPoint2d>(original_points.begin() + closest_idx, original_points.end());
  }
  void ApproachingEmergencyVehiclePlugin::incomingBsmCallback(carma_v2x_msgs::msg::BSM::UniquePtr msg)
  {
    // Only process incoming BSMs if guidance is currently engaged
    if(!is_guidance_engaged_){
      return;
    }

    // Get the vehicle ID associated with the received BSM
    std::stringstream ss;
    for(size_t i = 0; i < msg->core_data.id.size(); ++i){
      ss << std::setfill('0') << std::setw(2) << std::hex << (unsigned)msg->core_data.id.at(i);
    }
    std::string erv_vehicle_id = ss.str();
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Received a BSM from " << erv_vehicle_id);

    if(has_tracked_erv_){
      // If there is already an ERV approaching the ego vehicle, only process this BSM further if it is from that ERV and enough time has passed since the previously processed BSM

      if(erv_vehicle_id == tracked_erv_.vehicle_id){
        double seconds_since_prev_processed_bsm = (this->now() - tracked_erv_.latest_update_time).seconds();

        if(seconds_since_prev_processed_bsm < (1.0 / config_.bsm_processing_frequency)){
          // Do not process ERV's BSM further since not enough time has passed since its previously processed BSM
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ignoring BSM from tracked ERV " << erv_vehicle_id << " since a BSM from it was processed " << seconds_since_prev_processed_bsm << " seconds ago");
          return;
        }
      }
      else{
        // Do not process BSM further since it is not for the currently tracked ERV
        return;
      }
    }
    else{

      // If BSM is from a detected active ERV, only process it further if enough time has passed since the previously processed BSM from this ERV
      if (latest_erv_update_times_.find(erv_vehicle_id) != latest_erv_update_times_.end()){
        double seconds_since_prev_processed_bsm = (this->now() - latest_erv_update_times_[erv_vehicle_id]).seconds();

        if(seconds_since_prev_processed_bsm < (1.0 / config_.bsm_processing_frequency)){
          // Do not process ERV's BSM further since not enough time has passed since its previously processed BSM
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ignoring BSM from non tracked ERV " << erv_vehicle_id << " since a BSM from it was processed " << seconds_since_prev_processed_bsm << " seconds ago");
          return;
        }
      }
    }

    // Get ErvInformation object with information from the BSM if it is from an active ERV that is approaching the ego vehicle
    boost::optional<ErvInformation> erv_information = getErvInformationFromBsm(std::move(msg));
    if(!erv_information){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "BSM is not from an active ERV that is approaching the ego vehicle.");

        if(has_tracked_erv_){
          RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "BSM was from the currently tracked approaching ERV! ERV is no longer approaching.");
          has_tracked_erv_ = false;
          has_planned_upcoming_lc_ = false;
          transition_table_.event(ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV);
        }

      // BSM is not from an active ERV that is approaching the ego vehicle
      return;
    }
    else{
      // Update tracked ERV information since this is an active ERV that is approaching the ego vehicle
      tracked_erv_ = *erv_information;
      tracked_erv_.latest_update_time = this->now();

      if(!has_tracked_erv_){
        has_tracked_erv_ = true;
      }
    }

    return;
  }

  boost::optional<double> ApproachingEmergencyVehiclePlugin::getSecondsUntilPassing(lanelet::Optional<lanelet::routing::Route>& erv_future_route, const lanelet::BasicPoint2d& erv_position_in_map, 
                                                                   const double& erv_current_speed, lanelet::ConstLanelet& intersecting_lanelet){

    // Obtain ego vehicle and ERV distances to the end of the intersecting lanelet so neither vehicle will currently be past that point
    lanelet::ConstLineString2d intersecting_centerline = lanelet::utils::to2D(intersecting_lanelet.centerline());
    lanelet::BasicPoint2d intersecting_end_point = intersecting_centerline.back();

    // Get ego vehicle's (its rear bumper) distance to the intersecting lanelet's centerline endpoint
    double ego_dist_to_lanelet = wm_->routeTrackPos(intersecting_end_point).downtrack - (latest_route_state_.down_track - config_.vehicle_length);

    // Set erv_world_model_ route to the erv_future_route
    lanelet::routing::Route route = std::move(*erv_future_route);
    carma_wm::LaneletRoutePtr erv_future_route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
    erv_world_model_->setRoute(erv_future_route_ptr);

    // Get ERV's distance to the intersecting lanelet's centerline endpoint
    double erv_dist_to_lanelet = erv_world_model_->routeTrackPos(intersecting_end_point).downtrack - erv_world_model_->routeTrackPos(erv_position_in_map).downtrack;

    if(erv_dist_to_lanelet < ego_dist_to_lanelet){
      // When ERV is in front of the ego vehicle, only process further if the ERV is actively passing the ego vehicle

      if(transition_table_.getState() != ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Detected ERV is in front of the ego vehicle");
        return boost::optional<double>();
      }
      else{
        // This ERV is actively passing the ego vehicle until its distance in front of the ego vehicle is at least config_.finished_passing_threshold
        if((ego_dist_to_lanelet - erv_dist_to_lanelet) < config_.finished_passing_threshold){
          // Return zero to indicate that the ERV is currently passing the ego vehicle
          return 0.0;
        }
        else{
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "ERV has passed the ego vehicle");
          has_tracked_erv_ = false;
          has_planned_upcoming_lc_ = false;
          transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSED);
          return boost::optional<double>();
        }
      }
    }
    else{
      // When ERV is behind the ego vehicle, only process further if the ERV is travelling faster than the ego vehicle

      if(erv_current_speed < current_speed_){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Detected ERV is travelling slower than the ego vehicle, and will not pass the ego vehicle");
        return boost::optional<double>();
      }
      else{
        // Calculate seconds_until_passing and protect against division by zero
        double delta_speed = erv_current_speed - current_speed_;

        if(delta_speed == 0.0){
          delta_speed = epsilon_;
        }
        double seconds_until_passing = (erv_dist_to_lanelet - ego_dist_to_lanelet) / delta_speed;

        return seconds_until_passing;
      }
    }
  }

  boost::optional<lanelet::ConstLanelet> ApproachingEmergencyVehiclePlugin::getRouteIntersectingLanelet(const lanelet::routing::Route& erv_future_route){

    if(future_route_lanelet_ids_.empty()){
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Remaining route lanelets for the ego vehicle not found; plugin cannot compute the intersecting lanelet."); 
      return boost::optional<lanelet::ConstLanelet>();
    }

    // Get current downtrack from latest_route_state_
    double current_downtrack = latest_route_state_.down_track;

    // Find first successful intersecting lanelet between ERV route and ego route with a lanelet starting downtrack greater than current downtrack.
    // Additionally, remove lanelets from future_route_lanelet_ids_ that the ego vehicle has passed.
    for(auto it = future_route_lanelet_ids_.begin(); it != future_route_lanelet_ids_.end();){
      // Get lanelet
      lanelet::Id ego_route_lanelet_id = *it;
      lanelet::ConstLanelet ego_route_lanelet = wm_->getMap()->laneletLayer.get(ego_route_lanelet_id);

      // Get lanelet's centerline end point downtrack
      lanelet::BasicPoint2d ego_route_lanelet_centerline_end = lanelet::utils::to2D(ego_route_lanelet.centerline()).back();
      double ego_route_lanelet_centerline_end_downtrack = wm_->routeTrackPos(ego_route_lanelet_centerline_end).downtrack;

      if(current_downtrack > ego_route_lanelet_centerline_end_downtrack){
        // If current downtrack is greater than lanelet ending downtrack, remove lanelet from future route lanelet and continue
        // NOTE: Iterator is not updated since an element is being erased
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Removing passed lanelet " << ego_route_lanelet_id); 
        future_route_lanelet_ids_.erase(it);
      }
      else{
        // If lanelet exists in both, return it as the intersecting lanelet
        if(erv_future_route.contains(ego_route_lanelet)){
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Found intersecting lanelet " << ego_route_lanelet_id); 
          return boost::optional<lanelet::ConstLanelet>(ego_route_lanelet);
        }
        else{
          // Increase iterator
          ++it;
        }
      }
    }
  }

  void ApproachingEmergencyVehiclePlugin::routeStateCallback(carma_planning_msgs::msg::RouteState::UniquePtr msg)
  {
    latest_route_state_ = *msg;
  }

  void ApproachingEmergencyVehiclePlugin::twistCallback(geometry_msgs::msg::TwistStamped::UniquePtr msg){
    current_speed_ = msg->twist.linear.x;
  }

  void ApproachingEmergencyVehiclePlugin::guidanceStateCallback(carma_planning_msgs::msg::GuidanceState::UniquePtr msg){
    if(msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED){
      is_guidance_engaged_ = true;
    }
    else{
      is_guidance_engaged_ = false;
    }
  }

  void ApproachingEmergencyVehiclePlugin::routeCallback(carma_planning_msgs::msg::Route::UniquePtr msg){
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Received route callback"); 
    std::vector<int> new_future_route_lanelet_ids;

    for(size_t i = 0; i < msg->route_path_lanelet_ids.size(); ++i){
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "New route lanelet: " << msg->route_path_lanelet_ids[i]); 
      new_future_route_lanelet_ids.push_back(msg->route_path_lanelet_ids[i]);
    }
    
    future_route_lanelet_ids_ = new_future_route_lanelet_ids;
  }

  double ApproachingEmergencyVehiclePlugin::getLaneletSpeedLimit(const lanelet::ConstLanelet& lanelet)
  {
    double speed_limit = config_.default_speed_limit;

    lanelet::Optional<carma_wm::TrafficRulesConstPtr> traffic_rules = wm_->getTrafficRules();
    if (traffic_rules)
    {
      speed_limit =(*traffic_rules)->speedLimit(lanelet).speedLimit.value();
    }
    else
    {
      throw std::invalid_argument("No speed limit could be found since valid traffic rules object could not be built");
    }
    
    return speed_limit;
  }

  rclcpp::Duration ApproachingEmergencyVehiclePlugin::getManeuverDuration(const carma_planning_msgs::msg::Maneuver &maneuver, double epsilon) const
  {
    double maneuver_start_speed = GET_MANEUVER_PROPERTY(maneuver, start_speed);
    double maneuver_end_speed = getManeuverEndSpeed(maneuver);
    double sum_start_and_end_speed = maneuver_start_speed + maneuver_end_speed;

    if(sum_start_and_end_speed < epsilon){
        throw std::invalid_argument("Maneuver start and ending speed is zero");
    }

    rclcpp::Duration maneuver_duration{0,0};
    double maneuver_start_dist = GET_MANEUVER_PROPERTY(maneuver, start_dist);
    double maneuver_end_dist = GET_MANEUVER_PROPERTY(maneuver, end_dist);

    RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name),"maneuver_end_dist: " << maneuver_end_dist << ", maneuver_start_dist: " << maneuver_start_dist << ", sum_start_and_end_speed: " << sum_start_and_end_speed);

    maneuver_duration = rclcpp::Duration((maneuver_end_dist - maneuver_start_dist) / (0.5 * sum_start_and_end_speed) * 1e9);

    return maneuver_duration;
  }

  carma_planning_msgs::msg::Maneuver ApproachingEmergencyVehiclePlugin::composeLaneFollowingManeuverMessage(double start_dist, double end_dist, 
                                          double start_speed, double target_speed, int lanelet_id, rclcpp::Time& start_time) const
  {
    carma_planning_msgs::msg::Maneuver maneuver_msg;

    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_FOLLOWING;
    maneuver_msg.lane_following_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.lane_following_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
    maneuver_msg.lane_following_maneuver.parameters.planning_tactical_plugin = config_.lane_following_plugin;
    maneuver_msg.lane_following_maneuver.parameters.planning_strategic_plugin = strategic_plugin_name_;
    maneuver_msg.lane_following_maneuver.start_dist = start_dist;
    maneuver_msg.lane_following_maneuver.start_speed = start_speed;
    maneuver_msg.lane_following_maneuver.start_time = start_time;
    maneuver_msg.lane_following_maneuver.end_dist = end_dist;
    maneuver_msg.lane_following_maneuver.end_speed = target_speed;
    maneuver_msg.lane_following_maneuver.lane_ids = { std::to_string(lanelet_id) };

    rclcpp::Duration maneuver_duration = getManeuverDuration(maneuver_msg, epsilon_);
    maneuver_msg.lane_following_maneuver.end_time = start_time + maneuver_duration;

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Composed lane follow maneuver for lanelet ID:" << lanelet_id << " with duration " << maneuver_duration.seconds());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start speed: " << start_speed << ", end speed: " << target_speed);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start dist: " << start_dist << ", end dist: " << end_dist);

    return maneuver_msg;
  }

  carma_planning_msgs::msg::Maneuver ApproachingEmergencyVehiclePlugin::composeLaneChangeManeuverMessage(double start_dist, double end_dist, 
                                          double start_speed, double target_speed, lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, rclcpp::Time& start_time) const
  {
    carma_planning_msgs::msg::Maneuver maneuver_msg;
    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::LANE_CHANGE;
    maneuver_msg.lane_change_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.lane_change_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN;
    maneuver_msg.lane_change_maneuver.parameters.planning_tactical_plugin = config_.lane_change_plugin;
    maneuver_msg.lane_change_maneuver.parameters.planning_strategic_plugin = strategic_plugin_name_;
    maneuver_msg.lane_change_maneuver.start_dist = start_dist;
    maneuver_msg.lane_change_maneuver.start_speed = start_speed;
    maneuver_msg.lane_change_maneuver.start_time = start_time;
    maneuver_msg.lane_change_maneuver.end_dist = end_dist;
    maneuver_msg.lane_change_maneuver.end_speed = target_speed;
    maneuver_msg.lane_change_maneuver.starting_lane_id = std::to_string(starting_lane_id);
    maneuver_msg.lane_change_maneuver.ending_lane_id = std::to_string(ending_lane_id);

    rclcpp::Duration maneuver_duration = getManeuverDuration(maneuver_msg, epsilon_);
    maneuver_msg.lane_change_maneuver.end_time = start_time + maneuver_duration;

    // Preserve lane change maneuver ID from previous plan to maintain identical trajectory generation across separate maneuver plans
    if(has_planned_upcoming_lc_){
      maneuver_msg.lane_change_maneuver.parameters.maneuver_id = upcoming_lc_params_.maneuver_id;
    }
    else{
      static auto gen = boost::uuids::random_generator(); // Initialize uuid generator
      maneuver_msg.lane_change_maneuver.parameters.maneuver_id = boost::lexical_cast<std::string>(gen()); // generate uuid and convert to string
    }

    RCLCPP_DEBUG_STREAM(get_logger(),"Creating lane change id: "  << maneuver_msg.lane_change_maneuver.parameters.maneuver_id << "start dist: " << start_dist << " end dist: " << end_dist << " Starting llt: " << starting_lane_id << " Ending llt: " << ending_lane_id);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start speed: " << start_speed << ", end speed: " << target_speed << ", duration: " << maneuver_duration.seconds());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start dist: " << start_dist << ", end dist: " << end_dist);

    return maneuver_msg;
  }

  carma_planning_msgs::msg::Maneuver ApproachingEmergencyVehiclePlugin::composeStopAndWaitManeuverMessage(double start_dist, double end_dist, double start_speed, 
                                                        lanelet::Id starting_lane_id, lanelet::Id ending_lane_id, double stopping_deceleration, rclcpp::Time& start_time) const
  {
    carma_planning_msgs::msg::Maneuver maneuver_msg;
    maneuver_msg.type = carma_planning_msgs::msg::Maneuver::STOP_AND_WAIT;
    maneuver_msg.stop_and_wait_maneuver.parameters.negotiation_type = carma_planning_msgs::msg::ManeuverParameters::NO_NEGOTIATION;
    maneuver_msg.stop_and_wait_maneuver.parameters.presence_vector = carma_planning_msgs::msg::ManeuverParameters::HAS_TACTICAL_PLUGIN
                                                                    | carma_planning_msgs::msg::ManeuverParameters::HAS_FLOAT_META_DATA;
    maneuver_msg.stop_and_wait_maneuver.parameters.planning_tactical_plugin = config_.stop_and_wait_plugin;
    maneuver_msg.stop_and_wait_maneuver.parameters.planning_strategic_plugin = strategic_plugin_name_;
    maneuver_msg.stop_and_wait_maneuver.start_time = start_time;
    maneuver_msg.stop_and_wait_maneuver.start_dist = start_dist;
    maneuver_msg.stop_and_wait_maneuver.start_speed = start_speed;
    maneuver_msg.stop_and_wait_maneuver.end_dist = end_dist;
    maneuver_msg.stop_and_wait_maneuver.starting_lane_id = std::to_string(starting_lane_id);
    maneuver_msg.stop_and_wait_maneuver.ending_lane_id = std::to_string(ending_lane_id);

    rclcpp::Duration maneuver_duration = getManeuverDuration(maneuver_msg, epsilon_);
    maneuver_msg.stop_and_wait_maneuver.end_time = start_time + maneuver_duration;

    // Set the meta-data for the StopAndWait Maneuver to define the buffer in the route end point stopping location
    maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(config_.route_end_point_buffer);
    maneuver_msg.stop_and_wait_maneuver.parameters.float_valued_meta_data.push_back(stopping_deceleration);

    static auto gen = boost::uuids::random_generator(); // Initialize uuid generator
    maneuver_msg.stop_and_wait_maneuver.parameters.maneuver_id = boost::lexical_cast<std::string>(gen()); // generate uuid and convert to string

    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Composed stop and wait maneuver for with start lanelet:" << starting_lane_id << ", end lanelet: " << ending_lane_id << " with duration " << maneuver_duration.seconds());
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start speed: " << start_speed);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "start dist: " << start_dist << ", end dist: " << end_dist);

    return maneuver_msg;
  }

  void ApproachingEmergencyVehiclePlugin::addStopAndWaitToEndOfPlan(carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp, 
                          double downtrack_progress, double stop_maneuver_beginning_downtrack, double end_of_route_downtrack, 
                          double stopping_entry_speed, double stopping_deceleration, double current_lanelet_ending_downtrack,
                          lanelet::ConstLanelet current_lanelet, rclcpp::Time time_progress)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Maneuver plan is reaching the end of the route");

    // Add a lane follow maneuver before the stop and wait maneuver if the distance before stop_maneuver_beginning_downtrack is too large
    if((stop_maneuver_beginning_downtrack - downtrack_progress) >= config_.buffer_distance_before_stopping){
      // Compose lane follow maneuver and update downtrack_progress
      resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, stop_maneuver_beginning_downtrack,  
                            stopping_entry_speed, stopping_entry_speed, current_lanelet.id(), time_progress));
      
      downtrack_progress = stop_maneuver_beginning_downtrack;
      time_progress += getManeuverDuration(resp->new_plan.maneuvers.back(), epsilon_);
    }

    // Get the starting lanelet ID for the stop and wait maneuver
    lanelet::Id start_lane_id = current_lanelet.id();

    // Identify the ending lanelet for the stop and wait maneuver
    while(current_lanelet_ending_downtrack < end_of_route_downtrack){
      // Get the next lanelet in the current lane if it exists
      if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty()){
        current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front();
      }
      else{
        RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "A following lanelet in the current lane could not be found; returning empty plan");
        resp->new_plan.maneuvers = {};
        return;
      }          

      // Check whether the next lanelet is on the route
      if(!wm_->getRoute()->contains(current_lanelet)){
        RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "The next lanelet in the current lane is not on the route; returning empty plan");
        resp->new_plan.maneuvers = {};
        return;
      }

      // Break from loop if lanelet's ending downtrack is after the route end downtrack; this is the maneuver's ending lanelet 
      current_lanelet_ending_downtrack = wm_->routeTrackPos(current_lanelet.centerline2d().back()).downtrack;
      if(current_lanelet_ending_downtrack >= end_of_route_downtrack){
        break;
      }
    }

    // Add stop and wait maneuver to maneuver plan
    resp->new_plan.maneuvers.push_back(composeStopAndWaitManeuverMessage(downtrack_progress, end_of_route_downtrack,
                  stopping_entry_speed, start_lane_id, current_lanelet.id(), stopping_deceleration, time_progress));
  }

  void ApproachingEmergencyVehiclePlugin::generateReducedSpeedLaneFollowingeManeuverPlan(
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp,
    lanelet::ConstLanelet current_lanelet, double downtrack_progress, double current_lanelet_ending_downtrack,
    double speed_progress, double target_speed, rclcpp::Time time_progress, bool is_maintaining_non_reduced_speed)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Generating remain-in-lane maneuver plan");

    // Reset planned lane change flag since plugin will not be planning for an upcoming lane change
    has_planned_upcoming_lc_ = false;

    // Update maneuver target speed to be a speed being maintained or a reduced speed below the speed limit
    if(is_maintaining_non_reduced_speed){
      // ERV is close enough that if the ego vehicle slows down it could cause a safety hazard; set target speed to maintain a non-reduced speed
      target_speed = non_reduced_speed_to_maintain_;
    }
    else{
      // Reduce target_speed since this method is triggered when an ERV is approaching the ego vehicle
      target_speed = std::max((target_speed - config_.speed_limit_reduction_during_passing), config_.minimum_reduced_speed_limit);
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Maneuver target speed reduced to " << target_speed);
    }

    double maneuver_plan_ending_downtrack = downtrack_progress + config_.minimal_plan_duration * target_speed;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ending downtrack based on plan duration: " << maneuver_plan_ending_downtrack);

    maneuver_plan_ending_downtrack = std::min(maneuver_plan_ending_downtrack, wm_->getRouteEndTrackPos().downtrack);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ending downtrack based on end of route: " << wm_->getRouteEndTrackPos().downtrack);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Selected ending downtrack: " << maneuver_plan_ending_downtrack);

    // Compute target deceleration for stopping
    double stopping_deceleration = config_.vehicle_acceleration_limit * config_.stopping_accel_limit_multiplier;

    // Compute stopping distance where v_f = 0
    // (v_f^2 - v_i^2) / (2*a) = d 
    double stopping_distance = (target_speed * target_speed) / (2.0 * stopping_deceleration);
    double begin_stopping_downtrack = wm_->getRouteEndTrackPos().downtrack - stopping_distance;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Stop and wait maneuver must begin by downtrack " << begin_stopping_downtrack);

    // Generate maneuver plan
    while(downtrack_progress < maneuver_plan_ending_downtrack){

      if(begin_stopping_downtrack <= current_lanelet_ending_downtrack){
        // Complete maneuver plan with a stop and wait maneuver if the current lanelet intercepts the stopping downtrack
        addStopAndWaitToEndOfPlan(resp, downtrack_progress, begin_stopping_downtrack, wm_->getRouteEndTrackPos().downtrack, 
                        speed_progress, stopping_deceleration, current_lanelet_ending_downtrack, current_lanelet, time_progress);
        return;
      }
      else{
        // Compose lane following maneuver and add it to the response's maneuver plan
        resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                speed_progress, target_speed, current_lanelet.id(), time_progress));

        // Get the next lanelet in the current lane if it exists
        if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
        {
          current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front();
        }
        else
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "A following lanelet in the current lane could not be found; returning empty plan");
          resp->new_plan.maneuvers = {};
          return;
        }

        // Check whether the next lanelet is on the route
        if(!wm_->getRoute()->contains(current_lanelet)){
          RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "The next lanelet in the current lane is not on the route; returning empty plan");
          resp->new_plan.maneuvers = {};
          return;
        }

        // Update the lane follow maneuver parameters for the next maneuver in the plan
        downtrack_progress = wm_->routeTrackPos(current_lanelet.centerline2d().front()).downtrack;
        current_lanelet_ending_downtrack = wm_->routeTrackPos(current_lanelet.centerline2d().back()).downtrack;
        speed_progress = getManeuverEndSpeed(resp->new_plan.maneuvers.back());
        target_speed = getLaneletSpeedLimit(current_lanelet);

        // Update maneuver target speed to be a speed being maintained or a reduced speed below the speed limit
        if(is_maintaining_non_reduced_speed){
          // ERV is close enough that if the ego vehicle slows down it could cause a safety hazard; set target speed to maintain a non-reduced speed
          target_speed = non_reduced_speed_to_maintain_;
        }
        else{
          // Reduce target_speed if an ERV is actively passing the ego vehicle
          target_speed = std::max((target_speed - config_.speed_limit_reduction_during_passing), config_.minimum_reduced_speed_limit);
          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Maneuver target speed reduced to " << target_speed);
        }

        time_progress += getManeuverDuration(resp->new_plan.maneuvers.back(), epsilon_);

        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Next maneuver starting downtrack is " << downtrack_progress << ", end of plan is at " << maneuver_plan_ending_downtrack);
      }
    }
  }

  void ApproachingEmergencyVehiclePlugin::generateMoveOverManeuverPlan(
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp,
    lanelet::ConstLanelet current_lanelet, double downtrack_progress, double current_lanelet_ending_downtrack,
    double speed_progress, double target_speed, rclcpp::Time time_progress, int ego_lane_index, int erv_lane_index)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Generating move-over maneuver plan");

    double maneuver_plan_ending_downtrack = downtrack_progress + config_.minimal_plan_duration * target_speed;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ending downtrack based on plan duration: " << maneuver_plan_ending_downtrack);

    maneuver_plan_ending_downtrack = std::min(maneuver_plan_ending_downtrack, wm_->getRouteEndTrackPos().downtrack);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ending downtrack based on end of route: " << wm_->getRouteEndTrackPos().downtrack);
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Selected ending downtrack: " << maneuver_plan_ending_downtrack);

    // Compute target deceleration for stopping
    double stopping_deceleration = config_.vehicle_acceleration_limit * config_.stopping_accel_limit_multiplier;

    // Compute stopping distance where v_f = 0
    // (v_f^2 - v_i^2) / (2*a) = d 
    double stopping_distance = (target_speed * target_speed) / (2.0 * stopping_deceleration);
    double begin_stopping_downtrack = wm_->getRouteEndTrackPos().downtrack - stopping_distance;
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Stop and wait maneuver must begin by downtrack " << begin_stopping_downtrack);

    // Generate maneuver plan when there is already a planned upcoming lane change
    if(has_planned_upcoming_lc_){
      while(downtrack_progress < maneuver_plan_ending_downtrack){
        if(begin_stopping_downtrack <= current_lanelet_ending_downtrack){
          // Complete maneuver plan with a stop and wait maneuver if the current lanelet intercepts the stopping downtrack
          addStopAndWaitToEndOfPlan(resp, downtrack_progress, begin_stopping_downtrack, wm_->getRouteEndTrackPos().downtrack, 
                          speed_progress, stopping_deceleration, current_lanelet_ending_downtrack, current_lanelet, time_progress);
          return;
        }
        else{
          // Plan lane change maneuver if current downtrack progress is between the starting and ending downtracks of the planned upcoming lane change, otherwise plan lane follow maneuver
          if(((upcoming_lc_params_.start_dist - epsilon_) <= downtrack_progress) && (downtrack_progress < (upcoming_lc_params_.end_dist - epsilon_))){
            resp->new_plan.maneuvers.push_back(composeLaneChangeManeuverMessage(upcoming_lc_params_.start_dist, upcoming_lc_params_.end_dist,  
                                      upcoming_lc_params_.start_speed, upcoming_lc_params_.end_speed, upcoming_lc_params_.starting_lanelet.id(), upcoming_lc_params_.ending_lanelet.id(), time_progress));
            
            current_lanelet = upcoming_lc_params_.ending_lanelet;
          }
          else{
            resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                      speed_progress, target_speed, current_lanelet.id(), time_progress));
          }          
        
          // Get the next lanelet in the current lane if it exists
          if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
          {
            current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front();
          }
          else
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "A following lanelet in the current lane could not be found; returning empty plan");
            resp->new_plan.maneuvers = {};
            return;
          }

          // Check whether the next lanelet is on the route
          if(!wm_->getRoute()->contains(current_lanelet)){
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "The next lanelet in the current lane is not on the route; returning empty plan");
            resp->new_plan.maneuvers = {};
            return;
          }

          // Update the maneuver parameters for the next maneuver in the plan
          downtrack_progress = wm_->routeTrackPos(current_lanelet.centerline2d().front()).downtrack;
          current_lanelet_ending_downtrack = wm_->routeTrackPos(current_lanelet.centerline2d().back()).downtrack;
          speed_progress = getManeuverEndSpeed(resp->new_plan.maneuvers.back());
          target_speed = getLaneletSpeedLimit(current_lanelet);
          time_progress += getManeuverDuration(resp->new_plan.maneuvers.back(), epsilon_);

          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Next maneuver starting downtrack is " << downtrack_progress << ", end of plan is at " << maneuver_plan_ending_downtrack);
        }
      }
    }
    else{
      bool completed_initial_lane_following = false; // Flag to indicate whether sufficient lane following has been planned before the lane change
      double initial_lane_following_duration = 0.0; // Object to store the duration of lane following planned before the lane change

      while(downtrack_progress < maneuver_plan_ending_downtrack){

        if(begin_stopping_downtrack <= current_lanelet_ending_downtrack){
          // Complete maneuver plan with a stop and wait maneuver if the current lanelet intercepts the stopping downtrack
          addStopAndWaitToEndOfPlan(resp, downtrack_progress, begin_stopping_downtrack, wm_->getRouteEndTrackPos().downtrack, 
                          speed_progress, stopping_deceleration, current_lanelet_ending_downtrack, current_lanelet, time_progress);
          return;
        }
        else{
          // First maneuver(s) are lane-following to enable sufficient time to activate turn signals before conducting lane change
          if(!completed_initial_lane_following){
            resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                    speed_progress, target_speed, current_lanelet.id(), time_progress));

            initial_lane_following_duration += getManeuverDuration(resp->new_plan.maneuvers.back(), epsilon_).seconds();
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Seconds of lane following before lane change: " << initial_lane_following_duration);

            if(initial_lane_following_duration >= config_.min_lane_following_duration_before_lane_change){
              RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Minimum lane following duration before lane change satisfied.");
              completed_initial_lane_following = true;
            }
          }
          else{

            if(!has_planned_upcoming_lc_){
              // Initialize UpcomingLaneChangeParameters object to store information regarding the planned lane change maneuver
              UpcomingLaneChangeParameters new_lc_params;

              // Determine if left lane change or right lane change required, and get the target lanelet
              lanelet::Optional<lanelet::ConstLanelet> target_lanelet;
              if(ego_lane_index != erv_lane_index){
                RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Planning a move-over maneuver plan when the ego vehicle is not in the ERV's path! Returning an empty maneuver plan");
                resp->new_plan.maneuvers = {};
                return;              
              }
              else if((ego_lane_index == 0) && (erv_lane_index == 0)){
                // Ego vehicle and ERV are both in the rightmost lane so a left lane change is required
                new_lc_params.is_right_lane_change = false;
                target_lanelet = wm_->getMapRoutingGraph()->left(current_lanelet);   
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Composing a left lane change maneuver from lanelet " << current_lanelet.id());
              }
              else{
                // A right lane change is required
                new_lc_params.is_right_lane_change = true;
                target_lanelet = wm_->getMapRoutingGraph()->right(current_lanelet);
                RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Composing a right lane change maneuver from lanelet " << current_lanelet.id());
              }

              // Only compose lane change maneuver if the target lanelet exists, otherwise compose lane follow maneuver
              if(target_lanelet){
                resp->new_plan.maneuvers.push_back(composeLaneChangeManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                        speed_progress, target_speed, current_lanelet.id(), target_lanelet.get().id(), time_progress));
                
                has_planned_upcoming_lc_ = true;
                
                new_lc_params.starting_lanelet = current_lanelet;
                new_lc_params.ending_lanelet = target_lanelet.get();
                new_lc_params.start_dist = downtrack_progress;
                new_lc_params.end_dist = current_lanelet_ending_downtrack;
                new_lc_params.start_speed = speed_progress;
                new_lc_params.end_speed = target_speed;
                new_lc_params.maneuver_id = resp->new_plan.maneuvers.back().lane_change_maneuver.parameters.maneuver_id;
                upcoming_lc_params_ = new_lc_params;

                current_lanelet = *target_lanelet;
              }
              else{
                RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "A lane change maneuver from " << current_lanelet.id() << " could not be composed since no target lanelet was found!");
                resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                        speed_progress, target_speed, current_lanelet.id(), time_progress));
              }
            }
            else{
              resp->new_plan.maneuvers.push_back(composeLaneFollowingManeuverMessage(downtrack_progress, current_lanelet_ending_downtrack,  
                                        speed_progress, target_speed, current_lanelet.id(), time_progress));
            }
          }

          // Get the next lanelet in the current lane if it exists
          if(!wm_->getMapRoutingGraph()->following(current_lanelet, false).empty())
          {
            current_lanelet = wm_->getMapRoutingGraph()->following(current_lanelet, false).front();
          }
          else
          {
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "A following lanelet in the current lane could not be found; returning empty plan");
            resp->new_plan.maneuvers = {};
            return;
          }

          // Check whether the next lanelet is on the route
          if(!wm_->getRoute()->contains(current_lanelet)){
            RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "The next lanelet in the current lane is not on the route; returning empty plan");
            resp->new_plan.maneuvers = {};
            return;
          }

          // Update the maneuver parameters for the next maneuver in the plan
          downtrack_progress = wm_->routeTrackPos(current_lanelet.centerline2d().front()).downtrack;
          current_lanelet_ending_downtrack = wm_->routeTrackPos(current_lanelet.centerline2d().back()).downtrack;
          speed_progress = getManeuverEndSpeed(resp->new_plan.maneuvers.back());
          target_speed = getLaneletSpeedLimit(current_lanelet);
          time_progress += getManeuverDuration(resp->new_plan.maneuvers.back(), epsilon_);

          RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Next maneuver starting downtrack is " << downtrack_progress << ", end of plan is at " << maneuver_plan_ending_downtrack);
        }
      }    
    }
  }

  boost::optional<lanelet::ConstLanelet> ApproachingEmergencyVehiclePlugin::getLaneletOnEgoRouteFromMapPosition(const double& x_position, const double& y_position)
  {
    // Get lanelet(s) that the provided map coordinates are contained within
    std::vector<lanelet::ConstLanelet> ego_route_lanelets = wm_->getLaneletsFromPoint({x_position, y_position});
    boost::optional<lanelet::ConstLanelet> ego_route_lanelet;
    
    if (ego_route_lanelets.empty())
    {
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Given vehicle position is not on the road.");
      return boost::optional<lanelet::ConstLanelet>();
    }

    // Get the lanelet that is on the ego vehicle's route in case vehicle is located on overlapping lanelets
    for(const auto& lanelet : ego_route_lanelets)
    {
      if(wm_->getRoute()->contains(lanelet)){
        ego_route_lanelet = lanelet;
        break;
      }
    }

    return ego_route_lanelet;
  }

  void ApproachingEmergencyVehiclePlugin::plan_maneuvers_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
  {
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Received request to plan maneuvers");

    // Check whether ego vehicle is currently in the middle of a lane change, or whether its planned lane change has been completed
    bool is_currently_lane_changing = false;
    if(has_planned_upcoming_lc_){
      if((upcoming_lc_params_.start_dist < req->veh_downtrack) && (req->veh_downtrack < upcoming_lc_params_.end_dist)){
        is_currently_lane_changing = true;
      }
      else if(req->veh_downtrack > upcoming_lc_params_.end_dist){
        // Update flag since the planned lane change has been completed
        has_planned_upcoming_lc_ = false;
      }
    }

    boost::optional<lanelet::ConstLanelet> ego_current_lanelet_optional = getLaneletOnEgoRouteFromMapPosition(req->veh_x, req->veh_y);

    if(ego_current_lanelet_optional){
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "starting lanelet for maneuver plan is " << ego_current_lanelet_optional.get().id());
    }
    else{
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Given vehicle position is not within a lanelet on the route. Returning empty maneuver plan");
      return;
    }

    // Get ego vehicle's current lane index
    // Note: For 'lane index', 0 is rightmost lane, 1 is second rightmost, etc.; Only the current travel direction is considered
    ego_lane_index_ = wm_->getMapRoutingGraph()->rights(ego_current_lanelet_optional.get()).size();

    // Update state machine if there is currently an ERV being tracked and if ego vehicle is not currently in the middle of a lane change
    if(has_tracked_erv_ && !is_currently_lane_changing){
      if(ego_lane_index_ == tracked_erv_.lane_index){
        // Trigger state machine transition for case in which the ego vehicle is in the ERV's path
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ego vehicle and ERV are both in lane index " << ego_lane_index_);

        if(tracked_erv_.seconds_until_passing >= config_.approaching_threshold){
          transition_table_.event(ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV);
          has_planned_upcoming_lc_ = false;
        }
        else if((config_.passing_threshold <= tracked_erv_.seconds_until_passing) && (tracked_erv_.seconds_until_passing < config_.approaching_threshold)){
          transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_IN_PATH);
        }
        else if((0.0 <= tracked_erv_.seconds_until_passing) && (tracked_erv_.seconds_until_passing < config_.passing_threshold)){
          transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_PASSING_IN_PATH);
          
          if(tracked_erv_.seconds_until_passing >= MAINTAIN_SPEED_THRESHOLD){
            is_maintaining_non_reduced_speed_ = false;
          }
          else{
            // ERV is close enough (in the same lane) that if the ego vehicle slows down it could cause a safety hazard; set internal members for ego vehicle to maintain its current speed
            if(!is_maintaining_non_reduced_speed_){
              is_maintaining_non_reduced_speed_ = true;
              non_reduced_speed_to_maintain_ = req->veh_logitudinal_velocity;
            }
          }

          // Set flag to broadcast EmergencyVehicleResponse warning messages to ERV if they have not already been broadcasted
          if(!has_broadcasted_warning_messages_){
            should_broadcast_warnings_ = true;
            has_broadcasted_warning_messages_ = true;
          }
        }
        else{
          RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Unsupported seconds_until_passing of " << tracked_erv_.seconds_until_passing << ", returning empty maneuver plan");
          resp->new_plan.maneuvers = {};
          return;
        }
      }
      else{
        // Trigger state machine transition for case in which the ego vehicle is not in the ERV's path
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Ego vehicle is not in ERV's path");

        if((tracked_erv_.seconds_until_passing >= config_.approaching_threshold)){
          transition_table_.event(ApproachingEmergencyVehicleEvent::NO_APPROACHING_ERV);
          has_planned_upcoming_lc_ = false;
        }
        else if((0.0 <= tracked_erv_.seconds_until_passing) && (tracked_erv_.seconds_until_passing <= config_.approaching_threshold)){
          transition_table_.event(ApproachingEmergencyVehicleEvent::APPROACHING_ERV_NOT_IN_PATH);
        }
        else{
          RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Unsupported seconds_until_passing of " << tracked_erv_.seconds_until_passing << ", returning empty maneuver plan");
          resp->new_plan.maneuvers = {};
          return;
        }
      }
    }

    // Set initial maneuver plan parameters based on the ego vehicle's initial state
    double downtrack_progress = req->veh_downtrack;
    double current_lanelet_ending_downtrack = wm_->routeTrackPos(ego_current_lanelet_optional.get().centerline2d().back()).downtrack;
    double speed_progress = req->veh_logitudinal_velocity;
    double target_speed = getLaneletSpeedLimit(ego_current_lanelet_optional.get()); 
    rclcpp::Time time_progress = rclcpp::Time(req->header.stamp);

    // Generate maneuver plan based on the current state
    switch (transition_table_.getState())
    {
      case ApproachingEmergencyVehicleState::NO_APPROACHING_ERV:
        resp->new_plan.maneuvers = {};
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "No approaching ERV. Returning empty maneuver plan");
        is_maintaining_non_reduced_speed_ = false; // Reset flag since ego vehicle does not need to maintain a non-reduced speed
        break;

      case ApproachingEmergencyVehicleState::MOVING_OVER_FOR_APPROACHING_ERV:
        is_maintaining_non_reduced_speed_ = false; // Reset flag since ego vehicle does not need to maintain a non-reduced speed

        generateMoveOverManeuverPlan(resp, ego_current_lanelet_optional.get(), downtrack_progress, current_lanelet_ending_downtrack, speed_progress,
                                        target_speed, time_progress, ego_lane_index_, tracked_erv_.lane_index);
        break;

      case ApproachingEmergencyVehicleState::SLOWING_DOWN_FOR_ERV:
        // Generate a maneuver plan consisting of only lane-following maneuvers at a reduced speed since the ERV is actively passing the ego vehicle
        generateReducedSpeedLaneFollowingeManeuverPlan(resp, ego_current_lanelet_optional.get(), downtrack_progress, current_lanelet_ending_downtrack, speed_progress,
                                        target_speed, time_progress, is_maintaining_non_reduced_speed_);
        break;

      default:
        throw std::invalid_argument("Transition table in unsupported state");
    }

    latest_maneuver_plan_ = resp->new_plan;
  }

  bool ApproachingEmergencyVehiclePlugin::get_availability() {
    return true;
  }

  std::string ApproachingEmergencyVehiclePlugin::get_version_id() {
    return "v1.0";
  }

} // approaching_emergency_vehicle_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin)
