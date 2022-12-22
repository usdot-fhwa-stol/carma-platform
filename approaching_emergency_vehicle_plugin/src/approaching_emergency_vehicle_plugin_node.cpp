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
#include "approaching_emergency_vehicle_plugin/approaching_emergency_vehicle_plugin_node.hpp"

namespace approaching_emergency_vehicle_plugin
{
  namespace std_ph = std::placeholders;

  ApproachingEmergencyVehiclePlugin::ApproachingEmergencyVehiclePlugin(const rclcpp::NodeOptions &options)
      : carma_guidance_plugins::StrategicPlugin(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.passing_threshold = declare_parameter<double>("passing_threshold", config_.passing_threshold);
    config_.do_not_move_over_threshold = declare_parameter<double>("do_not_move_over_threshold", config_.do_not_move_over_threshold);
    config_.approaching_threshold = declare_parameter<double>("approaching_threshold", config_.approaching_threshold);
    config_.bsm_processing_frequency = declare_parameter<double>("bsm_processing_frequency", config_.bsm_processing_frequency);
    config_.speed_reduction_during_passing = declare_parameter<double>("speed_reduction_during_passing", config_.speed_reduction_during_passing);
    config_.minimum_reduced_speed = declare_parameter<double>("minimum_reduced_speed", config_.minimum_reduced_speed);
    config_.timeout_check_frequency = declare_parameter<double>("timeout_check_frequency", config_.timeout_check_frequency);
    config_.timeout_duration = declare_parameter<double>("timeout_duration", config_.timeout_duration);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);

    erv_world_model_.reset(new carma_wm::CARMAWorldModel);
  }

  rcl_interfaces::msg::SetParametersResult ApproachingEmergencyVehiclePlugin::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error_1 = update_params<double>({
        {"passing_threshold", config_.passing_threshold},
        {"do_not_move_over_threshold", config_.do_not_move_over_threshold},
        {"approaching_threshold", config_.approaching_threshold},
        {"bsm_processing_frequency", config_.bsm_processing_frequency},
        {"speed_reduction_during_passing", config_.speed_reduction_during_passing},
        {"timeout_check_frequency", config_.timeout_check_frequency},
        {"timeout_duration", config_.timeout_duration},
    }, parameters);

    auto error_2 = update_params<std::string>({{"vehicle_id", config_.vehicle_id}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error_1 && !error_2;

    return result;
  }

  carma_ros2_utils::CallbackReturn ApproachingEmergencyVehiclePlugin::on_configure_plugin()
  {
    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "ApproachingEmergencyVehiclePlugin trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("passing_threshold", config_.passing_threshold);
    get_parameter<double>("do_not_move_over_threshold", config_.do_not_move_over_threshold);
    get_parameter<double>("approaching_threshold", config_.approaching_threshold);
    get_parameter<double>("bsm_processing_frequency", config_.bsm_processing_frequency);
    get_parameter<double>("speed_reduction_during_passing", config_.speed_reduction_during_passing);
    get_parameter<double>("minimum_reduced_speed", config_.minimum_reduced_speed);
    get_parameter<double>("timeout_check_frequency", config_.timeout_check_frequency);
    get_parameter<double>("timeout_duration", config_.timeout_duration);
    get_parameter<std::string>("vehicle_id", config_.vehicle_id);

    RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "ApproachingEmergencyVehiclePlugin Config: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&ApproachingEmergencyVehiclePlugin::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    incoming_bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("incoming_bsm", 10,
                                            std::bind(&ApproachingEmergencyVehiclePlugin::incomingBsmCallback, this, std_ph::_1));

    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 10,
                                            std::bind(&ApproachingEmergencyVehiclePlugin::georeferenceCallback, this, std_ph::_1));

    route_state_sub_ = create_subscription<carma_planning_msgs::msg::RouteState>("route_state", 10,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::routeStateCallback, this, std_ph::_1));

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("current_velocity", 10,
                                          std::bind(&ApproachingEmergencyVehiclePlugin::twistCallback, this, std_ph::_1));

    // Setup publishers
    plugin_discovery_pub_ = create_publisher<carma_planning_msgs::msg::Plugin>("plugin_discovery", 10);

    outgoing_emergency_vehicle_response_pub_ = create_publisher<carma_v2x_msgs::msg::EmergencyVehicleResponse>("outgoing_emergency_vehicle_response", 10);

    upcoming_lane_change_status_pub_ = create_publisher<carma_planning_msgs::msg::UpcomingLaneChangeStatus>("upcoming_lane_change_status", 10);

    // set world model pointer from wm listener
    wml_ = get_world_model_listener();

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

    return CallbackReturn::SUCCESS;
  }

  void ApproachingEmergencyVehiclePlugin::checkForErvTimeout(){
    // Trigger timeout event if a timeout has occurred for the currently tracked ERV
    if(has_tracked_erv_){
      double seconds_since_prev_update = (this->get_clock()->now() - tracked_erv_.latest_update_time).seconds();
      std::cout<<"Seconds since previous update: " << seconds_since_prev_update << '\n';

      if(seconds_since_prev_update >= config_.timeout_duration){
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger(logger_name), "Timeout occurred for ERV " << tracked_erv_.vehicle_id);
        has_tracked_erv_ = false;
        transition_table_.event(ApproachingEmergencyVehicleEvent::ERV_UPDATE_TIMEOUT);
      }
    }
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

    std::cout<<"Vehicle ID is " << erv_information.vehicle_id << '\n';

    // Get timestamp from BSM
    erv_information.latest_bsm_timestamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());

    std::cout<<"Timestamp received\n";

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
      return boost::optional<ErvInformation>();
    }

    std::cout<<"ERV has active lights and sirens\n";

    // Get vehicle's current speed from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::SPEED_AVAILABLE){
      erv_information.current_speed = msg->core_data.speed;
    }
    else{
      // BSM is not a valid ERV BSM since current speed is not included; return an empty object
      return boost::optional<ErvInformation>();
    }

    std::cout<<"ERV speed is " << erv_information.current_speed << "\n";

    // Get vehicle's current latitude from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LATITUDE_AVAILABLE){
      erv_information.current_latitude = msg->core_data.latitude;
    }
    else{
      // BSM is not a valid ERV BSM since current latitude is not included; return an empty object
      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current longitude from the BSM
    if(msg->core_data.presence_vector & carma_v2x_msgs::msg::BSMCoreData::LONGITUDE_AVAILABLE){
      erv_information.current_longitude = msg->core_data.longitude;
    }
    else{
      // BSM is not a valid ERV BSM since current longitude is not included; return an empty object
      return boost::optional<ErvInformation>();
    }

    // Get vehicle's current position in the map frame from the BSM
    boost::optional<lanelet::BasicPoint2d> erv_position_in_map = getErvPositionInMap(erv_information.current_latitude, erv_information.current_longitude);
    
    if(erv_position_in_map){
      erv_information.current_position_in_map = *erv_position_in_map;
    }

    std::cout<<"ERV has current latitude and longitude \n";

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
      // BSM is not a valid ERV BSM since it does not include destination points; return an empty object
      return boost::optional<ErvInformation>();
    }

    std::cout<<"ERV has route destination points \n";

    // Generate ERV's route based on its current position and its destination points
    lanelet::Optional<lanelet::routing::Route> erv_future_route = generateErvRoute(erv_information.current_latitude, erv_information.current_longitude, erv_destination_points);

    if(!erv_future_route){
      // ERV cannot be tracked since its route could not be generated; return an empty object
      return boost::optional<ErvInformation>();
    }

    std::cout<<"ERV has future route \n";

    // Get intersecting lanelet between ERV's future route and ego vehicle's future shortest path
    boost::optional<lanelet::ConstLanelet> intersecting_lanelet = getRouteIntersectingLanelet(erv_future_route.get(), wm_->getRoute()->shortestPath());

    if(intersecting_lanelet){
      erv_information.intersecting_lanelet = *intersecting_lanelet;
    }
    else{
      // No intersecting lanelet between ERV and ego vehicle was found; return an empty object
      return boost::optional<ErvInformation>();
    }

    std::cout<< "Intersecting lanelet: " << (*intersecting_lanelet).id() << "\n";

    // Get the time (seconds) until the ERV passes the ego vehicle
    double seconds_until_passing = getSecondsUntilPassing(erv_future_route, erv_information.current_position_in_map, erv_information.current_speed, erv_information.intersecting_lanelet);

    if(epsilon_ <= seconds_until_passing && seconds_until_passing <= config_.approaching_threshold){
      RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "Detected approaching ERV; passing ego vehicle in " << seconds_until_passing << " seconds");
      erv_information.seconds_until_passing = seconds_until_passing;
    }
    else{
      RCLCPP_INFO_STREAM(rclcpp::get_logger(logger_name), "Detected non-approaching ERV; passing ego vehicle in " << seconds_until_passing << " seconds");

      // ERV will not be tracked since it is not considered to be approaching the ego vehicle; return an empty object
      return boost::optional<ErvInformation>();
    }

    // Determine whether ERV is currently in the rightmost lane
    // Note: For 'lane index', 0 is rightmost lane, 1 is second rightmost, etc.; Only the current travel direction is considered
    if(!erv_future_route->shortestPath().empty()){
      lanelet::ConstLanelet erv_current_lanelet = erv_future_route->shortestPath()[0];
      int lane_index = wm_->getMapRoutingGraph()->rights(erv_current_lanelet).size();

      if(lane_index == 0){
        erv_information.in_rightmost_lane = true;
      }
      else{
        erv_information.in_rightmost_lane = false;
      }
    }

    return erv_information;
  }

  void ApproachingEmergencyVehiclePlugin::georeferenceCallback(const std_msgs::msg::String::UniquePtr msg) 
  {
    // Build projector from proj string
    map_projector_ = msg->data;
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

    erv_destination_points_projected.emplace_back(projector.forward(current_erv_location));

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

    // Verify that ERV destination points are geometrically in the map
    for(size_t i = 0; i < erv_destination_points_in_map.size(); ++i){
      auto pt = erv_destination_points_in_map[i];

      // Return empty route if a destination point is not contained within the map
      if((wm_->getLaneletsFromPoint(erv_destination_points_in_map[i])).empty()){
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "ERV destination point " << i 
                << " is not contained in a lanelet map; x: " << pt.x() << " y: " << pt.y());

        return lanelet::Optional<lanelet::routing::Route>();
      }
    }

    // Obtain ERV's starting lanelet
    auto starting_lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, erv_destination_points_in_map.front(), 1);
    if(starting_lanelet_vector.empty())
    {
        RCLCPP_ERROR_STREAM(rclcpp::get_logger(logger_name), "Found no lanelets in the map. ERV routing cannot be completed.");

        // Return empty route
        return lanelet::Optional<lanelet::routing::Route>();
    }
    auto starting_lanelet = lanelet::ConstLanelet(starting_lanelet_vector[0].second.constData());

    // Obtain ERV's ending lanelet
    auto ending_lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, erv_destination_points_in_map.back(), 1);
    auto ending_lanelet = lanelet::ConstLanelet(ending_lanelet_vector[0].second.constData());

    // Obtain ERV's via lanelets
    std::vector<lanelet::BasicPoint2d> via = std::vector<lanelet::BasicPoint2d>(erv_destination_points_in_map.begin() + 1, erv_destination_points_in_map.end() - 1);
    lanelet::ConstLanelets via_lanelets_vector;
    for(const auto& point : via){
      auto lanelet_vector = lanelet::geometry::findNearest(wm_->getMap()->laneletLayer, point, 1);
      via_lanelets_vector.emplace_back(lanelet::ConstLanelet(lanelet_vector[0].second.constData()));
    }

    // Generate the ERV's route
    auto erv_route = wm_->getMapRoutingGraph()->getRouteVia(starting_lanelet, via_lanelets_vector, ending_lanelet);

    for(auto ll : erv_route->shortestPath()){
      std::cout<<"ERV shortest path lanelet: " << ll.id() << '\n';
    }

    return erv_route;
  }

  void ApproachingEmergencyVehiclePlugin::incomingBsmCallback(carma_v2x_msgs::msg::BSM::UniquePtr msg)
  {

    // If there is already an ERV approaching the ego vehicle, only process this BSM futher if enough time has passed since the previously processed BSM
    if(has_tracked_erv_){

      // Get the vehicle ID associated with the received BSM
      std::stringstream ss;
      for(size_t i = 0; i < msg->core_data.id.size(); ++i){
        ss << std::setfill('0') << std::setw(2) << std::hex << (unsigned)msg->core_data.id.at(i);
      }
      std::string erv_vehicle_id = ss.str();

      if(erv_vehicle_id == tracked_erv_.vehicle_id){
        rclcpp::Time current_bsm_timestamp = rclcpp::Time(msg->header.stamp, this->get_clock()->get_clock_type());
        double seconds_since_prev_bsm = (current_bsm_timestamp - tracked_erv_.latest_bsm_timestamp).seconds();

        if(seconds_since_prev_bsm < (1.0 / config_.bsm_processing_frequency)){
          // Do not process ERV's BSM further since not enough time has passed since its previously processed BSM
          return;
        }
      }
      else{
        // Do not process BSM further since it is not for the currently tracked ERV
        return;
      }
    }

    // Get ErvInformation object with information from the BSM if it is from an active ERV that is approaching the ego vehicle
    boost::optional<ErvInformation> erv_information = getErvInformationFromBsm(std::move(msg));
    if(!erv_information){
      // BSM is not from an active ERV that is approaching the ego vehicle
      return;
    }
    else{
      // Update tracked ERV information since this is an active ERV that is approaching the ego vehicle
      tracked_erv_ = *erv_information;
      tracked_erv_.latest_update_time = this->get_clock()->now();

      if(!has_tracked_erv_){
        has_tracked_erv_ = true;
      }
    }

    return;
  }

  double ApproachingEmergencyVehiclePlugin::getSecondsUntilPassing(lanelet::Optional<lanelet::routing::Route>& erv_future_route, const lanelet::BasicPoint2d& erv_position_in_map, 
                                                                   const double& erv_current_speed, lanelet::ConstLanelet& intersecting_lanelet){

    // Obtain ego vehicle and ERV distances to the end of the intersecting lanelet so neither vehicle will currently be past that point
    lanelet::ConstLineString2d intersecting_centerline = lanelet::utils::to2D(intersecting_lanelet.centerline());
    lanelet::BasicPoint2d intersecting_end_point = intersecting_centerline.back();

    // Get ego vehicle's distance to the beginning of the intersecting lanelet
    // Note: Distance shall be 0.0 if ego vehicle is currently located within the intersecting lanelet
    double ego_dist_to_lanelet = wm_->routeTrackPos(intersecting_end_point).downtrack - latest_route_state_.down_track;

    std::cout<<"Ego vehicle's distance to intersecting lanelet is " << ego_dist_to_lanelet << " meters\n";

    // Set erv_world_model_ route to the erv_future_route
    lanelet::routing::Route route = std::move(*erv_future_route);
    carma_wm::LaneletRoutePtr erv_future_route_ptr = std::make_shared<lanelet::routing::Route>(std::move(route));
    erv_world_model_->setRoute(erv_future_route_ptr);

    // Get downtrack of intersecting lanelet on ERV's route
    double erv_dist_to_lanelet = erv_world_model_->routeTrackPos(intersecting_end_point).downtrack - erv_world_model_->routeTrackPos(erv_position_in_map).downtrack;
    std::cout<<"ERV's distance to intersecting lanelet is " << erv_dist_to_lanelet << " meters\n";

    // Calculate seconds_until_passing and protect against division by zero
    double delta_speed = erv_current_speed - current_speed_;
    if(delta_speed == 0.0){
      delta_speed = epsilon_;
    }
    double seconds_until_passing = (erv_dist_to_lanelet - ego_dist_to_lanelet) / delta_speed;
    std::cout<<"Seconds until ERV passes ego vehicle: " << seconds_until_passing << '\n';

    return seconds_until_passing;
  }

  boost::optional<lanelet::ConstLanelet> ApproachingEmergencyVehiclePlugin::getRouteIntersectingLanelet(const lanelet::routing::Route& erv_future_route, 
                        const lanelet::routing::LaneletPath& ego_shortest_path){

    // Get the ego vehicle's future shortest path lanelets
    double ending_downtrack = wm_->getRouteEndTrackPos().downtrack;
    std::vector<lanelet::ConstLanelet> ego_future_shortest_path = wm_->getLaneletsBetween(latest_route_state_.down_track, ending_downtrack);

    if(ego_future_shortest_path.empty()){
      RCLCPP_WARN_STREAM(rclcpp::get_logger(logger_name), "Remaining shortest path for ego vehicle not found; intersecting lanelet with ERV will not be computed"); 
      return boost::optional<lanelet::ConstLanelet>();
    }

    // Loop through ego vehicle's future shortest path and find first lanelet that exists in ERV's future route
    for(size_t i = 0; i < ego_future_shortest_path.size(); ++i){
      if(erv_future_route.contains(ego_future_shortest_path[i])){
        return boost::optional<lanelet::ConstLanelet>(ego_future_shortest_path[i]);
      }
    }

    // No intersecting lanelet was found, return empty object
    return boost::optional<lanelet::ConstLanelet>();
  }

  void ApproachingEmergencyVehiclePlugin::routeStateCallback(carma_planning_msgs::msg::RouteState::UniquePtr msg)
  {
    latest_route_state_ = *msg;
  }

  void ApproachingEmergencyVehiclePlugin::twistCallback(geometry_msgs::msg::TwistStamped::UniquePtr msg){
    current_speed_ = msg->twist.linear.x;
  }

  void ApproachingEmergencyVehiclePlugin::plan_maneuvers_callback(
    std::shared_ptr<rmw_request_id_t>, 
    carma_planning_msgs::srv::PlanManeuvers::Request::SharedPtr req, 
    carma_planning_msgs::srv::PlanManeuvers::Response::SharedPtr resp)
  {
    // TODO: Implement maneuver-generation logic in future work.
  }

  bool ApproachingEmergencyVehiclePlugin::get_availability() {
    return false;
  }

  std::string ApproachingEmergencyVehiclePlugin::get_version_id() {
    return "v1.0";
  }

} // approaching_emergency_vehicle_plugin

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(approaching_emergency_vehicle_plugin::ApproachingEmergencyVehiclePlugin)
