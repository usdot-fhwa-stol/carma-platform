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
#include "mobilitypath_publisher/mobilitypath_publisher.hpp"

namespace mobilitypath_publisher
{
  namespace std_ph = std::placeholders;

  MobilityPathPublication::MobilityPathPublication(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.path_pub_rate = declare_parameter<double>("path_pub_rate", config_.path_pub_rate);
    config_.vehicle_id = declare_parameter<std::string>("vehicle_id", config_.vehicle_id);
  }

  rcl_interfaces::msg::SetParametersResult MobilityPathPublication::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({{"path_pub_rate", config_.path_pub_rate}}, parameters);
    auto error_2 = update_params<std::string>({{"vehicle_id", config_.vehicle_id}}, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    return result;
  }

  carma_ros2_utils::CallbackReturn MobilityPathPublication::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "MobilityPathPublication trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("path_pub_rate", config_.path_pub_rate);
    get_parameter<std::string>("vehicle_id", config_.vehicle_id);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&MobilityPathPublication::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    traj_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("plan_trajectory", 5, 
                                                                               std::bind(&MobilityPathPublication::trajectory_cb, this, std_ph::_1));
    bsm_sub_ = create_subscription<carma_v2x_msgs::msg::BSM>("bsm_outbound", 1,
                                                             std::bind(&MobilityPathPublication::bsm_cb, this, std_ph::_1));
    
    guidance_state_sub_ = create_subscription<carma_planning_msgs::msg::GuidanceState>("guidance_state", 5, std::bind(&MobilityPathPublication::guidance_state_cb, this, std::placeholders::_1));

    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                                   std::bind(&MobilityPathPublication::georeference_cb, this, std_ph::_1));

    // Setup publishers
    path_pub_ = create_publisher<carma_v2x_msgs::msg::MobilityPath>("mobility_path_msg", 5);
    

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void MobilityPathPublication::guidance_state_cb(const carma_planning_msgs::msg::GuidanceState::UniquePtr msg)
  {
    guidance_engaged_ = (msg->state == carma_planning_msgs::msg::GuidanceState::ENGAGED);
  }

  carma_ros2_utils::CallbackReturn MobilityPathPublication::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    // Timer setup
    int path_pub_period_millisecs = (1 / config_.path_pub_rate) * 1000; // Conversion from frequency (Hz) to milliseconds time period
    path_pub_timer_ = create_timer(get_clock(),
                                   std::chrono::milliseconds(path_pub_period_millisecs),
                                   std::bind(&MobilityPathPublication::spin_callback, this));

    return CallbackReturn::SUCCESS;
  }

  bool MobilityPathPublication::spin_callback()
  {
    // update timestamp of mobilitypath
    uint64_t millisecs = get_clock()->now().nanoseconds() / 1000000;
    latest_mobility_path_.m_header.timestamp = millisecs; //time in millisecond
    if (guidance_engaged_)
      path_pub_->publish(latest_mobility_path_);
    return true;
  }

  void MobilityPathPublication::georeference_cb(const std_msgs::msg::String::UniquePtr msg)
  {
    // Build projector from proj string
    map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());
  }

  void MobilityPathPublication::trajectory_cb(const carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
  {
    latest_trajectory_ = *msg;
    latest_mobility_path_ = mobility_path_message_generator(latest_trajectory_);
  }

  void MobilityPathPublication::bsm_cb(const carma_v2x_msgs::msg::BSM::UniquePtr msg)
  {
    bsm_core_ = msg->core_data;
  }

  carma_v2x_msgs::msg::MobilityPath MobilityPathPublication::mobility_path_message_generator(const carma_planning_msgs::msg::TrajectoryPlan& trajectory_plan)
  {
    carma_v2x_msgs::msg::MobilityPath mobility_path_msg;
    // TODO this caluclation uses a poor assumption of zero latency see https://github.com/usdot-fhwa-stol/carma-platform/issues/1606
    uint64_t millisecs = get_clock()->now().nanoseconds() / 1000000;
    mobility_path_msg.m_header = compose_mobility_header(millisecs);
        
    if (!map_projector_) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "MobilityPath cannot be populated as map projection is not available");
      return mobility_path_msg;
    }

    carma_v2x_msgs::msg::Trajectory mob_path_traj = trajectory_plan_to_trajectory(trajectory_plan.trajectory_points);
    mobility_path_msg.trajectory = mob_path_traj;

    return mobility_path_msg;
  }

  carma_v2x_msgs::msg::MobilityHeader MobilityPathPublication::compose_mobility_header(uint64_t time)
  {
    carma_v2x_msgs::msg::MobilityHeader header;
    header.sender_id = config_.vehicle_id;
    header.recipient_id = recipient_id;
    header.sender_bsm_id = BSMHelper::BSMHelper::bsmIDtoString(bsm_core_.id);

    // random GUID that identifies this particular plan for future reference
    header.plan_id = boost::uuids::to_string(boost::uuids::random_generator()());
    header.timestamp = time; //time in millisecond
        
    return header;
  }

  carma_v2x_msgs::msg::Trajectory MobilityPathPublication::trajectory_plan_to_trajectory(const std::vector<carma_planning_msgs::msg::TrajectoryPlanPoint>& traj_points) const
  {
    carma_v2x_msgs::msg::Trajectory traj;

    if (traj_points.empty()) {
      throw std::invalid_argument("Received an empty vector of Trajectory Plan Points");
    }

    carma_v2x_msgs::msg::LocationECEF ecef_location = trajectory_point_to_ECEF(traj_points[0]); //m to cm to fit the msg standard 

    if (traj_points.size() < 2){
      RCLCPP_WARN_STREAM(this->get_logger(), "Received Trajectory Plan is too small");
      traj.offsets = {};
    }
    else{
      carma_v2x_msgs::msg::LocationECEF prev_point = ecef_location;
      for (size_t i=1; i<traj_points.size(); i++){
                
        carma_v2x_msgs::msg::LocationOffsetECEF offset;
        carma_v2x_msgs::msg::LocationECEF new_point = trajectory_point_to_ECEF(traj_points[i]); //m to cm to fit the msg standard
        offset.offset_x = (int16_t)(new_point.ecef_x - prev_point.ecef_x);  
        offset.offset_y = (int16_t)(new_point.ecef_y - prev_point.ecef_y);
        offset.offset_z = (int16_t)(new_point.ecef_z - prev_point.ecef_z);
        prev_point = new_point;
        traj.offsets.push_back(offset);
        if( i >= 60 ){ break;};
      }
    }

    traj.location = ecef_location; 

    return traj;    
  }

  carma_v2x_msgs::msg::LocationECEF MobilityPathPublication::trajectory_point_to_ECEF(const carma_planning_msgs::msg::TrajectoryPlanPoint& traj_point) const
  {
    if (!map_projector_) {
      throw std::invalid_argument("No map projector available for ecef conversion");
    }
    carma_v2x_msgs::msg::LocationECEF location;    
        
    lanelet::BasicPoint3d ecef_point = map_projector_->projectECEF({traj_point.x, traj_point.y, 0.0}, 1);
    location.ecef_x = ecef_point.x() * 100.0;
    location.ecef_y = ecef_point.y() * 100.0;
    location.ecef_z = ecef_point.z() * 100.0;

    return location;
  }

} // mobilitypath_publisher

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(mobilitypath_publisher::MobilityPathPublication)
