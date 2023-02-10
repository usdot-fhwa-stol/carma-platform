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
#include "bsm_generator/bsm_generator_node.hpp"

namespace bsm_generator
{
  namespace std_ph = std::placeholders;

  BSMGenerator::BSMGenerator(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.bsm_generation_frequency = declare_parameter<double>("bsm_generation_frequency", config_.bsm_generation_frequency);
    config_.bsm_id_change_period = declare_parameter<double>("bsm_id_change_period", config_.bsm_id_change_period);
    config_.bsm_id_rotation_enabled  = declare_parameter<bool>("bsm_id_rotation_enabled", config_.bsm_id_rotation_enabled);
    config_.bsm_message_id           = declare_parameter<int>("bsm_message_id", config_.bsm_message_id);
    config_.vehicle_length           = declare_parameter<double>("vehicle_length", config_.vehicle_length);
    config_.vehicle_width            = declare_parameter<double>("vehicle_width", config_.vehicle_width);
  }

  rcl_interfaces::msg::SetParametersResult BSMGenerator::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<bool>({{"bsm_id_rotation_enabled", config_.bsm_id_rotation_enabled}}, parameters);
    auto error_2 = update_params<int>({{"bsm_message_id", config_.bsm_message_id}}, parameters);
    auto error_3 = update_params<double>({
        {"bsm_generation_frequency", config_.bsm_generation_frequency},
        {"bsm_id_change_period", config_.bsm_id_change_period},
        {"vehicle_length", config_.vehicle_length},
        {"vehicle_width", config_.vehicle_width}
    }, parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2 && !error_3;

    return result;
  }

  carma_ros2_utils::CallbackReturn BSMGenerator::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "BSMGenerator trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("bsm_generation_frequency", config_.bsm_generation_frequency);
    get_parameter<double>("bsm_id_change_period", config_.bsm_id_change_period);
    get_parameter<bool>("bsm_id_rotation_enabled", config_.bsm_id_rotation_enabled);
    get_parameter<int>("bsm_message_id", config_.bsm_message_id);
    get_parameter<double>("vehicle_length", config_.vehicle_length);
    get_parameter<double>("vehicle_width", config_.vehicle_width);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&BSMGenerator::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pose", 1,
                                                              std::bind(&BSMGenerator::poseCallback, this, std_ph::_1));
    accel_sub_ = create_subscription<automotive_platform_msgs::msg::VelocityAccelCov>("velocity_accel_cov", 1,
                                                              std::bind(&BSMGenerator::accelCallback, this, std_ph::_1));
    yaw_sub_ = create_subscription<sensor_msgs::msg::Imu>("imu_raw", 1,
                                                              std::bind(&BSMGenerator::yawCallback, this, std_ph::_1));
    gear_sub_ = create_subscription<j2735_v2x_msgs::msg::TransmissionState>("transmission_state", 1,
                                                              std::bind(&BSMGenerator::gearCallback, this, std_ph::_1));
    speed_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("ekf_twist", 1,
                                                              std::bind(&BSMGenerator::speedCallback, this, std_ph::_1));
    steer_wheel_angle_sub_ = create_subscription<std_msgs::msg::Float64>("steering_wheel_angle", 1,
                                                              std::bind(&BSMGenerator::steerWheelAngleCallback, this, std_ph::_1));
    brake_sub_ = create_subscription<std_msgs::msg::Float64>("brake_position", 1,
                                                              std::bind(&BSMGenerator::brakeCallback, this, std_ph::_1));
    heading_sub_ = create_subscription<gps_msgs::msg::GPSFix>("gnss_fix_fused", 1,
                                                              std::bind(&BSMGenerator::headingCallback, this, std_ph::_1));
    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&BSMGenerator::georeferenceCallback, this, std_ph::_1));

    // Setup publishers
    bsm_pub_ = create_publisher<carma_v2x_msgs::msg::BSM>("bsm_outbound", 5);

    // Initialize the generated BSM message
    initializeBSM();

    worker = std::make_shared<BSMGeneratorWorker>();

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  carma_ros2_utils::CallbackReturn BSMGenerator::handle_on_activate(const rclcpp_lifecycle::State &prev_state)
  {
    // Timer setup for generating a BSM
    int bsm_generation_period_ms = (1 / config_.bsm_generation_frequency) * 1000; // Conversion from frequency (Hz) to milliseconds time period
    timer_ = create_timer(get_clock(),
                          std::chrono::milliseconds(bsm_generation_period_ms),
                          std::bind(&BSMGenerator::generateBSM, this));

    return CallbackReturn::SUCCESS;
  }

  void BSMGenerator::initializeBSM()
  {
    bsm_.core_data.presence_vector = 0;
    bsm_.core_data.size.vehicle_width = config_.vehicle_width;
    bsm_.core_data.size.vehicle_length = config_.vehicle_length;
    bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_LENGTH_AVAILABLE;
    bsm_.core_data.size.presence_vector = bsm_.core_data.size.presence_vector | bsm_.core_data.size.VEHICLE_WIDTH_AVAILABLE;
  }

  void BSMGenerator::georeferenceCallback(const std_msgs::msg::String::UniquePtr msg)
  {
    // Build projector from proj string
    map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());
  }

  void BSMGenerator::speedCallback(const geometry_msgs::msg::TwistStamped::UniquePtr msg)
  {
    bsm_.core_data.speed = worker->getSpeedInRange(msg->twist.linear.x);
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SPEED_AVAILABLE;
  }

  void BSMGenerator::gearCallback(const j2735_v2x_msgs::msg::TransmissionState::UniquePtr msg)
  {
    bsm_.core_data.transmission.transmission_state = msg->transmission_state;
  }

  void BSMGenerator::steerWheelAngleCallback(const std_msgs::msg::Float64::UniquePtr msg)
  {
    bsm_.core_data.angle = worker->getSteerWheelAngleInRange(msg->data);
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.STEER_WHEEL_ANGLE_AVAILABLE;
  }

  void BSMGenerator::accelCallback(const automotive_platform_msgs::msg::VelocityAccelCov::UniquePtr msg)
  {
    bsm_.core_data.accel_set.longitudinal = worker->getLongAccelInRange(msg->accleration);
    bsm_.core_data.accel_set.presence_vector = bsm_.core_data.accel_set.presence_vector | bsm_.core_data.accel_set.ACCELERATION_AVAILABLE;
  }

  void BSMGenerator::yawCallback(const sensor_msgs::msg::Imu::UniquePtr msg)
  {
    bsm_.core_data.accel_set.yaw_rate = worker->getYawRateInRange(static_cast<float>(msg->angular_velocity.z));
    bsm_.core_data.accel_set.presence_vector = bsm_.core_data.accel_set.presence_vector | bsm_.core_data.accel_set.YAWRATE_AVAILABLE;
  }

  void BSMGenerator::brakeCallback(const std_msgs::msg::Float64::UniquePtr msg)
  {
    bsm_.core_data.brakes.wheel_brakes.brake_applied_status = worker->getBrakeAppliedStatus(msg->data);
  }

  void BSMGenerator::poseCallback(const geometry_msgs::msg::PoseStamped::UniquePtr msg)
  {
    // Use pose message as an indicator of new location updates
    if (!map_projector_) {
        RCLCPP_DEBUG_STREAM(get_logger(), "Ignoring pose message as projection string has not been defined");
        return;
    }
       
    lanelet::GPSPoint coord = map_projector_->reverse( { msg->pose.position.x, msg->pose.position.y, msg->pose.position.z } );
     
    bsm_.core_data.longitude = coord.lon;
    bsm_.core_data.latitude = coord.lat;
    bsm_.core_data.elev = coord.ele;
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LONGITUDE_AVAILABLE;
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.LATITUDE_AVAILABLE;
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.ELEVATION_AVAILABLE;
  }

  void BSMGenerator::headingCallback(const gps_msgs::msg::GPSFix::UniquePtr msg)
  {
    bsm_.core_data.heading = worker->getHeadingInRange(static_cast<float>(msg->track));
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.HEADING_AVAILABLE;
  }

  void BSMGenerator::generateBSM()
  {
    bsm_.header.stamp = this->now();
    bsm_.core_data.msg_count = worker->getNextMsgCount();

    if (config_.bsm_id_rotation_enabled)
      bsm_.core_data.id = worker->getMsgId( this->now(), config_.bsm_id_change_period);
    else
    {
      std::vector<uint8_t> id(4);
      for(int i = 0; i < id.size(); ++i)
      {
        id[i] = config_.bsm_message_id >> (8 * i);
      }
      bsm_.core_data.id = id;
    }

    bsm_.core_data.sec_mark = worker->getSecMark( this->now() );
    bsm_.core_data.presence_vector = bsm_.core_data.presence_vector | bsm_.core_data.SEC_MARK_AVAILABLE;
    // currently the accuracy is not available because ndt_matching does not provide accuracy measurement
    bsm_.core_data.accuracy.presence_vector = 0;
    bsm_pub_->publish(bsm_);
  }

} // namespace bsm_generator

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(bsm_generator::BSMGenerator)
