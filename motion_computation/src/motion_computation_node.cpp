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
#include "motion_computation/motion_computation_node.hpp"

namespace motion_computation
{
  namespace std_ph = std::placeholders;

  MotionComputationNode::MotionComputationNode(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options),
        motion_worker_(
          std::bind(&MotionComputationNode::publishObject, this, std_ph::_1),
          get_node_logging_interface()
        )
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.prediction_time_step = declare_parameter<double>("prediction_time_step", config_.prediction_time_step);
    config_.mobility_path_time_step = declare_parameter<double>("mobility_path_time_step", config_.mobility_path_time_step);
    config_.prediction_period = declare_parameter<double>("prediction_period", config_.prediction_period);
    config_.cv_x_accel_noise = declare_parameter<double>("cv_x_accel_noise", config_.cv_x_accel_noise);
    config_.cv_y_accel_noise = declare_parameter<double>("cv_y_accel_noise", config_.cv_y_accel_noise);
    config_.prediction_process_noise_max = declare_parameter<double>("prediction_process_noise_max", config_.prediction_process_noise_max);
    config_.prediction_confidence_drop_rate = declare_parameter<double>("prediction_confidence_drop_rate", config_.prediction_confidence_drop_rate);
    config_.external_object_prediction_mode = declare_parameter<int>("external_object_prediction_mode", config_.external_object_prediction_mode);
  }

  rcl_interfaces::msg::SetParametersResult MotionComputationNode::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    auto error = update_params<double>({
        {"prediction_time_step", config_.prediction_time_step},
        {"mobility_path_time_step", config_.mobility_path_time_step},
        {"prediction_period", config_.prediction_period},
        {"cv_x_accel_noise", config_.cv_x_accel_noise},
        {"cv_y_accel_noise", config_.cv_y_accel_noise},
        {"prediction_process_noise_max", config_.prediction_process_noise_max},
        {"prediction_confidence_drop_rate", config_.prediction_confidence_drop_rate}
    }, parameters);
    auto error_2 = update_params<int>({{"external_object_prediction_mode", config_.external_object_prediction_mode}}, parameters);
        
    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error && !error_2;

    return result;
  }

  carma_ros2_utils::CallbackReturn MotionComputationNode::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO_STREAM(get_logger(), "MotionComputationNode trying to configure");

    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<double>("prediction_time_step", config_.prediction_time_step);
    get_parameter<double>("mobility_path_time_step", config_.mobility_path_time_step);
    get_parameter<double>("prediction_period", config_.prediction_period);
    get_parameter<double>("cv_x_accel_noise", config_.cv_x_accel_noise);
    get_parameter<double>("cv_y_accel_noise", config_.cv_y_accel_noise);
    get_parameter<double>("prediction_process_noise_max", config_.prediction_process_noise_max);
    get_parameter<double>("prediction_confidence_drop_rate", config_.prediction_confidence_drop_rate);
    get_parameter<int>("external_object_prediction_mode", config_.external_object_prediction_mode);

    RCLCPP_INFO_STREAM(get_logger(), "Loaded params: " << config_);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&MotionComputationNode::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    motion_comp_sub_ = create_subscription<carma_perception_msgs::msg::ExternalObjectList>("external_objects", 1,
                                                              std::bind(&MotionComputationWorker::predictionLogic, &motion_worker_, std_ph::_1));
    mobility_path_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityPath>("incoming_mobility_path", 20,
                                                              std::bind(&MotionComputationWorker::mobilityPathCallback, &motion_worker_, std_ph::_1));
    georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 1,
                                                              std::bind(&MotionComputationWorker::georeferenceCallback, &motion_worker_, std_ph::_1));

    // Setup publishers
    carma_obj_pub_ = create_publisher<carma_perception_msgs::msg::ExternalObjectList>("external_object_predictions", 2);

    // Set motion_worker_'s prediction parameters
    motion_worker_.setPredictionTimeStep(config_.prediction_time_step);
    motion_worker_.setMobilityPathPredictionTimeStep(config_.mobility_path_time_step);
    motion_worker_.setPredictionPeriod(config_.prediction_period);
    motion_worker_.setXAccelerationNoise(config_.cv_x_accel_noise);
    motion_worker_.setYAccelerationNoise(config_.cv_y_accel_noise);
    motion_worker_.setProcessNoiseMax(config_.prediction_process_noise_max);
    motion_worker_.setConfidenceDropRate(config_.prediction_confidence_drop_rate);
    motion_worker_.setExternalObjectPredictionMode(config_.external_object_prediction_mode);

    // Return success if everthing initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void MotionComputationNode::publishObject(const carma_perception_msgs::msg::ExternalObjectList& obj_pred_msg) const
  {
    carma_obj_pub_->publish(obj_pred_msg);
  }

} // namespace motion_computation

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(motion_computation::MotionComputationNode)
