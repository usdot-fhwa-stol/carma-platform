// Copyright 2019-2023 Leidos
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOTION_COMPUTATION__MOTION_COMPUTATION_NODE_HPP_
#define MOTION_COMPUTATION__MOTION_COMPUTATION_NODE_HPP_

#include <lanelet2_extension/projection/local_frame_projector.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <carma_perception_msgs/msg/external_object_list.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <functional>
#include <vector>

#include "motion_computation/motion_computation_config.hpp"
#include "motion_computation/motion_computation_worker.hpp"

namespace motion_computation
{

/**
 * \class MotionComputationNode
 * \brief The class responsible for publishing external object predictions
 */
class MotionComputationNode : public carma_ros2_utils::CarmaLifecycleNode
{
private:
  // Subscribers
  carma_ros2_utils::SubPtr<carma_perception_msgs::msg::ExternalObjectList> motion_comp_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityPath> mobility_path_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::BSM> bsm_sub_;
  carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::PSM> psm_sub_;
  carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;

  // Publishers
  carma_ros2_utils::PubPtr<carma_perception_msgs::msg::ExternalObjectList> carma_obj_pub_;

  // MotionComputationWorker class object
  MotionComputationWorker motion_worker_;

  // Node configuration
  Config config_;

public:
  /**
   * \brief MotionComputationNode constructor
   */
  explicit MotionComputationNode(const rclcpp::NodeOptions &);

  /**
   * \brief Function callback for dynamic parameter updates
   */
  rcl_interfaces::msg::SetParametersResult parameter_update_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * \brief Function to publish ExternalObjectList
   * \param obj_pred_msg ExternalObjectList message to be published
   */
  void publishObject(const carma_perception_msgs::msg::ExternalObjectList & obj_pred_msg) const;

  ////
  // Overrides
  ////
  carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);
};

}  // namespace motion_computation

#endif  // MOTION_COMPUTATION__MOTION_COMPUTATION_NODE_HPP_
