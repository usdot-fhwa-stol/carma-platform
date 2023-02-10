#pragma once
/*
 * Copyright (C) 2020-2022 LEIDOS.
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
#include <tf2_ros/transform_broadcaster.h>
#include "pose_to_tf/PoseToTF2.hpp"
#include "pose_to_tf/PoseToTF2Config.hpp"
#include <carma_ros2_utils/carma_lifecycle_node.hpp>

namespace pose_to_tf
{

  /**
   * \class PoseToTfNode
   * \brief The class is responsible for processing pose to tf conversion.    
   */

  class PoseToTfNode : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // Subscribers
    carma_ros2_utils::SubPtr<geometry_msgs::msg::Pose> pose_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseStamped> pose_stamped_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovariance> pose_with_cov_sub;
    carma_ros2_utils::SubPtr<geometry_msgs::msg::PoseWithCovarianceStamped> pose_with_cov_stamped_sub;
    
    //PoseToTF2 class object
    std::shared_ptr <PoseToTF2> pose_to_tf_worker_;

    // Node configuration
    PoseToTF2Config config_;

  public:

    /**
     * \brief PoseToTfNode constructor 
     */
    explicit PoseToTfNode(const rclcpp::NodeOptions &);

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

  };

} // pose_to_tf
