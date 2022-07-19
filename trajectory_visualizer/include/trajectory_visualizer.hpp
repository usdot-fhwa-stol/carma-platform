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
#include <carma_planning_msgs/msg/trajectory_plan.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "trajectory_visualizer_config.hpp"

namespace trajectory_visualizer 
{

    /**
     * TrajectoryVisualizer . publishes trajectory represented by various colors of rviz markers depending on the speed of trajectory. 
     * More than 75% of max_speed is red, more 50% is blue, more than 25% is teal, less then 25% is green. 
     * 
    */ 
    const double MPH_TO_MS = 0.44704;

  class TrajectoryVisualizer : public carma_ros2_utils::CarmaLifecycleNode
    {

     private:
       // Subscribers
       carma_ros2_utils::SubPtr<carma_planning_msgs::msg::TrajectoryPlan> traj_sub_;

       // Publishers
       carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> traj_marker_pub_;

       // Node configuration
       Config config_;

       // callbacks
       void callbackPlanTrajectory(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg);

       // variables    
       double max_speed_;

       size_t prev_marker_list_size_ = 0;
       // we are not saving every trajectory history at this point

     public:

       /**
        * \brief TrajectoryVisualizer constructor 
        */
      explicit TrajectoryVisualizer(const rclcpp::NodeOptions &);

      ////
       // Overrides
      ////
     carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    };
}


