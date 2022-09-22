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
#include <limits>
#include <math.h>
#include "trajectory_visualizer.hpp"

namespace trajectory_visualizer {

  namespace std_ph = std::placeholders;

  TrajectoryVisualizer::TrajectoryVisualizer(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.max_speed = declare_parameter<double>("max_speed", config_.max_speed);
  }

    carma_ros2_utils::CallbackReturn TrajectoryVisualizer::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    // Setup subscribers
    traj_sub_ = create_subscription<carma_planning_msgs::msg::TrajectoryPlan>("plan_trajectory", 50,std::bind(&TrajectoryVisualizer::callbackPlanTrajectory,this,std_ph::_1));
    // Setup publisher
    traj_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_visualizer", 1);

    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }
 
    bool validateTime(const  rclcpp::Time& time) {

        int64_t sec64 = static_cast<int64_t>(floor(time.seconds()));
        if (sec64 < std::numeric_limits<int32_t>::min() || sec64 > std::numeric_limits<int32_t>::max())
           return false;

        return true;
    }

    void TrajectoryVisualizer::callbackPlanTrajectory(carma_planning_msgs::msg::TrajectoryPlan::UniquePtr msg)
    {
        if (msg->trajectory_points.size() == 0)
        {
           RCLCPP_WARN_STREAM(this->get_logger(),"No trajectory point in plan_trajectory! Returning");
        }
        visualization_msgs::msg::MarkerArray tmp_marker_array;
        // display by markers the velocity between each trajectory point/target time.
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp =  this->now();
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "trajectory_visualizer";


        marker.scale.x = 2;
        marker.scale.y = 2;
        marker.scale.z = 1;
        marker.frame_locked = true;

        size_t count = std::max(prev_marker_list_size_, msg->trajectory_points.size());

        for (size_t i = 1; i < count; i++)
        {
            marker.id = i;

            if (i >= msg->trajectory_points.size()) { // If we need to delete previous points
                marker.action = visualization_msgs::msg::Marker::DELETE;
                tmp_marker_array.markers.push_back(marker);
                continue;
            }
            
            double max_speed = config_.max_speed * MPH_TO_MS;
            double speed = max_speed;

            rclcpp::Time t2 = msg->trajectory_points[i].target_time;
            rclcpp::Time t1 = msg->trajectory_points[i - 1].target_time;

            if (validateTime(t2) && validateTime(t1) && t2 > t1 ) {
                rclcpp::Duration dt = t2 - t1;
                
                double dx = msg->trajectory_points[i].x - msg->trajectory_points[i-1].x;
                double dy = msg->trajectory_points[i].y - msg->trajectory_points[i-1].y;
                double dist = sqrt(dx * dx + dy * dy);

                speed = dist/ dt.seconds();
            }



            // map color to the scale of the speed
            // red being the highest, green being the lowest (0ms)
            
            RCLCPP_DEBUG_STREAM(this->get_logger(),"Speed:" << speed << "ms, max_speed:" << max_speed << "ms");
            if (speed > max_speed) 
            {
                RCLCPP_DEBUG_STREAM(this->get_logger(),"Speed was big, so capped at " << max_speed << "ms");
                speed = max_speed;
            }

            marker.color.r = 0.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0f;


            double ratio = speed / max_speed;
            if (ratio >= 0.75f)
            {
                marker.color.r = 1.0f;
            }
            else if (ratio >= 0.5f)
            {
                marker.color.b = 1.0f;
            }
            else if (ratio >= 0.25)
            {
                marker.color.b = 1.0f;
                marker.color.g = 1.0f;

            }
            else if (ratio >= 0.0)
            {
                marker.color.g = 1.0f;
            }

            marker.points = {};
            geometry_msgs::msg::Point start;
            start.x = msg->trajectory_points[i-1].x;
            start.y = msg->trajectory_points[i-1].y;

            geometry_msgs::msg::Point end;
            end.x = msg->trajectory_points[i].x;
            end.y = msg->trajectory_points[i].y;
            marker.points.push_back(start);
            marker.points.push_back(end);

            tmp_marker_array.markers.push_back(marker);
        }

        prev_marker_list_size_ = msg->trajectory_points.size();
        traj_marker_pub_->publish(tmp_marker_array);
    }
    
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(trajectory_visualizer::TrajectoryVisualizer)

