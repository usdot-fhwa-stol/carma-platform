#pragma once

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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <carma_v2x_msgs/msg/mobility_path.hpp>
#include <unordered_map>
#include <lanelet2_extension/projection/local_frame_projector.h>

#include "mobilitypath_visualizer/mobilitypath_visualizer_config.hpp"


namespace mobilitypath_visualizer {

    /**
     * MobilityPathVisualizer visualizes all incoming and host's mobilitypath in rviz. It also detects if any path is close to 1meter distance
     *                        detecting collision and displays it in text
     * 
    */ 
    struct MarkerColor
    {
        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;
    };

    class MobilityPathVisualizer :public carma_ros2_utils::CarmaLifecycleNode
    {
        
    public:

        
        /**
         * \brief Constructor
         */
        MobilityPathVisualizer(const rclcpp::NodeOptions &);

        /**
         * \brief Compose a visualization marker for mobilitypath messages.
         * \param msg Mobiliy path message
         * \param color color to visualize the marker, for example host car should have different color than other car
         * \return Visualization Marker in arrow type
         */
        visualization_msgs::msg::MarkerArray composeVisualizationMarker(const carma_v2x_msgs::msg::MobilityPath& msg, const MarkerColor& color);
        
        /**
         * \brief Accepts ECEF point in cm to convert to a point in map in meters
         * \param ecef_point ECEF point to convert in cm
         * \return Point in map
         */
        geometry_msgs::msg::Point ECEFToMapPoint(const carma_v2x_msgs::msg::LocationECEF& ecef_point) const;

        /**
         * \brief Compose a label marker that displays whether if any of the cav's path cross with that of host (respective points are within 1 meter)
         * \param host_marker Host marker's visualization marker as arrow type
         * \param cav_markers Other CAV marker's visualization markers as arrow type
         * \note  This function assumes that every point's timestamp in marker is matched
         * \return Visualization Marker in text type
         */
        visualization_msgs::msg::MarkerArray composeLabelMarker(const visualization_msgs::msg::MarkerArray& host_marker, const std::vector<visualization_msgs::msg::MarkerArray>& cav_markers) const;

        /**
         * \brief Matches timestamps of CAV's individual points of cav_markers to that of host_marker and interpolates their points using the speed between points
         * \param host_marker Host marker's visualization marker as arrow type
         * \param cav_markers Other CAV marker's visualization markers as arrow type
         * \note  This function assumes 0.1s between any of the points. It also drops CAV markers that start later than the first point in host_marker does.
         *        This is acceptable as the host is publishing in 0.1s and we don't need to visualize points in the future.
         *        It extrapolates the last point CAV's just to conform with 
         * \return Synchronized CAV markers 
         */
        std::vector<visualization_msgs::msg::MarkerArray> matchTrajectoryTimestamps(const visualization_msgs::msg::MarkerArray& host_marker, 
                                                                    const std::vector<visualization_msgs::msg::MarkerArray>& cav_markers) const;


        /**
         * \brief Callback for map projection string to define lat/lon -> map conversion
         * \brief msg The proj string defining the projection.
         */ 
        void georeferenceCallback(std_msgs::msg::String::UniquePtr msg);

        ////
        // Overrides
        ////
        carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

        carma_ros2_utils::CallbackReturn handle_on_activate(const rclcpp_lifecycle::State &);

    private:

        // publisher
        carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> host_marker_pub_;
        carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> cav_marker_pub_;
        carma_ros2_utils::PubPtr<visualization_msgs::msg::MarkerArray> label_marker_pub_;
        
        // subscriber
        carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityPath> host_mob_path_sub_;
        carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::MobilityPath> cav_mob_path_sub_;
        carma_ros2_utils::SubPtr<std_msgs::msg::String> georeference_sub_;
        
        // initialize this node before running
        void initialize();

        std::shared_ptr<lanelet::projection::LocalFrameProjector> map_projector_;

        Config config_;

        // Timers
        rclcpp::TimerBase::SharedPtr timer_;

        // callbacks
        void callbackMobilityPath(carma_v2x_msgs::msg::MobilityPath::UniquePtr msg);
        void timer_callback();
        
        // latest msgs
        std::unordered_map<std::string, carma_v2x_msgs::msg::MobilityPath> latest_cav_mob_path_msg_;

        // marker msgs
        visualization_msgs::msg::MarkerArray host_marker_;
        std::vector<visualization_msgs::msg::MarkerArray> cav_markers_;
        visualization_msgs::msg::MarkerArray label_marker_;

        // helper variable
        std::unordered_map<std::string, size_t> prev_marker_list_size_;
        bool host_marker_received_ = false;

    };

}


