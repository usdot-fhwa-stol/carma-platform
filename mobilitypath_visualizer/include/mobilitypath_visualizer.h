#pragma once

/*
 * Copyright (C) 2021 LEIDOS.
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

#include <ros/ros.h>
#include <carma_utils/CARMAUtils.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <vector>
#include <cav_msgs/MobilityPath.h>
#include <unordered_map>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


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

    class MobilityPathVisualizer
    {
        
    public:

        
        /**
         * \brief Default constructor
         */
        MobilityPathVisualizer();

        /**
         * \brief General starting point to run this node
         */
        void run();

        /**
         * \brief Compose a visualization marker for mobilitypath messages.
         * \param msg Mobiliy path message
         * \param map_in_earth Transform to convert ECEF points into map points
         * \param color color to visualize the marker, for example host car should have different color than other car
         * \return Visualization Marker in arrow type
         */
        visualization_msgs::MarkerArray composeVisualizationMarker(const cav_msgs::MobilityPath& msg, const MarkerColor& color, const tf2::Transform& map_in_earth);
        
        /**
         * \brief Accepts ECEF point in cm to convert to a point in map in meters
         * \param ecef_point ECEF point to convert in cm
         * \param map_in_earth A transform from ECEF to map
         * \return Point in map
         */
        geometry_msgs::Point ECEFToMapPoint(const cav_msgs::LocationECEF& ecef_point, const tf2::Transform& map_in_earth) const;

        /**
         * \brief Compose a label marker that displays whether if any of the cav's path cross with that of host (respective points are within 1 meter)
         * \param host_marker Host marker's visualization marker as arrow type
         * \param cav_markers Other CAV marker's visualization markers as arrow type
         * \note  This function assumes that every point's timestamp in marker is matched
         * \return Visualization Marker in text type
         */
        visualization_msgs::MarkerArray composeLabelMarker(const visualization_msgs::MarkerArray& host_marker, const std::vector<visualization_msgs::MarkerArray>& cav_markers) const;

        /**
         * \brief Matches timestamps of CAV's individual points of cav_markers to that of host_marker and interpolates their points using the speed between points
         * \param host_marker Host marker's visualization marker as arrow type
         * \param cav_markers Other CAV marker's visualization markers as arrow type
         * \note  This function assumes 0.1s between any of the points. It also drops CAV markers that start later than the first point in host_marker does.
         *        This is acceptable as the host is publishing in 0.1s and we don't need to visualize points in the future.
         *        It extrapolates the last point CAV's just to conform with 
         * \return Synchronized CAV markers 
         */
        std::vector<visualization_msgs::MarkerArray> matchTrajectoryTimestamps(const visualization_msgs::MarkerArray& host_marker, 
                                                                    const std::vector<visualization_msgs::MarkerArray>& cav_markers) const;

    private:

        // public and private node handles
        std::shared_ptr<ros::CARMANodeHandle> nh_, pnh_;

        // publisher
        ros::Publisher host_marker_pub_;
        ros::Publisher cav_marker_pub_;
        ros::Publisher label_marker_pub_;
        
        // subscriber
        ros::Subscriber host_mob_path_sub_;
        ros::Subscriber cav_mob_path_sub_;
        
        // initialize this node before running
        void initialize();

        // TF listener
        tf2_ros::Buffer tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
        tf2::Transform map_in_earth_;

        // spin rate
        double spin_rate_;

        // callbacks
        void callbackMobilityPath(const cav_msgs::MobilityPath& msg);
        bool spinCallback();

        // latest msgs
        std::unordered_map<std::string, cav_msgs::MobilityPath> latest_cav_mob_path_msg_;

        // marker msgs
        visualization_msgs::MarkerArray host_marker_;
        std::vector<visualization_msgs::MarkerArray> cav_markers_;
        visualization_msgs::MarkerArray label_marker_;

        // helper variable
        std::unordered_map<std::string, size_t> prev_marker_list_size_;
        std::string host_id_;
        bool host_marker_received_ = false;

        // Visualization::MarkerArray parameters
        double x_;
        double y_;
        double z_;
        double t_;
    };

}


