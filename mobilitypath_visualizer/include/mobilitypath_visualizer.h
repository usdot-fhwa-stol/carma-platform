#pragma once

/*
 * Copyright (C) 2020 LEIDOS.
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
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <boost/shared_ptr.hpp>
#include <cav_msgs/MobilityPath.h>
#include <cav_msgs/BSM.h>
#include <geometry_msgs/PoseStamped.h>
#include <unordered_map>


namespace mobilitypath_visualizer {

    /**
     * MobilityPathVisualizer TODO
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

        visualization_msgs::MarkerArray composeVisualizationMarker(const cav_msgs::MobilityPath& msg, const tf2::Transform& map_in_earth, const MarkerColor& color);

        visualization_msgs::MarkerArray composeLabelMarker(visualization_msgs::MarkerArray host_marker, std::vector<visualization_msgs::MarkerArray> cav_markers);

        // expects points in cm
        geometry_msgs::Point ECEFToMapPoint(const cav_msgs::LocationECEF& ecef_point, const tf2::Transform& map_in_earth) const;

        // we are assuming host is planning every 0.1s
        // which means host position/time is updated every 0.1s and 
        // it is nearly impossible for other cars' marker to be starting later than the host other than 0.1s error
        // and unless the other car has significantly mismatched time
    
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

        // spin rate
        double spin_rate_;

        // callbacks
        void callbackMobilityPath(const cav_msgs::MobilityPath& msg);
        bool spinCallback();

        // TF listener
        tf2_ros::Buffer tf2_buffer_;
        std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
        tf2::Transform map_in_earth_;

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

    };

}


