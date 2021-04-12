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
#include <limits>
#include <math.h>
#include "mobilitypath_visualizer.h"

namespace mobilitypath_visualizer {

    namespace
    {
        // Helper function for computing 2d distance
        double compute_2d_distance(const geometry_msgs::Point& pt1, const geometry_msgs::Point&  pt2)
        {
            double dx = pt2.x - pt1.x;
            double dy = pt2.y - pt1.y;
            return sqrt(dx * dx + dy * dy);
        }
    }

    MobilityPathVisualizer::MobilityPathVisualizer() {}
    void MobilityPathVisualizer::run()
    {
        initialize();
        ros::CARMANodeHandle::setSpinRate(spin_rate_);
        ros::CARMANodeHandle::setSpinCallback(std::bind(&MobilityPathVisualizer::spinCallback, this));
        ros::CARMANodeHandle::spin();
    }

    void MobilityPathVisualizer::initialize()
    {
        // init CARMANodeHandle
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->param<double>("spin_rate", spin_rate_, 10.0);
        nh_->getParam("/vehicle_id", host_id_);

        // init publishers
        host_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("host_marker", 1);
        cav_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("cav_marker", 1);
        label_marker_pub_ = nh_->advertise<visualization_msgs::MarkerArray>("label_marker", 1);

        // init subscribers
        host_mob_path_sub_ = nh_->subscribe("mobility_path_msg", 1, &MobilityPathVisualizer::callbackMobilityPath, this);
        cav_mob_path_sub_ = nh_->subscribe("incoming_mobility_path", 1, &MobilityPathVisualizer::callbackMobilityPath, this);
        tf2_listener_.reset(new tf2_ros::TransformListener(tf2_buffer_));
    }
    
    void MobilityPathVisualizer::callbackMobilityPath(const cav_msgs::MobilityPath& msg)
    {
        ROS_ERROR_STREAM("Received new message");
        if (msg.header.timestamp == 0) //if empty
            return;
        ROS_ERROR_STREAM("Received msg from sender: " << msg.header.sender_id << ", and plan id:" << msg.header.plan_id << ", for receiver:" 
                            << msg.header.recipient_id << ", time:" << msg.header.timestamp << ", at now: " << std::to_string(ros::Time::now().toSec()));
        if (msg.header.recipient_id.compare(host_id_) != 0 && msg.header.sender_id.compare(host_id_) != 0)
        {
            return;
        }
        if (msg.header.plan_id == latest_cav_mob_path_msg_[msg.header.sender_id].header.plan_id &&
            msg.header.timestamp == latest_cav_mob_path_msg_[msg.header.sender_id].header.timestamp)
        {
            ROS_ERROR_STREAM("Already received this plan id:" << msg.header.plan_id );
            return;
        }
        latest_cav_mob_path_msg_[msg.header.sender_id] = msg;
        try
        {
            //tf2::convert(tf2_buffer_.lookupTransform("map", "map", ros::Time(0)).transform, map_in_earth_); 
            tf2::Transform map_in_earth_;
            map_in_earth_.setIdentity();
            MarkerColor cav_color;
            if (msg.header.sender_id == host_id_)
            {
                cav_color.green = 1.0;
                host_marker_ = composeVisualizationMarker(msg,map_in_earth_,cav_color);
                host_marker_received_ = true;
                ROS_ERROR_STREAM("Composed host marker successfuly!");
            }
            else
            {
                cav_color.blue = 1.0;
                cav_markers_.push_back(composeVisualizationMarker(msg,map_in_earth_,cav_color));
            }
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
        }
    
    }

    geometry_msgs::Point MobilityPathVisualizer::ECEFToMapPoint(const cav_msgs::LocationECEF& ecef_point, const tf2::Transform& map_in_earth) const
    {
        geometry_msgs::Point output;
        // convert input point to transform
        tf2::Transform point_in_earth;
        tf2::Quaternion no_rotation(0, 0, 0, 1);
        tf2::Vector3 input_point {(double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0};
        point_in_earth.setOrigin(input_point);
        point_in_earth.setRotation(no_rotation);
        // convert to map frame by (T_e_m)^(-1) * T_e_p
        auto point_in_map = map_in_earth.inverse() * point_in_earth;
        output.x = point_in_map.getOrigin().getX();
        output.y = point_in_map.getOrigin().getY();
        output.z = point_in_map.getOrigin().getZ();

        return output;
    } 

    visualization_msgs::MarkerArray MobilityPathVisualizer::composeVisualizationMarker(const cav_msgs::MobilityPath& msg, const tf2::Transform& map_in_earth, const MarkerColor& color)
    {
        visualization_msgs::MarkerArray output;
        
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time((double)msg.header.timestamp/1000.0);
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "mobilitypath_visualizer";

        marker.scale.x = 2.0;
        marker.scale.y = 2.0;
        marker.scale.z = 1.0;
        marker.frame_locked = true;

        marker.color.r = color.red;
        marker.color.g = color.green;
        marker.color.b = color.blue;
        marker.color.a = 1.0f;
        
        size_t count = std::max(prev_marker_list_size_[msg.header.sender_id], msg.trajectory.offsets.size());

        auto curr_location_msg = msg; //variable to update on each iteration as offsets are measured since last traj point
        
        marker.id = 0;
        geometry_msgs::Point arrow_start;
        cav_msgs::LocationECEF point; //in cm
        point.ecef_x = curr_location_msg.trajectory.location.ecef_x;
        point.ecef_y = curr_location_msg.trajectory.location.ecef_y;
        point.ecef_z = curr_location_msg.trajectory.location.ecef_z;
        ROS_ERROR_STREAM("ECEF- x: " << point.ecef_x << ", y:" << point.ecef_y);
        arrow_start = ECEFToMapPoint(point,map_in_earth); //also convert from cm to m
        ROS_ERROR_STREAM("Map- x: " << arrow_start.x << ", y:" << arrow_start.y);

        geometry_msgs::Point arrow_end;
        point.ecef_x = curr_location_msg.trajectory.location.ecef_x + msg.trajectory.offsets[0].offset_x;
        point.ecef_y = curr_location_msg.trajectory.location.ecef_y + msg.trajectory.offsets[0].offset_y;
        point.ecef_z = curr_location_msg.trajectory.location.ecef_z + msg.trajectory.offsets[0].offset_z;
        arrow_end = ECEFToMapPoint(point,map_in_earth); //also convert from cm to m

        marker.points.push_back(arrow_start);
        marker.points.push_back(arrow_end);

        curr_location_msg.trajectory.location = point;
        output.markers.push_back(marker);

        for (size_t i = 1; i < count; i++) // start id = 1 to comply with arrow marker type
        {
            marker.id = i;
            marker.header.stamp += ros::Duration(0.1);

            if (i >= msg.trajectory.offsets.size()) { // If we need to delete previous points
                marker.action = visualization_msgs::Marker::DELETE;
            }

            marker.points = {};
            point.ecef_x = (curr_location_msg.trajectory.location.ecef_x); //offset has been added on prev iteration
            point.ecef_y = (curr_location_msg.trajectory.location.ecef_y); 
            point.ecef_z = (curr_location_msg.trajectory.location.ecef_z);
            arrow_start = ECEFToMapPoint(point,map_in_earth); //convert from cm to m
            ROS_ERROR_STREAM("ECEF- x: " << point.ecef_x << ", y:" << point.ecef_y);
            ROS_ERROR_STREAM("Map- x: " << arrow_start.x << ", y:" << arrow_start.y);

            point.ecef_x = curr_location_msg.trajectory.location.ecef_x + msg.trajectory.offsets[i].offset_x;
            point.ecef_y = curr_location_msg.trajectory.location.ecef_y + msg.trajectory.offsets[i].offset_y;
            point.ecef_z = curr_location_msg.trajectory.location.ecef_z + msg.trajectory.offsets[i].offset_z;
            arrow_end = ECEFToMapPoint(point,map_in_earth);

            marker.points.push_back(arrow_start);
            marker.points.push_back(arrow_end);

            curr_location_msg.trajectory.location = point;
            output.markers.push_back(marker);
        }
        ROS_ERROR_STREAM("ECEF- x: " << point.ecef_x << ", y:" << point.ecef_y);
        ROS_ERROR_STREAM("Map- x: " << arrow_end.x << ", y:" << arrow_end.y);
        prev_marker_list_size_[msg.header.sender_id] = msg.trajectory.offsets.size();
        
        return output;
    }

    visualization_msgs::MarkerArray MobilityPathVisualizer::composeLabelMarker(visualization_msgs::MarkerArray host_marker, std::vector<visualization_msgs::MarkerArray> cav_markers)
    {
        visualization_msgs::MarkerArray output;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.ns = "mobilitypath_visualizer";

        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.frame_locked = true;
        marker.lifetime = ros::Duration(2);
        for (auto const& cav_marker: cav_markers)
        {
            size_t idx = 0;
            while(idx < host_marker.markers.size() && idx < cav_marker.markers.size())
            {
                if (compute_2d_distance(cav_marker.markers[idx].points[0], host_marker.markers[idx].points[0]) <= 1.0) // within 1 meter
                {
                    marker.header.stamp = ros::Time::now();
                    marker.pose.position.x = cav_marker.markers[idx].points[0].x;
                    marker.pose.position.y = cav_marker.markers[idx].points[0].y;
                    marker.text = "Collision in " + std::to_string((cav_marker.markers[idx].header.stamp - marker.header.stamp).toSec())+ "s!";
                    output.markers.push_back(marker);
                }
                idx++;
            }
            // last point
            
            if (compute_2d_distance(cav_marker.markers[idx-1].points[1], host_marker.markers[idx-1].points[1]) <= 1.0) // within 1 meter
            {
                marker.header.stamp = ros::Time::now();
                marker.pose.position.x = cav_marker.markers[idx-1].points[1].x;
                marker.pose.position.y = cav_marker.markers[idx-1].points[1].y;
                
                marker.text = "Collision in " + std::to_string((cav_marker.markers[idx-1].header.stamp + ros::Duration(0.1) - marker.header.stamp).toSec())+ "s!";
                output.markers.push_back(marker);
            }
        }
        
        return output;
    }

    
    std::vector<visualization_msgs::MarkerArray> MobilityPathVisualizer::matchTrajectoryTimestamps(const visualization_msgs::MarkerArray& host_marker, 
                                                                    const std::vector<visualization_msgs::MarkerArray>& cav_markers) const
    {
        std::vector<visualization_msgs::MarkerArray> synchronized_output;
        for (auto const& curr_cav: cav_markers)
        {
            visualization_msgs::MarkerArray synchronized_marker_array;
            unsigned int curr_idx = 0;
            double time_step = 0.1;
            // although it is very rare to reach here
            // we do not need to visualize other car that starts "in the future", so skip
            if (curr_cav.markers[0].header.stamp > host_marker.markers[0].header.stamp)
            {
                continue;
            }
            // this marker is outdated, drop
            if (curr_cav.markers.back().header.stamp + ros::Duration(time_step) < host_marker.markers[0].header.stamp) 
                continue;

            // skip points until the idx to start interpolating
            while (curr_idx < curr_cav.markers.size() && curr_cav.markers[curr_idx].header.stamp <= host_marker.markers[0].header.stamp)
            { 
                curr_idx++;
            }
            
            curr_idx -= 1; // cav_msg stamp is before that of host now

            // interpolate position to match the starting time (dt < time_step)
            double dt = (host_marker.markers[0].header.stamp - curr_cav.markers[curr_idx].header.stamp).toSec();
            
            ros::Time curr_time = host_marker.markers[0].header.stamp;
            
            // only update start_point of each arrow type marker for now

            while (curr_idx < curr_cav.markers.size())
            {
                visualization_msgs::Marker curr_marker = curr_cav.markers[curr_idx]; //copy static info
                if (dt != 0.0) // if not already synchronized
                {
                    double dx = (curr_cav.markers[curr_idx].points[1].x - curr_cav.markers[curr_idx].points[0].x)/time_step * dt;
                    double dy = (curr_cav.markers[curr_idx].points[1].y - curr_cav.markers[curr_idx].points[0].y)/time_step * dt;
                    curr_marker.points[0].x += dx; 
                    curr_marker.points[0].y += dy;
                } //else just loop and copy
                curr_marker.header.stamp = curr_time;
                synchronized_marker_array.markers.push_back(curr_marker);
                curr_idx ++;
                curr_time += ros::Duration(time_step);
            }

            curr_idx = 0; // resetting idx to work on new, synchronized list

            // update end_point of each arrow type marker, except that of last point
            while (curr_idx < synchronized_marker_array.markers.size()-1)
            {
                synchronized_marker_array.markers[curr_idx].points[1] = synchronized_marker_array.markers[curr_idx + 1].points[0];
                curr_idx ++;
            }

            // extrapolate the last point just to conform with 0.1s interval between points
            double dx = (synchronized_marker_array.markers[curr_idx].points[1].x - synchronized_marker_array.markers[curr_idx].points[0].x)/(time_step - dt) * time_step;
            double dy = (synchronized_marker_array.markers[curr_idx].points[1].y - synchronized_marker_array.markers[curr_idx].points[0].y)/(time_step - dt) * time_step;
            synchronized_marker_array.markers[curr_idx].points[1].x = synchronized_marker_array.markers[curr_idx].points[0].x + dx; 
            synchronized_marker_array.markers[curr_idx].points[1].y = synchronized_marker_array.markers[curr_idx].points[0].y + dy; 

            synchronized_output.push_back(synchronized_marker_array);
            
        }

        return synchronized_output;
    }

    bool MobilityPathVisualizer::spinCallback()
    {
        
        // match cav_markers' timestamps to that of host
        if (!host_marker_received_)
            return true;

        //cav_markers_ = matchTrajectoryTimestamps(host_marker_, cav_markers_);
        
        // publish host marker
        host_marker_pub_.publish(host_marker_);
        
        // publish cav markers
        for (auto const &marker: cav_markers_)
        {
            cav_marker_pub_.publish(marker);
        }

        // publish label
        label_marker_ = composeLabelMarker(host_marker_, cav_markers_);
        for (auto const &marker: label_marker_.markers)
        {
            label_marker_pub_.publish(marker);
        }

        return true;
    }

}

