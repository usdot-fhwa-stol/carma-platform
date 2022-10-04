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

#include "mobilitypath_visualizer/mobilitypath_visualizer.hpp"

namespace mobilitypath_visualizer {
    
    namespace std_ph = std::placeholders;

    namespace
    {
        // Helper function for computing 2d distance
        double compute_2d_distance(const geometry_msgs::msg::Point& pt1, const geometry_msgs::msg::Point&  pt2)
        {
            double dx = pt2.x - pt1.x;
            double dy = pt2.y - pt1.y;
            double dz = pt2.z - pt1.z;

            return sqrt(dx * dx + dy * dy + dz * dz);
        }
    }

    MobilityPathVisualizer::MobilityPathVisualizer(const rclcpp::NodeOptions &options) 
            : carma_ros2_utils::CarmaLifecycleNode(options)
    {
        // Create initial config
        config_ = Config();

        // Declare parameters
        config_.timer_cb_rate = declare_parameter<int>("timer_cb_rate", config_.timer_cb_rate);
        config_.x = declare_parameter<double>("x", config_.x);
        config_.y = declare_parameter<double>("y", config_.y);
        config_.z = declare_parameter<double>("z", config_.z);
        config_.t = declare_parameter<double>("t", config_.t);
        config_.host_id = declare_parameter<std::string>("vehicle_id", config_.host_id);
    }


    carma_ros2_utils::CallbackReturn MobilityPathVisualizer::handle_on_configure(const rclcpp_lifecycle::State &)
    {
        //Reset Config
        config_ = Config();
        
        //Load parameters
        get_parameter<int>("timer_cb_rate", config_.timer_cb_rate);
        get_parameter<double>("x", config_.x);
        get_parameter<double>("y", config_.y);
        get_parameter<double>("z", config_.z);
        get_parameter<double>("t", config_.t);

        get_parameter("vehicle_id", config_.host_id);

        RCLCPP_INFO_STREAM(get_logger(), "Loaded params "<< config_);

        // init publishers
        host_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("host_marker", 1);
        cav_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("cav_marker", 1);
        label_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("label_marker", 1);

        // init subscribers
        host_mob_path_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityPath>("mobility_path_msg", 10,
                                                              std::bind(&MobilityPathVisualizer::callbackMobilityPath, this, std_ph::_1));

        cav_mob_path_sub_ = create_subscription<carma_v2x_msgs::msg::MobilityPath>("incoming_mobility_path", 10,
                                                              std::bind(&MobilityPathVisualizer::callbackMobilityPath, this, std_ph::_1));
    
        georeference_sub_ = create_subscription<std_msgs::msg::String>("georeference", 10,
                                                              std::bind(&MobilityPathVisualizer::georeferenceCallback, this, std_ph::_1));

        // Return success if everthing initialized successfully
        return CallbackReturn::SUCCESS;
    }

    carma_ros2_utils::CallbackReturn MobilityPathVisualizer::handle_on_activate(const rclcpp_lifecycle::State &){
        
        //Setup timer
        timer_ = create_timer(get_clock(), std::chrono::milliseconds(config_.timer_cb_rate), 
        std::bind(&MobilityPathVisualizer::timer_callback, this));

        return CallbackReturn::SUCCESS;
    }

    void MobilityPathVisualizer::timer_callback()
    {
        // match cav_markers' timestamps to that of host
        if (!host_marker_received_){
            return;
        }   

        // publish host marker
        host_marker_pub_->publish(host_marker_);
        
        cav_markers_ = matchTrajectoryTimestamps(host_marker_, cav_markers_);

        for (auto const &marker: cav_markers_)
        {
            cav_marker_pub_->publish(marker);
        }

        // publish label
        label_marker_ = composeLabelMarker(host_marker_, cav_markers_);
        label_marker_pub_->publish(label_marker_);
        
    }

    void MobilityPathVisualizer::georeferenceCallback(std_msgs::msg::String::UniquePtr msg) 
    {
        map_projector_ = std::make_shared<lanelet::projection::LocalFrameProjector>(msg->data.c_str());  // Build projector from proj string
    }
    
    void MobilityPathVisualizer::callbackMobilityPath(carma_v2x_msgs::msg::MobilityPath::UniquePtr msg)
    {
        if (msg->m_header.timestamp == 0) //if empty
        {
            host_marker_received_ = false;
            return;
        } 
        RCLCPP_DEBUG_STREAM(get_logger(), "Received a msg from sender: " << msg->m_header.sender_id << ", and plan id:" << msg->m_header.plan_id << ", for receiver:" 
                            << msg->m_header.recipient_id << ", time:" << msg->m_header.timestamp << ", at now: " << std::to_string(this->now().seconds()));

        if (latest_cav_mob_path_msg_.find(msg->m_header.sender_id) != latest_cav_mob_path_msg_.end() &&
            msg->m_header.plan_id.compare(latest_cav_mob_path_msg_[msg->m_header.sender_id].m_header.plan_id) == 0 &&
            msg->m_header.timestamp == latest_cav_mob_path_msg_[msg->m_header.sender_id].m_header.timestamp)
        {
            RCLCPP_DEBUG_STREAM(get_logger(), "Already received this plan id:" << msg->m_header.plan_id << "from sender_id: " << msg->m_header.sender_id);
            return;
        }
        latest_cav_mob_path_msg_[msg->m_header.sender_id] = *msg;

        if (!map_projector_) {
            RCLCPP_DEBUG(get_logger(), "Cannot visualize mobility path as map projection not yet available");
            return;
        }

        MarkerColor cav_color;
        if (msg->m_header.sender_id.compare(config_.host_id) == 0)
        {
            cav_color.green = 1.0;
            host_marker_ = composeVisualizationMarker(*msg,cav_color);
            host_marker_received_ = true;
            RCLCPP_DEBUG_STREAM(get_logger(), "Composed host marker successfuly!");
        }
        else
        {
            cav_color.blue = 1.0;
            cav_markers_.push_back(composeVisualizationMarker(*msg,cav_color));
            RCLCPP_DEBUG_STREAM(get_logger(), "Composed cav marker successfuly! with sender_id: " << msg->m_header.sender_id);
        }

    }

    visualization_msgs::msg::MarkerArray MobilityPathVisualizer::composeVisualizationMarker(const carma_v2x_msgs::msg::MobilityPath& msg, const MarkerColor& color)
    {
        visualization_msgs::msg::MarkerArray output;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";

        rclcpp::Time marker_header_stamp = rclcpp::Time((msg.m_header.timestamp/1000.0) * 1e9);
        marker.header.stamp = builtin_interfaces::msg::Time(marker_header_stamp);
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "mobilitypath_visualizer";

        marker.scale.x = config_.x;
        marker.scale.y = config_.y;
        marker.scale.z = config_.z;
        marker.frame_locked = true;

        marker.color.r = (float)color.red;
        marker.color.g = (float)color.green;
        marker.color.b = (float)color.blue;
        marker.color.a = 1.0f;
        
        size_t count = std::max(prev_marker_list_size_[msg.m_header.sender_id], msg.trajectory.offsets.size());

        auto curr_location_msg = msg; //variable to update on each iteration as offsets are measured since last traj point
        
        marker.id = 0;
        geometry_msgs::msg::Point arrow_start;
        geometry_msgs::msg::Point arrow_end;

        if (msg.trajectory.offsets.empty())
        {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        else
        {
            RCLCPP_DEBUG_STREAM(get_logger(), "ECEF point x: " << curr_location_msg.trajectory.location.ecef_x << ", y:" << curr_location_msg.trajectory.location.ecef_y);
            arrow_start = ECEFToMapPoint(curr_location_msg.trajectory.location); //also convert from cm to m
            RCLCPP_DEBUG_STREAM(get_logger(), "Map point x: " << arrow_start.x << ", y:" << arrow_start.y);
            
            curr_location_msg.trajectory.location.ecef_x += + msg.trajectory.offsets[0].offset_x;
            curr_location_msg.trajectory.location.ecef_y += + msg.trajectory.offsets[0].offset_y;
            curr_location_msg.trajectory.location.ecef_z += + msg.trajectory.offsets[0].offset_z;
            arrow_end = ECEFToMapPoint(curr_location_msg.trajectory.location); //also convert from cm to m

            marker.points.push_back(arrow_start);
            marker.points.push_back(arrow_end);
        }

        output.markers.push_back(marker);

        for (size_t i = 1; i < count; i++) // start id = 1 to comply with arrow marker type
        {
            marker.id = i;
            rclcpp::Time marker_cur_time(marker.header.stamp);
            //Update time by 0.1s
            rclcpp::Time updated_time =  marker_cur_time + rclcpp::Duration(0.1 * 1e9);
            marker.header.stamp = builtin_interfaces::msg::Time(updated_time);

            if (i >= msg.trajectory.offsets.size()) { // If we need to delete previous points
                marker.action = visualization_msgs::msg::Marker::DELETE;
                output.markers.push_back(marker);
                continue;
            }

            marker.points = {};
            RCLCPP_DEBUG_STREAM(get_logger(), "ECEF Point- DEBUG x: " << curr_location_msg.trajectory.location.ecef_x << ", y:" << curr_location_msg.trajectory.location.ecef_y);
            arrow_start = ECEFToMapPoint(curr_location_msg.trajectory.location); //convert from cm to m
            RCLCPP_DEBUG_STREAM(get_logger(), "Map Point- DEBUG x: " << arrow_start.x << ", y:" << arrow_start.y);

            curr_location_msg.trajectory.location.ecef_x += msg.trajectory.offsets[i].offset_x;
            curr_location_msg.trajectory.location.ecef_y += msg.trajectory.offsets[i].offset_y;
            curr_location_msg.trajectory.location.ecef_z += msg.trajectory.offsets[i].offset_z;
            arrow_end = ECEFToMapPoint(curr_location_msg.trajectory.location);

            marker.points.push_back(arrow_start);
            marker.points.push_back(arrow_end);

            output.markers.push_back(marker);
        }
        RCLCPP_DEBUG_STREAM(get_logger(), "Last ECEF Point- DEBUG x: " << curr_location_msg.trajectory.location.ecef_x << ", y:" << curr_location_msg.trajectory.location.ecef_y);
        RCLCPP_DEBUG_STREAM(get_logger(), "Last Map Point- DEBUG x: " << arrow_end.x << ", y:" << arrow_end.y);
        prev_marker_list_size_[msg.m_header.sender_id] = msg.trajectory.offsets.size();
        
        return output;
    }

    geometry_msgs::msg::Point MobilityPathVisualizer::ECEFToMapPoint(const carma_v2x_msgs::msg::LocationECEF& ecef_point) const
    {

        if (!map_projector_) {
            throw std::invalid_argument("No map projector available for ecef conversion");
        }
        geometry_msgs::msg::Point output;
        
        lanelet::BasicPoint3d map_point = map_projector_->projectECEF( { (double)ecef_point.ecef_x/100.0, (double)ecef_point.ecef_y/100.0, (double)ecef_point.ecef_z/100.0 } , -1);
        output.x = map_point.x();
        output.y = map_point.y();
        output.z = map_point.z();

        return output;
    } 
    

    visualization_msgs::msg::MarkerArray MobilityPathVisualizer::composeLabelMarker(const visualization_msgs::msg::MarkerArray& host_marker, const std::vector<visualization_msgs::msg::MarkerArray>& cav_markers) const
    {
        visualization_msgs::msg::MarkerArray output;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "mobilitypath_visualizer";

        marker.scale.x = config_.x;
        marker.scale.y = config_.y;
        marker.scale.z = config_.z;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.frame_locked = true;
        marker.lifetime = builtin_interfaces::msg::Duration((rclcpp::Duration(config_.t * 1e9)));
        
        for (auto const& cav_marker: cav_markers)
        {
            size_t idx = 0;
            while(idx < host_marker.markers.size() && idx < cav_marker.markers.size())
            {
                if (compute_2d_distance(cav_marker.markers[idx].points[0], host_marker.markers[idx].points[0]) <= 1.0) // within 1 meter
                {
                    marker.id = (int32_t)idx;
                    marker.header.stamp = host_marker.markers[idx].header.stamp;
                    marker.pose.position.x = cav_marker.markers[idx].points[0].x;
                    marker.pose.position.y = cav_marker.markers[idx].points[0].y;
                    marker.pose.position.z = cav_marker.markers[idx].points[0].z;
                    marker.pose.orientation.w = 1.0f;

                    std::string collision_time = std::to_string((rclcpp::Time(cav_marker.markers[idx].header.stamp) - rclcpp::Time(host_marker.markers[0].header.stamp)).seconds());
                    collision_time = collision_time.substr(0,collision_time.size() - 4); //reduce precision to 2 decimals
                    marker.text = "Collision in " + collision_time + "s!";

                    output.markers.push_back(marker);
                }
                idx++;
            }
            
            if (compute_2d_distance(cav_marker.markers[idx-1].points[1], host_marker.markers[idx-1].points[1]) <= 1.0) // within 1 meter
            {   
                // last point
                marker.id = (int32_t)idx;
                marker.header.stamp = host_marker.markers[idx-1].header.stamp;
                marker.pose.position.x = cav_marker.markers[idx-1].points[1].x;
                marker.pose.position.y = cav_marker.markers[idx-1].points[1].y;
                marker.pose.position.z = cav_marker.markers[idx-1].points[1].z;

                rclcpp::Time cav_marker_stamp(cav_marker.markers[idx-1].header.stamp);
                rclcpp::Time host_marker_stamp(host_marker.markers[0].header.stamp);
                rclcpp::Duration duration = cav_marker_stamp - host_marker_stamp;
                std::string collision_time = std::to_string(duration.seconds());
                collision_time = collision_time.substr(0,collision_time.size() - 4); //reduce precision to 2 decimals
                marker.text = "Collision in " + collision_time + "s!";
                output.markers.push_back(marker);
            }
        }
        return output;
    }

    std::vector<visualization_msgs::msg::MarkerArray> MobilityPathVisualizer::matchTrajectoryTimestamps(const visualization_msgs::msg::MarkerArray& host_marker, 
                                                                    const std::vector<visualization_msgs::msg::MarkerArray>& cav_markers) const
    {
        std::vector<visualization_msgs::msg::MarkerArray> synchronized_output;
        for (auto const& curr_cav: cav_markers)
        {
            visualization_msgs::msg::MarkerArray synchronized_marker_array;
            unsigned int curr_idx = 0;
            double time_step = 0.1;
            // although it is very rare to reach here
            // we do not need to visualize other car that starts "in the future", so skip
            if (rclcpp::Time(curr_cav.markers[0].header.stamp) > rclcpp::Time(host_marker.markers[0].header.stamp))
            {
                continue;
            }
            // this marker is outdated, drop
            rclcpp::Time curr_cav_marker_stamp(curr_cav.markers.back().header.stamp);
            rclcpp::Time host_marker_stamp(host_marker.markers[0].header.stamp);
            
            if (curr_cav_marker_stamp + rclcpp::Duration(time_step * 1e9) < host_marker_stamp){
                continue;
            }
            
            // skip points until the idx to start interpolating
            while (curr_idx < curr_cav.markers.size() && rclcpp::Time(curr_cav.markers[curr_idx].header.stamp) <= rclcpp::Time(host_marker.markers[0].header.stamp))
            { 
                curr_idx++;
            }
            
            curr_idx -= 1; // carma_v2x_msg stamp is before that of host now
            // interpolate position to match the starting time (dt < time_step)
            double dt = (rclcpp::Time(host_marker.markers[0].header.stamp) - rclcpp::Time(curr_cav.markers[curr_idx].header.stamp)).seconds();
            
            rclcpp::Time curr_time(host_marker.markers[0].header.stamp);
            
            // only update start_point of each arrow type marker for now

            while (curr_idx < curr_cav.markers.size())
            {
                visualization_msgs::msg::Marker curr_marker = curr_cav.markers[curr_idx]; //copy static info
                if (dt != 0.0) // if not already synchronized
                {
                    double dx = (curr_cav.markers[curr_idx].points[1].x - curr_cav.markers[curr_idx].points[0].x)/time_step * dt;
                    double dy = (curr_cav.markers[curr_idx].points[1].y - curr_cav.markers[curr_idx].points[0].y)/time_step * dt;
                    double dz = (curr_cav.markers[curr_idx].points[1].z - curr_cav.markers[curr_idx].points[0].z)/time_step * dt;

                    curr_marker.points[0].x += dx; 
                    curr_marker.points[0].y += dy;
                    curr_marker.points[0].z += dz;

                } //else just loop and copy
                curr_marker.header.stamp = curr_time;
                synchronized_marker_array.markers.push_back(curr_marker);
                curr_idx ++;
                curr_time += rclcpp::Duration(time_step, 0.0);
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
            double dz = (synchronized_marker_array.markers[curr_idx].points[1].z - synchronized_marker_array.markers[curr_idx].points[0].z)/(time_step - dt) * time_step;

            synchronized_marker_array.markers[curr_idx].points[1].x = synchronized_marker_array.markers[curr_idx].points[0].x + dx; 
            synchronized_marker_array.markers[curr_idx].points[1].y = synchronized_marker_array.markers[curr_idx].points[0].y + dy;
            synchronized_marker_array.markers[curr_idx].points[1].z = synchronized_marker_array.markers[curr_idx].points[0].z + dz; 


            synchronized_output.push_back(synchronized_marker_array);
            
        }

        return synchronized_output;
    }

}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(mobilitypath_visualizer::MobilityPathVisualizer)
