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
#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <lanelet2_extension/projection/local_frame_projector.h>

TEST(MobilityPathVisualizerTest, TestComposeVisualizationMarker)
{
    rclcpp::NodeOptions options;
    auto viz_node = std::make_shared<mobilitypath_visualizer::MobilityPathVisualizer>(options);
    viz_node->configure();
    viz_node->activate();
    
    // 1 to 1 transform
    std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
    std_msgs::msg::String msg;
    msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> msg_ptr (new std_msgs::msg::String(msg));
    viz_node->georeferenceCallback(std::move(msg_ptr));  // Set projection
    
    // INPUT MSG
    carma_v2x_msgs::msg::MobilityPath input_msg;
    input_msg.m_header.plan_id = "";
    input_msg.m_header.sender_id = ""; //host
    input_msg.m_header.timestamp = 10000; // 10sec
    
    input_msg.trajectory.location.ecef_x = 0;
    input_msg.trajectory.location.ecef_y = 0;
    input_msg.trajectory.location.ecef_z = 0;

    carma_v2x_msgs::msg::LocationOffsetECEF offset;
    offset.offset_x = 100;
    input_msg.trajectory.offsets.push_back(offset); 
    input_msg.trajectory.offsets.push_back(offset); // actual positions now 0, 1, 2 -> 2 markers

    mobilitypath_visualizer::MarkerColor expected_color_blue;
    expected_color_blue.blue = 1.0;

    // EXPECTED RESULT STATIC INFO
    visualization_msgs::msg::MarkerArray expected_msg;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(10.0*1e9)); //10sec
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "mobilitypath_visualizer";

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.frame_locked = true;

    marker.color.r = expected_color_blue.red;
    marker.color.g = expected_color_blue.green;
    marker.color.b = expected_color_blue.blue;
    marker.color.a = 1.0f;
    
    // EXPECTED MSG DYNAMIC
    marker.id = 0;
    geometry_msgs::msg::Point point;
    point.x = 0; 
    marker.points.push_back(point);
    point.x = 1; //in meters
    marker.points.push_back(point);

    expected_msg.markers.push_back(marker);
    marker.header.stamp =  builtin_interfaces::msg::Time(rclcpp::Time(10.1*1e9)); 
    marker.id = 1;
    marker.points = {};
    point.x = 1;
    marker.points.push_back(point);
    point.x = 2;
    marker.points.push_back(point);

    expected_msg.markers.push_back(marker);

    auto result = viz_node->composeVisualizationMarker(input_msg, expected_color_blue);

    EXPECT_EQ(expected_msg.markers[0].header.frame_id, result.markers[0].header.frame_id);
    EXPECT_EQ(expected_msg.markers[0].header.stamp.sec, result.markers[0].header.stamp.sec);
    EXPECT_EQ(expected_msg.markers[0].type, result.markers[0].type);
    EXPECT_EQ(expected_msg.markers[0].action, result.markers[0].action);
    EXPECT_EQ(expected_msg.markers[0].ns, result.markers[0].ns);
    
    EXPECT_EQ(expected_msg.markers[0].color.a, result.markers[0].color.a);
    EXPECT_EQ(expected_msg.markers[0].color.b, result.markers[0].color.b);
    EXPECT_EQ(expected_msg.markers[0].color.g, result.markers[0].color.g);
    EXPECT_EQ(expected_msg.markers[0].color.r, result.markers[0].color.r);
    
    EXPECT_EQ(expected_msg.markers[0].points[0].x, result.markers[0].points[0].x);
    EXPECT_EQ(expected_msg.markers[0].points[0].y, result.markers[0].points[0].y);
    EXPECT_EQ(expected_msg.markers[0].points[0].z, result.markers[0].points[0].z);
    
    EXPECT_EQ(expected_msg.markers[0].points[1].x, result.markers[0].points[1].x);
    EXPECT_EQ(expected_msg.markers[0].points[1].y, result.markers[0].points[1].y);
    EXPECT_EQ(expected_msg.markers[0].points[1].z, result.markers[0].points[1].z);

    EXPECT_EQ(expected_msg.markers[1].points[0].x, result.markers[1].points[0].x);
    EXPECT_EQ(expected_msg.markers[1].points[1].x, result.markers[1].points[1].x);
    
}

TEST(MobilityPathVisualizerTest, TestECEFToMapPoint)
{
    rclcpp::NodeOptions options;
    auto viz_node = std::make_shared<mobilitypath_visualizer::MobilityPathVisualizer>(options);
    viz_node->configure();
    viz_node->activate();

    // 1 to 1 transform
    std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
    std_msgs::msg::String msg;
    msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> msg_ptr (new std_msgs::msg::String(msg));
    viz_node->georeferenceCallback(std::move(msg_ptr));  // Set projection

    carma_v2x_msgs::msg::LocationECEF ecef_point;
    ecef_point.ecef_x = 100;
    ecef_point.ecef_y = 200;
    ecef_point.ecef_z = 300;
    geometry_msgs::msg::Point expected_point;
    expected_point.x = 1;
    expected_point.y = 2;
    expected_point.z = 3;

    auto result = viz_node->ECEFToMapPoint(ecef_point);

    EXPECT_NEAR(expected_point.x, result.x, 0.0001);
    EXPECT_NEAR(expected_point.y, result.y, 0.0001);
    EXPECT_NEAR(expected_point.z, result.z, 0.0001);
}

TEST(MobilityPathVisualizerTest, TestMatchTrajectoryTimestamps)
{
    rclcpp::NodeOptions options;
    auto viz_node = std::make_shared<mobilitypath_visualizer::MobilityPathVisualizer>(options);
    viz_node->configure();
    viz_node->activate();

    // 1 to 1 transform
    std::string base_proj = lanelet::projection::LocalFrameProjector::ECEF_PROJ_STR;
    std_msgs::msg::String msg;
    msg.data = base_proj;
    std::unique_ptr<std_msgs::msg::String> msg_ptr(new std_msgs::msg::String(msg));
    viz_node->georeferenceCallback(std::move(msg_ptr));  // Set projection
    
    // INPUT RESULT STATIC INFO
    visualization_msgs::msg::MarkerArray host_msg;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(10*1e9)); //10sec  
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "mobilitypath_visualizer";
    
    // INPUT MSG DYNAMIC
    marker.id = 0;
    geometry_msgs::msg::Point point;
    point.x = 0; 
    marker.points.push_back(point);
    point.x = 1; //in meters
    marker.points.push_back(point);

    host_msg.markers.push_back(marker);
    marker.header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(10.1*1e9)); 
    
    marker.id = 1;
    marker.points = {};
    point.x = 1;
    marker.points.push_back(point);
    point.x = 2;
    marker.points.push_back(point);

    host_msg.markers.push_back(marker);

    // Base test
    // Give CAV Exactly same as Host Marker itself
    
    auto result = viz_node->matchTrajectoryTimestamps(host_msg, {host_msg});
    
    EXPECT_EQ(result[0].markers[0].header.stamp.sec, host_msg.markers[0].header.stamp.sec);
    EXPECT_EQ(result[0].markers[0].id, host_msg.markers[0].id);
    EXPECT_EQ(result[0].markers[0].points[0].x, host_msg.markers[0].points[0].x);
    EXPECT_EQ(result[0].markers[0].points[0].y, host_msg.markers[0].points[0].y);
    EXPECT_EQ(result[0].markers[0].points[1].x, host_msg.markers[0].points[1].x);
    EXPECT_EQ(result[0].markers[0].points[1].y, host_msg.markers[0].points[1].y);
    
    EXPECT_EQ(result[0].markers[1].header.stamp.sec, host_msg.markers[1].header.stamp.sec);
    EXPECT_EQ(result[0].markers[1].id, host_msg.markers[1].id);
    EXPECT_EQ(result[0].markers[1].points[0].x, host_msg.markers[1].points[0].x);
    EXPECT_EQ(result[0].markers[1].points[0].y, host_msg.markers[1].points[0].y);
    EXPECT_EQ(result[0].markers[1].points[1].x, host_msg.markers[1].points[1].x);
    EXPECT_EQ(result[0].markers[1].points[1].y, host_msg.markers[1].points[1].y);
    
    visualization_msgs::msg::MarkerArray cav_msg;
    cav_msg = host_msg;

    // NO POINTS DROPPED
    cav_msg.markers[0].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(9.96 * 1e9));
    cav_msg.markers[1].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(10.06 * 1e9));

    result = viz_node->matchTrajectoryTimestamps(host_msg, {cav_msg});
    
    visualization_msgs::msg::MarkerArray expected_msg;
    expected_msg = host_msg;
    
    expected_msg.markers[0].points[0].x = 0.4;
    expected_msg.markers[0].points[1].x = 1.4;
    expected_msg.markers[1].points[0].x = 1.4;
    expected_msg.markers[1].points[1].x = 2.4;
    
    EXPECT_EQ(expected_msg.markers[0].header.stamp.sec, result[0].markers[0].header.stamp.sec);
    EXPECT_EQ(expected_msg.markers[0].id, result[0].markers[0].id);
    EXPECT_EQ(expected_msg.markers[0].points[0].x, result[0].markers[0].points[0].x);
    EXPECT_EQ(expected_msg.markers[0].points[1].x, result[0].markers[0].points[1].x);
    
    EXPECT_EQ(expected_msg.markers[1].header.stamp.sec, result[0].markers[1].header.stamp.sec);
    EXPECT_EQ(expected_msg.markers[1].id, result[0].markers[1].id);
    EXPECT_EQ(expected_msg.markers[1].points[0].x, result[0].markers[1].points[0].x);
    EXPECT_EQ(expected_msg.markers[1].points[1].x, result[0].markers[1].points[1].x);
    
    // 1 OUT OF 2 POINTS DROPPED
    cav_msg.markers[0].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(9.86 * 1e9));
    cav_msg.markers[1].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(9.96 * 1e9));
    
    result = viz_node->matchTrajectoryTimestamps(host_msg, {cav_msg});
    
    expected_msg.markers[0].id = 1;
    expected_msg.markers[0].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(10.0* 1e9));
    expected_msg.markers[0].points[0].x = 1.4;
    expected_msg.markers[0].points[1].x = 2.4;
    
    EXPECT_EQ(expected_msg.markers[0].header.stamp, result[0].markers[0].header.stamp);
    EXPECT_EQ(expected_msg.markers[0].id, result[0].markers[0].id);
    EXPECT_EQ(expected_msg.markers[0].points[0].x, result[0].markers[0].points[0].x);
    EXPECT_EQ(expected_msg.markers[0].points[1].x, result[0].markers[0].points[1].x);
    
    // ALL POINTS DROPPED OUTDATED
    cav_msg.markers[0].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(9.66* 1e9));
    cav_msg.markers[1].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(9.76* 1e9));
    
    result = viz_node->matchTrajectoryTimestamps(host_msg, {cav_msg});
    
    EXPECT_EQ(0, result.size()); 

    // ALL POINTS DROPPED TOO FUTURE
    cav_msg.markers[0].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(11.66 * 1e9));
    cav_msg.markers[1].header.stamp = builtin_interfaces::msg::Time(rclcpp::Time(12.76 * 1e9));
    
    result = viz_node->matchTrajectoryTimestamps(host_msg, {cav_msg});
    
    EXPECT_EQ(0, result.size()); 
}

TEST(MobilityPathVisualizerTest, TestComposeLabelMarker)
{
    rclcpp::NodeOptions options;
    auto viz_node = std::make_shared<mobilitypath_visualizer::MobilityPathVisualizer>(options);
    viz_node->configure();
    viz_node->activate();
    
    // INPUT RESULT STATIC INFO
    visualization_msgs::msg::MarkerArray host_msg;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = builtin_interfaces::msg::Time(viz_node->now() + rclcpp::Duration(10.0 * 10e9)); //10sec
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "mobilitypath_visualizer";
    
    // INPUT MSG DYNAMIC
    marker.id = 0;
    geometry_msgs::msg::Point point;
    point.x = 0; 
    marker.points.push_back(point);
    point.x = 1; //in meters
    marker.points.push_back(point);

    host_msg.markers.push_back(marker);
    marker.header.stamp.sec += rclcpp::Duration(0.1,0.0).seconds(); 
    marker.id = 1;
    marker.points = {};
    point.x = 1;
    marker.points.push_back(point);
    point.x = 2;
    marker.points.push_back(point);

    host_msg.markers.push_back(marker);

    // Base test
    // Give CAV Exactly same as Host Marker itself
    // Collision in every seconds
    
    auto result = viz_node->composeLabelMarker(host_msg, {host_msg});
    
    //TEST STATIC
    EXPECT_EQ(result.markers[0].header.frame_id, "map");
    EXPECT_EQ(result.markers[0].type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    EXPECT_EQ(result.markers[0].action, visualization_msgs::msg::Marker::ADD);
    EXPECT_EQ(result.markers[0].ns, "mobilitypath_visualizer");
    EXPECT_EQ(result.markers[0].color.a, 1.0);
    EXPECT_EQ(result.markers[0].frame_locked, 1);

    // TEST DYNAMIC
    EXPECT_EQ(result.markers.size(), host_msg.markers.size() + 1);
    EXPECT_NEAR(result.markers[0].pose.position.x, host_msg.markers[0].points[0].x,0.001);
    EXPECT_NEAR(result.markers[0].pose.position.y, host_msg.markers[0].points[0].y,0.001);
    
    EXPECT_NEAR(result.markers[1].pose.position.x, host_msg.markers[1].points[0].x,0.001);
    EXPECT_NEAR(result.markers[1].pose.position.y, host_msg.markers[1].points[0].y,0.001);

    EXPECT_NEAR(result.markers[2].pose.position.x, host_msg.markers[1].points[1].x,0.001);
    EXPECT_NEAR(result.markers[2].pose.position.y, host_msg.markers[1].points[1].y,0.001);

    EXPECT_NEAR(result.markers[2].header.stamp.sec, host_msg.markers[1].header.stamp.sec + 0.1,0.1);
    
}

// Run all the tests
int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
}