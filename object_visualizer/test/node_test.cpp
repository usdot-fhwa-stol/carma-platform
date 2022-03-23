/*
 * Copyright (C) 2022 LEIDOS.
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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>

#include "object_visualizer/object_visualizer_node.hpp"


TEST(Testobject_visualizer, external_objects_test){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<object_visualizer::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::unique_ptr<carma_perception_msgs::msg::ExternalObjectList> msg = std::make_unique<carma_perception_msgs::msg::ExternalObjectList>();
    
    msg->header.frame_id = "map";

    carma_perception_msgs::msg::ExternalObject obj;
    obj.id = 1;
    obj.header.frame_id = "map";
    obj.pose.pose.position.x = 1.0;
    obj.pose.pose.position.y = 2.0;
    obj.pose.pose.position.z = 3.0;
    obj.pose.pose.orientation.x = 0.0;
    obj.pose.pose.orientation.y = 1.0;
    obj.pose.pose.orientation.z = 0.0;
    obj.pose.pose.orientation.w = 1.0;

    obj.size.x = 4.0;
    obj.size.y = 5.0;
    obj.size.z = 6.0;

    obj.object_type = carma_perception_msgs::msg::ExternalObject::PEDESTRIAN;

    msg->objects.push_back(obj);

    auto obj2 = obj;
    obj2.id = 2;
    obj2.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;

    msg->objects.push_back(obj2); // Duplicate the object so that we can check the id is incremented



    // create subscription to receive message after publish

    visualization_msgs::msg::MarkerArray result;
    auto sub = worker_node->create_subscription<visualization_msgs::msg::MarkerArray>("external_objects_viz", 1,
        [&](visualization_msgs::msg::MarkerArray::UniquePtr msg)
        {
            result = *msg;
        });


    worker_node->external_objects_callback(move(msg)); // Manually drive topic callbacks

    
    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger


    // Check that the result is correct
    ASSERT_EQ(result.markers.size(), 2u);
    ASSERT_EQ(result.markers[0].id, 0);
    ASSERT_EQ(result.markers[1].id, 1);

    ASSERT_EQ(result.markers[0].header.frame_id.compare("map"), 0);
    ASSERT_EQ(result.markers[0].pose.position.x, obj.pose.pose.position.x);
    ASSERT_EQ(result.markers[0].pose.position.y, obj.pose.pose.position.y);
    ASSERT_EQ(result.markers[0].pose.position.z, obj.pose.pose.position.z);
    ASSERT_EQ(result.markers[0].pose.orientation.x, obj.pose.pose.orientation.x);
    ASSERT_EQ(result.markers[0].pose.orientation.y, obj.pose.pose.orientation.y);
    ASSERT_EQ(result.markers[0].pose.orientation.z, obj.pose.pose.orientation.z);
    ASSERT_EQ(result.markers[0].pose.orientation.w, obj.pose.pose.orientation.w);

    ASSERT_NEAR(result.markers[0].scale.x, obj.size.x * 2.0, 0.00001);
    ASSERT_NEAR(result.markers[0].scale.y, obj.size.y * 2.0, 0.00001);
    ASSERT_NEAR(result.markers[0].scale.z, obj.size.z * 2.0, 0.00001);

    ASSERT_EQ(result.markers[0].type, visualization_msgs::msg::Marker::CUBE);

    ASSERT_EQ(result.markers[0].action, visualization_msgs::msg::Marker::ADD);
    ASSERT_EQ(result.markers[1].action, visualization_msgs::msg::Marker::ADD);

    ASSERT_EQ(result.markers[0].color.r, 1.0);
    ASSERT_EQ(result.markers[0].color.g, 0.0);
    ASSERT_EQ(result.markers[0].color.b, 0.0);
    ASSERT_EQ(result.markers[0].color.a, 1.0);

    ASSERT_EQ(result.markers[1].color.r, 0.0);
    ASSERT_EQ(result.markers[1].color.g, 0.0);
    ASSERT_EQ(result.markers[1].color.b, 1.0);
    ASSERT_EQ(result.markers[1].color.a, 1.0);

    // Verify clearing of old markers
    std::unique_ptr<carma_perception_msgs::msg::ExternalObjectList> msg_empty = std::make_unique<carma_perception_msgs::msg::ExternalObjectList>();
    
    worker_node->external_objects_callback(move(msg_empty)); // Manually drive topic callbacks

    
    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    ASSERT_EQ(result.markers.size(), 2u);
    ASSERT_EQ(result.markers[0].action, visualization_msgs::msg::Marker::DELETE);
    ASSERT_EQ(result.markers[1].action, visualization_msgs::msg::Marker::DELETE);

}

TEST(Testobject_visualizer, roadway_obstacles_test){

    rclcpp::NodeOptions options;
    auto worker_node = std::make_shared<object_visualizer::Node>(options);

    worker_node->configure(); //Call configure state transition
    worker_node->activate();  //Call activate state transition to get not read for runtime

    std::unique_ptr<carma_perception_msgs::msg::RoadwayObstacleList> msg = std::make_unique<carma_perception_msgs::msg::RoadwayObstacleList>();

    carma_perception_msgs::msg::RoadwayObstacle obs;
    carma_perception_msgs::msg::RoadwayObstacle obs2;
    carma_perception_msgs::msg::RoadwayObstacle obs3;

    carma_perception_msgs::msg::ExternalObject obj;
    obj.id = 1;
    obj.header.frame_id = "map";
    obj.pose.pose.position.x = 1.0;
    obj.pose.pose.position.y = 2.0;
    obj.pose.pose.position.z = 3.0;
    obj.pose.pose.orientation.x = 0.0;
    obj.pose.pose.orientation.y = 1.0;
    obj.pose.pose.orientation.z = 0.0;
    obj.pose.pose.orientation.w = 1.0;

    obj.size.x = 4.0;
    obj.size.y = 5.0;
    obj.size.z = 6.0;

    obj.object_type = carma_perception_msgs::msg::ExternalObject::PEDESTRIAN;

    obs.object = obj;

    msg->roadway_obstacles.push_back(obs);

    auto obj2 = obj;
    obj2.id = 2;
    obj2.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;

    obs2.object = obj2;
    obs2.connected_vehicle_type.type = carma_perception_msgs::msg::ConnectedVehicleType::CONNECTED;

    msg->roadway_obstacles.push_back(obs2);

    auto obj3 = obj;
    obj3.id = 3;
    obj3.object_type = carma_perception_msgs::msg::ExternalObject::SMALL_VEHICLE;

    obs3.connected_vehicle_type.type = carma_perception_msgs::msg::ConnectedVehicleType::NOT_CONNECTED;

    msg->roadway_obstacles.push_back(obs3);



    // create subscription to receive message after publish

    visualization_msgs::msg::MarkerArray result;
    auto sub = worker_node->create_subscription<visualization_msgs::msg::MarkerArray>("roadway_obstacles_viz", 1,
        [&](visualization_msgs::msg::MarkerArray::UniquePtr msg)
        {
            result = *msg;
        });


    worker_node->roadway_obstacles_callback(move(msg)); // Manually drive topic callbacks

    
    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger


    // Check that the result is correct
    ASSERT_EQ(result.markers.size(), 3u);
    ASSERT_EQ(result.markers[0].id, 0);
    ASSERT_EQ(result.markers[1].id, 1);
    ASSERT_EQ(result.markers[2].id, 2);

    ASSERT_EQ(result.markers[0].header.frame_id.compare("map"), 0);
    ASSERT_EQ(result.markers[0].pose.position.x, obj.pose.pose.position.x);
    ASSERT_EQ(result.markers[0].pose.position.y, obj.pose.pose.position.y);
    ASSERT_EQ(result.markers[0].pose.position.z, obj.pose.pose.position.z);
    ASSERT_EQ(result.markers[0].pose.orientation.x, obj.pose.pose.orientation.x);
    ASSERT_EQ(result.markers[0].pose.orientation.y, obj.pose.pose.orientation.y);
    ASSERT_EQ(result.markers[0].pose.orientation.z, obj.pose.pose.orientation.z);
    ASSERT_EQ(result.markers[0].pose.orientation.w, obj.pose.pose.orientation.w);

    ASSERT_NEAR(result.markers[0].scale.x, obj.size.x * 2.0, 0.00001);
    ASSERT_NEAR(result.markers[0].scale.y, obj.size.y * 2.0, 0.00001);
    ASSERT_NEAR(result.markers[0].scale.z, obj.size.z * 2.0, 0.00001);

    ASSERT_EQ(result.markers[0].type, visualization_msgs::msg::Marker::CUBE);

    ASSERT_EQ(result.markers[0].action, visualization_msgs::msg::Marker::ADD);
    ASSERT_EQ(result.markers[1].action, visualization_msgs::msg::Marker::ADD);

    ASSERT_EQ(result.markers[0].color.r, 1.0);
    ASSERT_EQ(result.markers[0].color.g, 0.0);
    ASSERT_EQ(result.markers[0].color.b, 0.0);
    ASSERT_EQ(result.markers[0].color.a, 1.0);

    ASSERT_EQ(result.markers[1].color.r, 0.0);
    ASSERT_EQ(result.markers[1].color.g, 1.0);
    ASSERT_EQ(result.markers[1].color.b, 0.0);
    ASSERT_EQ(result.markers[1].color.a, 1.0);

    ASSERT_EQ(result.markers[2].color.r, 0.0);
    ASSERT_EQ(result.markers[2].color.g, 0.0);
    ASSERT_EQ(result.markers[2].color.b, 1.0);
    ASSERT_EQ(result.markers[2].color.a, 1.0);

    // Verify clearing of old markers
    std::unique_ptr<carma_perception_msgs::msg::RoadwayObstacleList> msg_empty = std::make_unique<carma_perception_msgs::msg::RoadwayObstacleList>();
    
    worker_node->roadway_obstacles_callback(move(msg_empty)); // Manually drive topic callbacks

    
    // Provide some time for publication to occur
    std::this_thread::sleep_for(std::chrono::seconds(2));

    rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

    ASSERT_EQ(result.markers.size(), 3u);
    ASSERT_EQ(result.markers[0].action, visualization_msgs::msg::Marker::DELETE);
    ASSERT_EQ(result.markers[1].action, visualization_msgs::msg::Marker::DELETE);
    ASSERT_EQ(result.markers[2].action, visualization_msgs::msg::Marker::DELETE);

}

int main(int argc, char ** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    //Initialize ROS
    rclcpp::init(argc, argv);

    bool success = RUN_ALL_TESTS();

    //shutdown ROS
    rclcpp::shutdown();

    return success;
} 