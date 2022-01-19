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

#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <thread>
#include <future>
#include <thread>
#include <chrono>
// Using deprecated sensor_msgs/PointCloud to convert to PointCloud2 message in unit tests
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud_conversion.hpp>

#include "frame_transformer/frame_transformer_node.hpp"

namespace frame_transformer
{

    TEST(frame_transformer_test, transform_test)
    {

        std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
        remaps.push_back("--ros-args");
        remaps.push_back("-r");
        remaps.push_back("output:=/transform_test/output");
        remaps.push_back("-r");
        remaps.push_back("input:=/transform_test/input");
        remaps.push_back("-r");
        remaps.push_back("__node:=transform_test_frame_transformer");

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        options.arguments(remaps);
        auto worker_node = std::make_shared<frame_transformer::Node>(options);

        // Set the parameters for this test
        worker_node->set_parameter(rclcpp::Parameter("message_type", "geometry_msgs/PointStamped"));
        worker_node->set_parameter(rclcpp::Parameter("target_frame", "base_link"));
        worker_node->set_parameter(rclcpp::Parameter("queue_size", 1));
        worker_node->set_parameter(rclcpp::Parameter("timeout", 0));

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that the node has been properly configured
        ASSERT_TRUE(!!worker_node->buffer_);
        ASSERT_TRUE(!!worker_node->listener_);
        ASSERT_TRUE(!!worker_node->transformer_);

        std::unique_ptr<geometry_msgs::msg::PointStamped> msg = std::make_unique<geometry_msgs::msg::PointStamped>();
        msg->header.frame_id = "velodyne";
        msg->header.stamp = worker_node->now();
        msg->point.x = 1.0;
        msg->point.y = 0.0;
        msg->point.z = 0.0;

        // Create a transform
        geometry_msgs::msg::TransformStamped base_link_tf;
        base_link_tf.header.frame_id = "base_link";
        base_link_tf.child_frame_id = "velodyne";
        base_link_tf.transform.translation.x = 1.0;
        base_link_tf.transform.translation.y = 0.0;
        base_link_tf.transform.translation.z = 0.0;
        base_link_tf.transform.rotation.x = 0.0;
        base_link_tf.transform.rotation.y = 0.0;
        base_link_tf.transform.rotation.z = 0.0;
        base_link_tf.transform.rotation.w = 1.0;

        worker_node->buffer_->setTransform(base_link_tf, "test_authority", true);

        // create subscription to receive message after publish

        geometry_msgs::msg::PointStamped result;
        auto sub = worker_node->create_subscription<geometry_msgs::msg::PointStamped>("/transform_test/output", 1,
            [&](geometry_msgs::msg::PointStamped::UniquePtr msg)
            {
                result = *msg;
            });

        { // Protective scope for raw pointer access from unique_ptr
        
            ASSERT_NE(nullptr, worker_node->transformer_.get());

            // Raw pointer access to transformer for downcasting unique_ptr
            TransformerBase* transformer = worker_node->transformer_.get();
            Transformer<geometry_msgs::msg::PointStamped>* cast_transformer = static_cast<Transformer<geometry_msgs::msg::PointStamped>*>(transformer);
            
            // Trigger the callback
            cast_transformer->input_callback(std::move(msg));
        }
        // Provide some time for publication to occur
        std::this_thread::sleep_for(std::chrono::seconds(2));

        rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

        // Verify that the message was transformed
        ASSERT_EQ(result.header.frame_id, "base_link");
        ASSERT_NEAR(result.point.x, 2.0, 1e-6);
        ASSERT_NEAR(result.point.y, 0.0, 1e-6);
        ASSERT_NEAR(result.point.z, 0.0, 1e-6);
    }

    // NOTE: This test uses deprecated sensor_msgs/PointCloud (not PointCloud2) to convert to PointCloud2 messages
    //       This results in a deprecated warning message at compile time. 
    //       If the ROS2 version is upgrade beyond foxy this test will need to be updated
    TEST(frame_transformer_test, point_cloud_transform_test)
    {

        std::vector<std::string> remaps; // Remaps to keep topics separate from other tests
        remaps.push_back("--ros-args");
        remaps.push_back("-r");
        remaps.push_back("output:=/point_cloud_transform_test/output");
        remaps.push_back("-r");
        remaps.push_back("input:=/point_cloud_transform_test/input");
        remaps.push_back("-r");
        remaps.push_back("__node:=point_cloud_transform_test_frame_transformer");


        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
        options.arguments(remaps);
        auto worker_node = std::make_shared<frame_transformer::Node>(options);

        // Set the parameters for this test
        worker_node->set_parameter(rclcpp::Parameter("message_type", "sensor_msgs/PointCloud2"));
        worker_node->set_parameter(rclcpp::Parameter("target_frame", "base_link"));
        worker_node->set_parameter(rclcpp::Parameter("queue_size", 1));
        worker_node->set_parameter(rclcpp::Parameter("timeout", 0));

        worker_node->configure(); //Call configure state transition
        worker_node->activate();  //Call activate state transition to get not read for runtime

        // Verify that the node has been properly configured
        ASSERT_TRUE(!!worker_node->buffer_);
        ASSERT_TRUE(!!worker_node->listener_);
        ASSERT_TRUE(!!worker_node->transformer_);


        // Build message

        sensor_msgs::msg::PointCloud point_cloud;
        point_cloud.header.frame_id = "velodyne";
        point_cloud.header.stamp = worker_node->now();

        geometry_msgs::msg::Point32 p1;
        p1.x = 1.0;
        p1.y = 2.0;
        p1.z = 3.0;

        geometry_msgs::msg::Point32 p2;
        p2.x = 4.0;
        p2.y = 5.0;
        p2.z = 6.0;

        point_cloud.points.push_back(p1);
        point_cloud.points.push_back(p2);

        std::unique_ptr<sensor_msgs::msg::PointCloud2> msg = std::make_unique<sensor_msgs::msg::PointCloud2>();

        sensor_msgs::convertPointCloudToPointCloud2(point_cloud, *msg);




        // Create a transform
        geometry_msgs::msg::TransformStamped base_link_tf;
        base_link_tf.header.frame_id = "base_link";
        base_link_tf.child_frame_id = "velodyne";
        base_link_tf.transform.translation.x = 1.0;
        base_link_tf.transform.translation.y = 0.0;
        base_link_tf.transform.translation.z = 0.0;
        base_link_tf.transform.rotation.x = 0.0;
        base_link_tf.transform.rotation.y = 0.0;
        base_link_tf.transform.rotation.z = 0.0;
        base_link_tf.transform.rotation.w = 1.0;

        worker_node->buffer_->setTransform(base_link_tf, "test_authority", true);

        // create subscription to receive message after publish

        sensor_msgs::msg::PointCloud2 result;
        auto sub = worker_node->create_subscription<sensor_msgs::msg::PointCloud2>("/point_cloud_transform_test/output", 1,
            [&](sensor_msgs::msg::PointCloud2::UniquePtr msg)
            {
                result = *msg;
            });

        { // Protective scope for raw pointer access from unique_ptr
        
            ASSERT_NE(nullptr, worker_node->transformer_.get());

            // Raw pointer access to transformer for downcasting unique_ptr
            TransformerBase* transformer = worker_node->transformer_.get();
            Transformer<sensor_msgs::msg::PointCloud2>* cast_transformer = static_cast<Transformer<sensor_msgs::msg::PointCloud2>*>(transformer);
            
            // Trigger the callback
            cast_transformer->input_callback(std::move(msg));
        }
        // Provide some time for publication to occur
        std::this_thread::sleep_for(std::chrono::seconds(2));

        rclcpp::spin_some(worker_node->get_node_base_interface()); // Spin current queue to allow for subscription callback to trigger

        // Verify that the message was transformed
        
        sensor_msgs::msg::PointCloud readable_result;
        sensor_msgs::convertPointCloud2ToPointCloud(result, readable_result);

        ASSERT_EQ(readable_result.header.frame_id, "base_link");
        ASSERT_EQ(readable_result.points.size(), 2u);
        
        ASSERT_NEAR(readable_result.points[0].x, 2.0, 1e-6);
        ASSERT_NEAR(readable_result.points[0].y, 2.0, 1e-6);
        ASSERT_NEAR(readable_result.points[0].z, 3.0, 1e-6);

        ASSERT_NEAR(readable_result.points[1].x, 5.0, 1e-6);
        ASSERT_NEAR(readable_result.points[1].y, 5.0, 1e-6);
        ASSERT_NEAR(readable_result.points[1].z, 6.0, 1e-6);

    }
}

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