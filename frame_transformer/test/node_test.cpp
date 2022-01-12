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

#include "frame_transformer/frame_transformer_node.hpp"

namespace frame_transformer
{

    TEST(frame_transformer_test, transform_test)
    {

        rclcpp::NodeOptions options;
        options.use_intra_process_comms(true);
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
        auto sub = worker_node->create_subscription<geometry_msgs::msg::PointStamped>("output", 1,
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