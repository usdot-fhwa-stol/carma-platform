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

#include "frame_transformer_base.hpp"
#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <chrono>
#include <gtest/gtest_prod.h>

namespace frame_transformer
{
  namespace std_ph = std::placeholders;
  using std_ms = std::chrono::milliseconds;

  /**
   * \brief Class template for data transformers which use the tf2_ros::Buffer.transform() method to perform transforms on ros messages.
   *        The class sets up publishers and subscribers using the provided node for the specified message type.
   *        The specified message type must have a tf2_ros::Buffer.doTransform specialization for its type in order for it to compile.
   * 
   * \tparam The message type to transform. Must be supported by tf2_ros::Buffer.doTransform
   */
  template <class T>
  class Transformer : public TransformerBase // TransformerBase allows for type polymorphism in the containing Node and constructor enforcement
  {

  protected:
    // Subscribers
    carma_ros2_utils::SubPtr<T> input_sub_;

    // Publishers
    carma_ros2_utils::PubPtr<T> output_pub_;


  public:

    /**
     * \brief Constructor which sets up the required publishers and subscribers
     * 
     * See comments in TransformerBase for parameter descriptions
     */ 
    Transformer(Config config, std::shared_ptr<tf2_ros::Buffer> buffer, std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node)
    : TransformerBase(config, buffer, node) {

      input_sub_ = node_->create_subscription<T>("input", config_.queue_size,
                                                  std::bind(&Transformer::input_callback, this, std_ph::_1));

      // Setup publishers
      output_pub_ = node_->create_publisher<T>("output", config_.queue_size);
    }


    /**
     * \brief Helper method which takes in an input message and transforms it to the provided frame.
     *        Returns false if the provided timeout is exceeded for getting the transform or the transform could not be computed
     * 
     * \param in The input message to transform
     * \param[out] out The preallocated location for the output message
     * \param target_frame The frame the out message data will be in
     * \param timeout A timeout in ms which if exceeded will result in a false return and invalid out value. This call may block for this period.
     *                If set to zero, then lookup will be attempted only once. 
     * 
     * \return True if transform succeeded, false if timeout exceeded or transform could not be performed. 
     */ 
    bool transform(const T &in, T &out, const std::string &target_frame, const std_ms timeout)
    {

      try
      {
        buffer_->transform(in, out, target_frame, timeout);
      }
      catch (tf2::TransformException &ex)
      {
        std::string error = ex.what();
        error = "Failed to get transform with exception: " + error;
        auto& clk = *node_->get_clock(); // Separate reference required for proper throttle macro call
        RCLCPP_WARN_THROTTLE(node_->get_logger(), clk, 1000, error);

        return false;
      }

      return true;
    }

    /**
     * \brief Callback for input data. Transforms the data then republishes it
     * 
     * NOTE: This method can be specialized for unique preallocation approaches for large messages such as point clouds or images
     * 
     * \param in_msg The input message to transform
     */ 
    void input_callback(std::unique_ptr<T> in_msg)
    {
      T out_msg;

      if (!transform(*in_msg, out_msg, config_.target_frame, std_ms(config_.timeout)))
      {
        return;
      }

      output_pub_->publish(out_msg);
    }

    // Unit Test Accessors
    FRIEND_TEST(frame_transformer_test, transform_test);
  };

  // Specialization of input_callback for PointCloud2 messages to allow for preallocation of points vector
  // This is done due to the large size of that data set
  template <>
  inline void Transformer<sensor_msgs::msg::PointCloud2>::input_callback(std::unique_ptr<sensor_msgs::msg::PointCloud2> in_msg) {

    sensor_msgs::msg::PointCloud2 out_msg;
    out_msg.data.reserve(in_msg->data.size()); // Preallocate points vector
    

    if (!transform(*in_msg, out_msg, config_.target_frame, std_ms(config_.timeout)))
    {
      return;
    }

    // The following if block is added purely for ensuring consistency with Autoware.Auto (prevent "Malformed PointCloud2" error from ray_ground_filter)
    // It's a bit out of scope for this node to have this functionality here, 
    // but the alternative is to modify a 3rd party driver, an Autoware.Auto component, or make a new node just for this.
    // Therefore, the logic will live here until such a time as a better location presents itself.
    if (out_msg.height == 1) // 1d point cloud
    {
      out_msg.row_step = out_msg.data.size();
    }

    output_pub_->publish(out_msg);
  }

}