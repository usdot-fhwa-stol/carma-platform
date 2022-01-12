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

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <gtest/gtest_prod.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "frame_transformer/frame_transformer_config.hpp"
#include "frame_transformer/frame_transformer_base.hpp"
#include "frame_transformer/frame_transformer.hpp"

namespace frame_transformer
{

  /**
   * \brief Node which subcribes to and input topic, transforms the data into a new frame and republishes it onto an output topic.
   * 
   */
  class Node : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:

    //! Node configuration
    Config config_;

    //! Pointer to a transformer which will setup its own pub/subs to perform the transformation  
    std::unique_ptr<TransformerBase> transformer_;

    //! tf2 Buffer to store transforms broadcast on the system
    std::shared_ptr<tf2_ros::Buffer> buffer_;

    //! tf2 Listener to subscribe to system transform broadcasts
    std::shared_ptr<tf2_ros::TransformListener> listener_;

  public:
    /**
     * \brief Node constructor 
     */
    explicit Node(const rclcpp::NodeOptions &);


    /**
     * \brief Factory method which returns an initialized pointer to a TransformerBase. 
     * 
     * ASSUMPTION: this->config_ and this->buffer_ are already initialized
     * 
     * \return Initialized pointer to a TransformBase object which contains the required pub/sub for transformations to occur  
     */ 
    std::unique_ptr<TransformerBase> build_transformer();

    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &);

    carma_ros2_utils::CallbackReturn handle_on_cleanup(const rclcpp_lifecycle::State &);

    // Unit Test Accessors
    FRIEND_TEST(frame_transformer_test, transform_test);

  };

} // frame_transformer
