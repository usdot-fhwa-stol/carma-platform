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

#include <string>
#include <memory>
#include <tf2_ros/buffer.h>

namespace frame_transformer
{

  /**
   * \brief Transformer base class provides default member variables for tf2 lookup and ros network access. 
   *        The constructor is also private meaning it must be called by extending classes in an attempt to enforce a construction interface.
   * 
   */
  class TransformerBase
  {

  protected:

    //! Transformation configuration
    Config config_;

    //! TF2 Buffer for storing transform history and looking up transforms
    std::shared_ptr<tf2_ros::Buffer> buffer_;

    //! Containing node which provides access to the ros network
    std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node_;

  protected: // Protected to force super call in child classes
    TransformerBase(const Config& config, std::shared_ptr<tf2_ros::Buffer> buffer, std::shared_ptr<carma_ros2_utils::CarmaLifecycleNode> node) 
      : config_(config), buffer_(buffer), node_(node) {}
  };

}