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

#include <iostream>

namespace frame_transformer
{

  /**
   * \brief Stuct containing the algorithm configuration values for frame_transformer
   */
  struct Config
  {
    //! The target coordinate frame id to transform data into
    std::string target_frame = "base_link";

    //! Message type which will be transformed
    std::string message_type = "sensor_msgs/PointCloud2";
    
    //! Queue size for publishers and subscribers
    int queue_size = 1;

    //! Timeout in ms for transform lookup. A value of 0 means lookup will occur once without blocking if it fails.
    int timeout = 0;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "frame_transformer::Config { " << std::endl
           << "target_frame: " << c.target_frame << std::endl
           << "message_type: " << c.message_type << std::endl
           << "queue_size: " << c.queue_size << std::endl
           << "timeout: " << c.timeout << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // frame_transformer