#pragma once
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

#include <string>

namespace pose_to_tf
{
    /**
   * \brief Stuct containing the frame configuration values for pose_to_tf
   */
   
  struct PoseToTF2Config
  {
    std::string child_frame = "base_link";
    std::string default_parent_frame = "map";

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const PoseToTF2Config &c)
    {
      output << "pose_to_tf::PoseToTF2Config { " << std::endl
           << "child_frame: " << c.child_frame << std::endl
           << "default_parent_frame: " << c.default_parent_frame << std::endl
           << "}" << std::endl;
      return output;
    }
  };
} // namespace pose_to_tf