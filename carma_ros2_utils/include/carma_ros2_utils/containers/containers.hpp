#pragma once
/*
 * Copyright (C) 2018-2022 LEIDOS.
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

/**
 * Containers namespace contains utility functions that can be used with C++ containers.
 */

#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace carma_ros2_utils
{
namespace containers
{
/**
 * \brief Downsamples an input vector by saving only each nth element.
 * For example, given an input vector of { 0, 1, 2, 3, 4, 5 } and n = 2
 * The output vector will be {0, 2, 4}
 * 
 * \param input The input vector to downsample
 * \param n The count of the elements to save
 * \param include_last_point If true, append the last point
          NOTE: default is true as this function is only used for trajectory generation 
          and this keeps the lanelet's boundaries consistent no matter the downsample ratio
 * \return The downsampled vector
 */ 
template <class T, class A = std::allocator<T>>
std::vector<T, A> downsample_vector(const std::vector<T, A>& input, unsigned int n, bool include_last_point = true)
{
  std::vector<T, A> output;

  if (n == 0 || input.empty()) {
    return output;
  }

  output.reserve((input.size() / n) + 1);

  bool included_last_point = false;
  for (size_t i = 0; i < input.size(); i += n)
  {
    output.push_back(input[i]);
    if (i == input.size() - 1)
    {
      included_last_point = true;
    }
  }

  if (include_last_point && !included_last_point)
  {
    output.push_back(input.back());
  }

  return output;
}
}  // namespace containers
}  // namespace carma_ros2_utils