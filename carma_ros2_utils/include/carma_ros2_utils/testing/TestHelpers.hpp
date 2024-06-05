#pragma once
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
#include <chrono>
#include <thread>
#include <atomic>
namespace carma_ros2_utils
{
namespace testing
{
/**
 * \brief Helper function which waits until the provided atomic matches the expected value or the timeout expires.
 *        This function is NOT intended for production use as it does not support simulated time.
 * 
 * \param timeout_s The timeout in seconds
 * \param expected The expected value to occur before the timeout
 * \param actual The atomic variable to compare against expected until timeout occurs
 */
template <class T1, class T2>
inline bool waitForEqOrTimeout(double timeout_s, T1 expected, std::atomic<T2>& actual)
{
  auto start = std::chrono::system_clock::now();
  std::chrono::duration<double, std::ratio<1, 1>> sec(timeout_s);
  auto elapsed_seconds = std::chrono::duration<double>(std::chrono::system_clock::now() - start);

  while (elapsed_seconds < sec)
  {
    
    if (actual.load() == expected)
    {
      return true;
    }
    elapsed_seconds = std::chrono::system_clock::now() - start;
    auto period = std::chrono::milliseconds(10);
    std::this_thread::sleep_for(period);
  }
  return false;
}
}  // namespace testing
}  // namespace carma_utils