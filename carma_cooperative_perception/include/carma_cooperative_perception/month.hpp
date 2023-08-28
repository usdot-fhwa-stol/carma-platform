#ifndef CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_
#define CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_

/*
 * Copyright 2023 Leidos
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

namespace carma_cooperative_perception
{

// Should be replaced with std::chrono::month when CARMA targets C++20
enum class Month
{
  kJanuary = 1
};

static inline auto month_from_number(int month)
{
  switch (month) {
    case 1:
      return Month::kJanuary;
    default:
      return Month::kJanuary;
  }
}

}  // namespace carma_cooperative_perception

#endif  // CARMA_COOPERATIVE_PERCEPTION_MONTH_HPP_
