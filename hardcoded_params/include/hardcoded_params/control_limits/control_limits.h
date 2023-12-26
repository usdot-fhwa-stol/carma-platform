#pragma once

/*
 * Copyright (C) 2020-2021 LEIDOS.
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

namespace hardcoded_params
{
namespace control_limits
{
/**
 * The maximum allowable longitudinal velocity of the vehicle in m/s
 */
constexpr double MAX_LONGITUDINAL_VELOCITY_MPS = 35.7632;
constexpr double MAX_LONGITUDINAL_ACCEL_MPS2 = 7.0;
}
}  // namespace hardcoded_params
