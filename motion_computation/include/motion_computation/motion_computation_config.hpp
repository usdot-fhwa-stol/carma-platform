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

#include <iostream>
#include <vector>

namespace motion_computation {

/**
 * \brief Stuct containing the algorithm configuration values for motion_computation
 */
struct Config {
  double prediction_time_step = 0.1;     // Time between predicted states (in seconds)
  double mobility_path_time_step = 0.1;  // Time between received mobility path predicted states (in seconds)
  double prediction_period = 2.0;        // Period of prediction (in seconds)
  double cv_x_accel_noise = 9.0;         // CV Model X-Axis Acceleration Noise
  double cv_y_accel_noise = 9.0;         // CV Model Y-Axis Acceleration Noise
  double prediction_process_noise_max = 1000.0;  // Maximum expected process noise; used for mapping noise to confidence in [0,1] range
  double prediction_confidence_drop_rate = 0.95;  // Percentage of initial confidence to propagate to next time step

  // If true then BSM messages will be converted to ExternalObjects.
  // If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)
  bool enable_bsm_processing = false;

  // If true then PSM messages will be converted to ExternalObjects.
  // If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)
  bool enable_psm_processing = false;

  // If true then MobilityPath messages will be converted to ExternalObjects.
  // If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)
  bool enable_mobility_path_processing = true;

  // If true then ExternalObjects generated from sensor data will be processed.
  // If other object sources are enabled, they will be synchronized but no fusion will occur (objects may be duplicated)
  bool enable_sensor_processing = false;

  // Stream operator for this config
  friend std::ostream &operator<<(std::ostream &output, const Config &c) {
    output << "motion_computation::Config { " << std::endl
           << "prediction_time_step: " << c.prediction_time_step << std::endl
           << "mobility_path_time_step: " << c.mobility_path_time_step << std::endl
           << "prediction_period: " << c.prediction_period << std::endl
           << "cv_x_accel_noise: " << c.cv_x_accel_noise << std::endl
           << "cv_y_accel_noise: " << c.cv_y_accel_noise << std::endl
           << "prediction_process_noise_max: " << c.prediction_process_noise_max << std::endl
           << "prediction_confidence_drop_rate: " << c.prediction_confidence_drop_rate << std::endl
           << "enable_bsm_processing: " << c.enable_bsm_processing << std::endl
           << "enable_psm_processing: " << c.enable_psm_processing << std::endl
           << "enable_mobility_path_processing: " << c.enable_mobility_path_processing << std::endl
           << "enable_sensor_processing: " << c.enable_sensor_processing << std::endl
           << "}" << std::endl;
    return output;
  }
};

}  // namespace motion_computation