/*
 * Copyright (C) 2018-2019 LEIDOS.
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
package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import org.ros.message.Duration;

/**
 * Specifies the data input interface for a SpeedHarmonizationManeuver. This keeps knowledge of the ROS network out of the remainder
 * of the package.
 */
public interface ISpeedHarmInputs {
  /**
   * Returns the most recent speed command which the providing object would like to have executed.
   * Units: m/s
   *
   * @return Speed command
   */
  double getSpeedCommand();

  /**
   * Returns the most recent acceleration command which the providing object would like to have executed.
   * Units: m/s^2
   * Acceleration limit should be positive
   *
   * @return Acceleration command
   */
  double getMaxAccelLimit();

  /**
   * Returns the time since the providing object last received or calculated a new speed or acceleration command.
   *
   * @return Time of last update
   */
  Duration getTimeSinceLastUpdate();
}
