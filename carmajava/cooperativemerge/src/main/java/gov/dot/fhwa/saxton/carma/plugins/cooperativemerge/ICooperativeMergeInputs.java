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
package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

import org.ros.message.Duration;

/**
 * Specifies the data input interface for a CooperativeMergeManeuver.
 */
public interface ICooperativeMergeInputs {
  /**
   * Returns the most recent speed command which the providing object would like to have executed.
   * Units: m/s
   *
   * @return Speed command
   */
  double getSpeedCommand();

  /**
   * Returns the most recent steering command which the providing object would like to have executed.
   * Units: radians
   *
   * @return Steering command
   */
  double getSteeringCommand();

  /**
   * Returns the most recent acceleration command which the providing object would like to have executed.
   * Units: m/s^2
   * Acceleration limit should be positive
   *
   * @return Acceleration command
   */
  double getMaxAccelLimit();
}
