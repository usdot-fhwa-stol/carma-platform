/*
 * Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import org.ros.message.Time;

/**
 * Defines an interface to all complex maneuvers that cannot be fully planned in advance
 * <p>
 * Trajectory execution must always end after a complex maneuver. No complex maneuvers
 * are provided by the base CARMA Platform. They must be defined by the respective 
 * plugins which wish to use them.
 */
public interface IComplexManeuver extends IManeuver {
  /**
   * Get the human readable name of this complex maneuver
   */
  String getManeuverName();

  /**
   * Get the shortest possible time the maneuver might take to complete, in seconds
   * 
   * Presently only used as an expectation rather than a hard limit but this is subject to change.
   */
  Time getMinCompletionTime();

  /**
   * Get the longest possible time the maneuver might take to complete, in seconds
   * 
   * Presently only used as an expectation rather than a hard limit but this is subject to change.
   */
  Time getMaxCompletionTime();

  /**
   * Get the highest speed this complex maneuver is expected to command, in m/s
   * 
   * Presently only used as an expectation rather than a hard limit but this is subject to change.
   */
  double getMaxExpectedSpeed();

  /**
   * Get the lowest speed this complex maneuver is expected to command, in m/s
   * 
   * Presently only used as an expectation rather than a hard limit but this is subject to change.
   */
  double getMinExpectedSpeed();
}
