/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import java.util.List;

/**
 * DTO class for describing an error or failure during Trajectory validation
 */
public class TrajectoryValidationError {
  protected String errorDescriptor;
  protected List<IManeuver> offendingManeuvers;

  public TrajectoryValidationError(String errorDescriptor, List<IManeuver> offendingManeuvers) {
    this.errorDescriptor = errorDescriptor;
    this.offendingManeuvers = offendingManeuvers;
  }

  /**
   * Get a human-readable description of the validation failure that ocurred.
   */
  public String getErrorDescriptor() {
    return errorDescriptor;
  }

  /**
   * Get a list of the Maneuver instances that violated the constraint or otherwise
   * cause the validation failure
   */
  public List<IManeuver> getOffendingManeuvers() {
    return offendingManeuvers;
  }
}
