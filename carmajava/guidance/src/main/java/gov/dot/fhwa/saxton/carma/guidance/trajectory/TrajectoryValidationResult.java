/*
 * Copyright (C) 2018 LEIDOS.
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

/**
 * DTO class for representing validaiton success or containing more information
 * about an error or failure that ocurred
 */
public class TrajectoryValidationResult {
  protected TrajectoryValidationError error = null;

  /**
   * Construct a TrajectoryValidationResult that indicates success
   */
  public TrajectoryValidationResult() {
  }

  /**
   * Construct a TrajectoryValidationResult that indicates failure
   */
  public TrajectoryValidationResult(TrajectoryValidationError error) {
    this.error = error;
  }

  /**
   * Get the error if there was one, return null o.w.
   */
  public TrajectoryValidationError getError() {
    return error;
  }

  /**
   * True if validation was successful, false o.w.
   */
  public boolean getSuccess() {
    return error == null;
  }
}
