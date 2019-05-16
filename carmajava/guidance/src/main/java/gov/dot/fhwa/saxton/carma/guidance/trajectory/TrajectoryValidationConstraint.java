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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;

/**
 * Interface implemented by constraints for Trajectory validation
 */
public interface TrajectoryValidationConstraint {
  /**
   * Inform this TrajectoryValidationConstraint about a Maneuver in the Trajectory
   * under evaluation.
   * <p>
   * All maneuvers are passed in in order by start location but not separated by
   * Maneuver type.
   */
  public void visit(IManeuver maneuver);

  /**
   * Get the result of this constraint after it has visited any number of Maneuvers
   */
  public TrajectoryValidationResult getResult();
}
