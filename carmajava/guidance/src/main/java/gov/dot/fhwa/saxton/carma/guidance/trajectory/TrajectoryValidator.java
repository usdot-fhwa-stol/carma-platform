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

package gov.dot.fhwa.saxton.carma.guidance.trajectory;

import java.util.List;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import java.util.ArrayList;

/**
 * TrajectoryValidator is responsible for using different criteria to evaluate
 * the validity and viability of a fully or partially planned Trajectory.
 */
public class TrajectoryValidator {
  protected List<TrajectoryValidationConstraint> constraints = new ArrayList<>();

  /**
   * Register a new validation constraint with the TrajectoryValidator
   */
  public synchronized void addValidationConstraint(TrajectoryValidationConstraint constraint) {
    constraints.add(constraint);
  }

  /**
   * Use the currently registered validation constraints to process a candidate
   * Trajectory.
   * <p>
   * The result is the logical AND of the results of all the registered
   * TrajectoryValidationConstraints associated with this TrajectoryValidator.
   */
  public synchronized boolean validate(Trajectory traj) {
    boolean valid = true;
    for (IManeuver m : traj.getManeuvers()) {
      for (TrajectoryValidationConstraint c : constraints) {
        c.visit(m);
      }
    }

    for (TrajectoryValidationConstraint c : constraints) {
      valid = valid & c.getResult().getSuccess();
    }

    return valid;
  }
}
