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

import java.util.List;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import java.util.ArrayList;

/**
 * TrajectoryValidator is responsible for using different criteria to evaluate
 * the validity and viability of a fully or partially planned Trajectory.
 */
public class TrajectoryValidator {
  protected ILogger log = LoggerManager.getLogger();
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
      TrajectoryValidationResult result = c.getResult();
      valid = valid && result.getSuccess();
      if (!result.getSuccess()) {
        // Log our failure state, including as much detail on the failure as possible
        log.warn(String.format("Trajectory from [%.02f, %.02f) failed validation on constraint: %s for reason: %s!",
        traj.getStartLocation(),
        traj.getEndLocation(),
        c.getClass().getSimpleName(),
        result.getError().getErrorDescriptor()));

        StringBuilder builder = new StringBuilder();
        builder.append("{");
        for (IManeuver m : result.getError().getOffendingManeuvers()) {
          builder.append(String.format("%s@[%.02f, %.02f),",
          m.getClass().getSimpleName(),
          m.getStartDistance(),
          m.getEndDistance()));
        }
        builder.setCharAt(builder.length() - 1, '}');

        log.warn("Offending maneuvers: " + builder.toString());
      }
    }

    return valid;
  }
}
