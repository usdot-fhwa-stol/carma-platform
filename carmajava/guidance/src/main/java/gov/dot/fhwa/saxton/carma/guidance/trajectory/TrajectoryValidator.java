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
import java.util.ArrayList;

public class TrajectoryValidator {
  protected List<TrajectoryValidationConstraint> constraints = new ArrayList<>();

  public void addValidationConstraint(TrajectoryValidationConstraint constraint) {
    constraints.add(constraint);
  }

  public boolean validate(Trajectory traj) {
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
