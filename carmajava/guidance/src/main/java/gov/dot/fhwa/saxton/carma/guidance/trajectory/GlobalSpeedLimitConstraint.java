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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import java.util.ArrayList;

/**
 * GlobalSpeedLimitConstraint ensures that the planned Trajectory never exceeds a globally configured speed limit
 */
public class GlobalSpeedLimitConstraint implements TrajectoryValidationConstraint {
  protected double globalSpeedLimit;
  protected List<IManeuver> offendingManeuvers;

  public GlobalSpeedLimitConstraint(double speedLimit) {
    globalSpeedLimit = speedLimit;
    offendingManeuvers = new ArrayList<>();
  }

  @Override
  public void visit(IManeuver maneuver) {
    if (!(maneuver instanceof LongitudinalManeuver)) {
      return;
    }

    // Note: Assumes longitudinal maneuvers linearly interpolate speed over distance
    // If non-linear longitudinal maneuvers are ever implemented this logic will not handle that case
    if (maneuver.getStartSpeed() > globalSpeedLimit || maneuver.getTargetSpeed() > globalSpeedLimit) {
      offendingManeuvers.add(maneuver);
    }
  }

  @Override
  public TrajectoryValidationResult getResult() {
    if (offendingManeuvers.isEmpty()) {
      reset();
      return new TrajectoryValidationResult();
    } else {
      TrajectoryValidationResult out = new TrajectoryValidationResult(new TrajectoryValidationError(
          "Maneuvers exceed Global Speed Limit of " + globalSpeedLimit + "m/s", offendingManeuvers));
      reset();
      return out;
    }
  }

  /**
   * Reset the state so the constraint can accept another Trajectory
   */
  private void reset() {
    offendingManeuvers = new ArrayList<>();
  }
}
