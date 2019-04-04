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
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.LongitudinalManeuver;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ManeuverType;
import java.util.ArrayList;

/**
 * TrajectoryValidationConstraint implementation responsible for ensuring that maneuvers
 * in a planned trajectory allow for enough space in between to smoothly blend the
 * maneuvers together
 */
public class MinimumDistanceBetweenManeuversConstraint implements TrajectoryValidationConstraint {

  private IManeuver lastLateralManeuver = null;
  private IManeuver lastLongitudinalManeuver = null;
  private boolean valid = true;
  private List<IManeuver> offendingManeuvers = new ArrayList<>();
  private double minimumDistanceBetweenManeuvers;

  public MinimumDistanceBetweenManeuversConstraint(double minDist) {
    this.minimumDistanceBetweenManeuvers = minDist;
  }

	/**
	 * Current workaround for maneuvers not providing their own types
	 * TODO: Replace with more robust system of differentiating Maneuvers
	 */
	private ManeuverType getType(IManeuver maneuver) {
		if (maneuver instanceof LongitudinalManeuver) {
			return ManeuverType.LONGITUDINAL;
		} else {
			return ManeuverType.LATERAL;
		}
	}

	@Override
	public void visit(IManeuver maneuver) {
    if (!valid) {
      return;
    }

    if (getType(maneuver) == ManeuverType.LATERAL) {
      if (lastLateralManeuver != null) {
        if (Math.abs(lastLateralManeuver.getEndDistance() - maneuver.getStartDistance())
            < minimumDistanceBetweenManeuvers) {
          valid = false;
          offendingManeuvers.add(lastLateralManeuver);
          offendingManeuvers.add(maneuver);
        }
      }

      lastLateralManeuver = maneuver;
    }

    if (getType(maneuver) == ManeuverType.LONGITUDINAL) {
      if (lastLongitudinalManeuver != null) {
        if (Math.abs(lastLongitudinalManeuver.getEndDistance() - maneuver.getStartDistance())
            < minimumDistanceBetweenManeuvers) {
          valid = false;
          offendingManeuvers.add(lastLongitudinalManeuver);
          offendingManeuvers.add(maneuver);
        }
      }

      lastLongitudinalManeuver = maneuver;
    }
	}

	@Override
	public TrajectoryValidationResult getResult() {
    if (valid) {
      reset();
      return new TrajectoryValidationResult();
    } else {
      TrajectoryValidationResult out = new TrajectoryValidationResult(new TrajectoryValidationError("Minimum distance between maneuvers not observed!", offendingManeuvers));

      reset();
      return out;
    }
	}

  /**
   * Reset the state so the constraint can accept another Trajectory
   */
  private void reset() {
    valid = true;
    lastLongitudinalManeuver = null;
    lastLateralManeuver = null;
    offendingManeuvers = new ArrayList<>();
  }
}
