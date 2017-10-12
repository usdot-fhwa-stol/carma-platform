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

import gov.dot.fhwa.saxton.carma.guidance.trajectory.IManeuver.ManeuverType;

import java.util.List;
import java.util.ArrayList;

public class MinimumDistanceBetweenManeuversConstraint implements TrajectoryValidationConstraint {

  public IManeuver lastLateralManeuver = null;
  public IManeuver lastLongitudinalManeuver = null;
  public boolean valid = true;
  public List<IManeuver> offendingManeuvers = new ArrayList<>();
  public double minimumDistanceBetweenManeuvers;

  public MinimumDistanceBetweenManeuversConstraint(double minDist) {
    this.minimumDistanceBetweenManeuvers = minDist;
  }

	@Override
	public void visit(IManeuver maneuver) {
    if (!valid) {
      return;
    }

    if (maneuver.getType() == ManeuverType.LATERAL) {
      if (lastLateralManeuver != null) {
        if (Math.abs(lastLateralManeuver.getEndLocation() - maneuver.getStartLocation())
            < minimumDistanceBetweenManeuvers) {
          valid = false;
          offendingManeuvers.add(lastLateralManeuver);
          offendingManeuvers.add(maneuver);
        }
      }

      lastLateralManeuver = maneuver;
    }

    if (maneuver.getType() == ManeuverType.LONGITUDINAL) {
      if (lastLongitudinalManeuver != null) {
        if (Math.abs(lastLongitudinalManeuver.getEndLocation() - maneuver.getStartLocation())
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
      return new TrajectoryValidationResult();
    } else {
      return new TrajectoryValidationResult(new TrajectoryValidationError("Minimum distance between maneuvers not observed!", offendingManeuvers));
    }
	}

}
