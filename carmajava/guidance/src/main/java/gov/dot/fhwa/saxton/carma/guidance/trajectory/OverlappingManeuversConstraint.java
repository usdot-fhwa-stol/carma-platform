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

import java.util.ArrayList;
import java.util.List;

/**
 * Trajectory constraint ensuring that Maneuvers are only allowed to overlap their
 * endpoint with the startpoint of another maneuver, but at no other locations.
 */
public class OverlappingManeuversConstraint implements TrajectoryValidationConstraint {

	protected List<IManeuver> visited = new ArrayList<>();

	@Override
	public void visit(IManeuver maneuver) {
		visited.add(maneuver);
	}

  /**
   * Compute if the ranges [start1, end1) and [start2, end2) overlap by analyizing the boundary points
   */
	private boolean checkOverlap(double start1, double end1, double start2, double end2) {
		// Start2 is inside the first range
		if (start2 >= start1 && start2 < end1) {
			return true;
		}
		//End2 is inside the range
		if (end2 > start1 && end2 < end1) { 
			return true;
		}
		// The whole of range 2 overlaps range1
		if (start2 <= start1 && end2 >= end1) {
			return true;
		}

		// Start2 is inside the first range
		if (start1 >= start2 && start1 < end2) {
			return true;
		}
		//End2 is inside the range
		if (end1 > start2 && end1 < end2) { 
			return true;
		}
		// The whole of range 2 overlaps range1
		if (start1 <= start2 && end1 >= end2) {
			return true;
		}

		return false;
	}

	@Override
	public TrajectoryValidationResult getResult() {
		List<IManeuver> overlappingManeuvers = new ArrayList<>();
		for (int i = 0; i < visited.size(); i++) {
			IManeuver m = visited.get(i);
			for (int j = i + 1; j < visited.size(); j++) {
				IManeuver m2 = visited.get(j);

				if (checkOverlap(m.getStartLocation(), m.getEndLocation(), m2.getStartLocation(), m2.getEndLocation())
				&& m.getType() == m2.getType()) {
					overlappingManeuvers.add(m);
					overlappingManeuvers.add(m2);
					return new TrajectoryValidationResult(new TrajectoryValidationError("Overlapping maneuvers detected!", overlappingManeuvers));
				}
			}
		}

    visited = new ArrayList<>();
		return new TrajectoryValidationResult();
	}

}
