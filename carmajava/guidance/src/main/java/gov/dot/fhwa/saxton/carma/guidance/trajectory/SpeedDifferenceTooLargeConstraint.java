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

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuver;

/**
 * Constraint which ensures that consecutive Maneuvers cannot exceed a speed difference
 * such as to not violate acceleration limits (or otherwise be impossible to execute)
 *
 * Currently only a stub, pending real implementation of Maneuvers package to get
 * speed data.
 *
 * TODO: Implement after Maneuvers package.
 */
public class SpeedDifferenceTooLargeConstraint implements TrajectoryValidationConstraint {

	@Override
	public void visit(IManeuver maneuver) {
		// NO-OP
	}

	@Override
	public TrajectoryValidationResult getResult() {
		return new TrajectoryValidationResult();
	}

}
