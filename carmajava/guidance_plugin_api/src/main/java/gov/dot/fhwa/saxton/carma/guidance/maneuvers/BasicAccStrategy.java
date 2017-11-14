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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Simple IAccStrategyImplementation based on a sliding blend of vehicle speeds.
 * <p>
 * Linearly blends our vehicle's speed command with the speed of the front vehicle based on
 * distance between the two.
 */
public class BasicAccStrategy extends AbstractAccStrategy {
  protected double standoffDistance = 5.0;

	@Override
	public boolean evaluateAccTriggerConditions(double distToFrontVehicle, double currentSpeed,
			double frontVehicleSpeed) {
    return distToFrontVehicle < computeDesiredHeadway(currentSpeed);
	}

	@Override
	public double computeAccOverrideSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed,
			double desiredSpeedCommand) {
		if (evaluateAccTriggerConditions(distToFrontVehicle, currentSpeed, frontVehicleSpeed)) {
      // Linearly interpolate the speed blend between our speed and front vehicle speed
      double desiredHeadway = computeDesiredHeadway(currentSpeed);
      // Clamp distance - adjusted to ensure at least minimum standoff distance - into the range of [0, desiredHeadway]
      double distance = Math.min(Math.max(distToFrontVehicle - standoffDistance, 0), desiredHeadway);
      double blendFactor = distance / desiredHeadway; // This factor will be used to compute our blend, as we get closer we give their speed more influence
      double ourSpeedFactor = (1 - blendFactor) * currentSpeed;
      double frontVehicleSpeedFactor = blendFactor * frontVehicleSpeed;

      return ourSpeedFactor + frontVehicleSpeedFactor; // Our final blend of speeds
    } else {
      return desiredSpeedCommand;
    }
	}

	@Override
	public double computeDesiredHeadway(double currentSpeed) {
    return currentSpeed * desiredTimeGap;
	}
}