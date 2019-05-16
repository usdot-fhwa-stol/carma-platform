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

package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

/**
 * Base abstract class for IAccStrategy implementations.
 * <p>
 * Handles the configuration intake and storage so that implementations need only worry
 * about the algorithmic logic.
 */
public abstract class AbstractAccStrategy implements IAccStrategy {
  protected static double TIMESTEP_DURATION_S = 0.1;
  protected double vehicleResponseDelay;
  protected double desiredTimeGap;
  protected double maxAccel;
  protected double lowerSpeedLimit;
  protected double upperSpeedLimit;

	@Override
	public void setVehicleResponseDelay(double responseTime) {
		vehicleResponseDelay = responseTime;
	}

	@Override
	public void setDesiredTimeGap(double timeGap) {
		desiredTimeGap = timeGap;
	}

	@Override
	public void setMaxAccel(double accel) {
		maxAccel = accel;
  }

  @Override
	public double getMaxAccel() {
		return maxAccel;
	}

	@Override
	public void setSpeedLimit(double lowerSpeedLimit, double upperSpeedLimit) {
    this.lowerSpeedLimit = lowerSpeedLimit;
    this.upperSpeedLimit = upperSpeedLimit;
	}

  @Override
	public abstract boolean evaluateAccTriggerConditions(double distToFrontVehicle, double currentSpeed,
			double frontVehicleSpeed);

  @Override
	public abstract double computeAccOverrideSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed,
			double desiredSpeedCommand);
  
  protected double applyAccelLimit(double overrideCommand, double currentSpeed, double maxAccel) {
    if (Math.abs(overrideCommand - currentSpeed) / TIMESTEP_DURATION_S > maxAccel) {
      // We're exceeding our accel limit
      if (overrideCommand > currentSpeed) {
        return currentSpeed + maxAccel * TIMESTEP_DURATION_S;
      } else if (overrideCommand < currentSpeed) {
        return currentSpeed - maxAccel * TIMESTEP_DURATION_S;
      } else {
        // Should never hit this branch, but just in case
        return currentSpeed;
      }
    } else {
      return overrideCommand;
    }
  }
}
