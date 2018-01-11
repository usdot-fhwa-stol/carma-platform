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

import gov.dot.fhwa.saxton.carma.guidance.signals.Pipeline;
import gov.dot.fhwa.saxton.carma.guidance.signals.Signal;
import java.util.Optional;

/**
 * Simple IAccStrategyImplementation based on a sliding blend of vehicle speeds.
 * <p>
 * Linearly blends our vehicle's speed command with the speed of the front vehicle based on
 * distance between the two.
 */
public class BasicAccStrategy extends AbstractAccStrategy {
  protected double standoffDistance = 5.0;
  private Pipeline<Double> speedCmdPipeline; // Pipeline is assumed to include a pid controller
  private double exitDistanceFactor = 2.0;
  private boolean pidActive = false;

  public BasicAccStrategy(double minStandoffDistance, double exitDistanceFactor, Pipeline<Double> speedCmdPipeline) {
    super();
    this.standoffDistance = minStandoffDistance;
    this.exitDistanceFactor = exitDistanceFactor;
    this.speedCmdPipeline = speedCmdPipeline;
  }

  private double computeActualTimeGap(double distToFrontVehicle, double currentSpeed, double frontVehicleSpeed) {
    if (currentSpeed <= 0) {
      return Double.POSITIVE_INFINITY;
    }
    return distToFrontVehicle / currentSpeed;
  }

  @Override
  public void setDesiredTimeGap(double timeGap) {
    super.setDesiredTimeGap(timeGap);
    speedCmdPipeline.reset();
    speedCmdPipeline.changeSetpoint(timeGap);
  }

  @Override
  public boolean evaluateAccTriggerConditions(double distToFrontVehicle, double currentSpeed,
      double frontVehicleSpeed) {
    return computeActualTimeGap(distToFrontVehicle, currentSpeed, frontVehicleSpeed) < desiredTimeGap;
  }

  //@Deprecated
  protected double computeAccIdealSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed,
    double desiredSpeedCommand) {
    // Linearly interpolate the speed blend between our speed and front vehicle speed
    double desiredHeadway = computeDesiredHeadway(currentSpeed);
    // Clamp distance - adjusted to ensure at least minimum standoff distance - into the range of [0, desiredHeadway]
    double distance = Math.max(distToFrontVehicle - standoffDistance, 0);
    double blendFactor = distance / desiredHeadway; // This factor will be used to compute our blend, as we get closer we give their speed more influence
    double ourSpeedFactor = (blendFactor) * currentSpeed;
    double frontVehicleSpeedFactor = (1 - blendFactor) * frontVehicleSpeed;

    return ourSpeedFactor + frontVehicleSpeedFactor;
  }

  @Override
  public double computeAccOverrideSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed,
      double desiredSpeedCommand) {
    // Check PID control state
    if (!pidActive && evaluateAccTriggerConditions(distToFrontVehicle, currentSpeed, frontVehicleSpeed)) {
      pidActive = true;
      // If PID becomes inactive we should reset the controller before it is reactivated
      speedCmdPipeline.reset();
    }
    if (pidActive && computeActualTimeGap(distToFrontVehicle, currentSpeed, frontVehicleSpeed) > exitDistanceFactor
        * desiredTimeGap) {
      pidActive = false;
    }

    double speedCmd = desiredSpeedCommand;
    if (pidActive) {
      Optional<Signal<Double>> speedCmdSignal = speedCmdPipeline
          .apply(new Signal<>(computeActualTimeGap(distToFrontVehicle, currentSpeed, frontVehicleSpeed)));
      speedCmd = speedCmdSignal.get().getData() + desiredSpeedCommand;
      speedCmd = applyAccelLimit(speedCmd, currentSpeed, maxAccel);
      speedCmd = Math.min(speedCmd, desiredSpeedCommand);
    }

    return speedCmd;
  }

  //@Deprecated
  @Override 
  public double computeDesiredHeadway(double currentSpeed) {
    return currentSpeed * desiredTimeGap;
  }
}