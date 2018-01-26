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
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;

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
  protected ILogger log;

  public BasicAccStrategy(double minStandoffDistance, double exitDistanceFactor, Pipeline<Double> speedCmdPipeline) {
    super();
    this.standoffDistance = minStandoffDistance;
    this.exitDistanceFactor = exitDistanceFactor;
    this.speedCmdPipeline = speedCmdPipeline;
    log = LoggerManager.getLogger();
  }

  private double computeActualTimeGap(double distToFrontVehicle, double currentSpeed, double frontVehicleSpeed) {
    if (currentSpeed <= 0) {
      return Double.POSITIVE_INFINITY;
    }
    return distGap(distToFrontVehicle) / currentSpeed;
  }

  @Override
  public void setDesiredTimeGap(double timeGap) {
    super.setDesiredTimeGap(timeGap);
    log.info("ACC timegap set to " + timeGap);
    speedCmdPipeline.reset();
    speedCmdPipeline.changeSetpoint(timeGap);
  }

  @Override
  public boolean evaluateAccTriggerConditions(double distToFrontVehicle, double currentSpeed,
      double frontVehicleSpeed) {
    return computeActualTimeGap(distGap(distToFrontVehicle), currentSpeed, frontVehicleSpeed) < desiredTimeGap;
  }

  @Override
  public double computeAccOverrideSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed,
      double desiredSpeedCommand) {
    // Check PID control state
    if (!pidActive && evaluateAccTriggerConditions(distGap(distToFrontVehicle), currentSpeed, frontVehicleSpeed)) {
      log.info(
          String.format("ACC PID Control now active! {distToFrontVehicle=%.02f, distGap=%.02f, currentSpeed=%.02f, fvehSpeed=%.02f",
              distToFrontVehicle, distGap(distToFrontVehicle), currentSpeed, frontVehicleSpeed));
      pidActive = true;
      // If PID becomes inactive we should reset the controller before it is reactivated
      speedCmdPipeline.reset();
    }
    if (pidActive && computeActualTimeGap(distGap(distToFrontVehicle), currentSpeed, frontVehicleSpeed) > exitDistanceFactor
        * desiredTimeGap) {
      log.info(
          String.format("ACC PID Control now inactive! {distToFrontVehicle=%.02f, distGap=%.02f, currentSpeed=%.02f, fvehSpeed=%.02f",
              distToFrontVehicle, distGap(distToFrontVehicle),currentSpeed, frontVehicleSpeed));
      pidActive = false;
    }

    double speedCmd = desiredSpeedCommand;
    if (pidActive) {
      Optional<Signal<Double>> speedCmdSignal = speedCmdPipeline
          .apply(new Signal<>(computeActualTimeGap(distGap(distToFrontVehicle), currentSpeed, frontVehicleSpeed)));
      double rawSpeedCmd = speedCmdSignal.get().getData() + currentSpeed;
      speedCmd = applyAccelLimit(rawSpeedCmd, currentSpeed, maxAccel);
      log.info(String.format(
          "ACC OVERRIDE CMD = %.02f, current speed = %.02f, override after accel limit applied (%.02f m/s/s) = %.02f",
          rawSpeedCmd, currentSpeed, maxAccel, speedCmd));
      speedCmd = Math.min(speedCmd, desiredSpeedCommand);
    }

    return speedCmd;
  }

  /**
   * Helper function adjusts the perceived distance to the front vehicle
   * Adds the standoff distance to the front vehicle distance
   */
  private final double distGap(double distToFrontVehicle) {
    return standoffDistance + distToFrontVehicle;
  }
}