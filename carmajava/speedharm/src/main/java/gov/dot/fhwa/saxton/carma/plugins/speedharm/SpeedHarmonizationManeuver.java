/*
 * Copyright (C) 2018 LEIDOS.
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
package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ComplexManeuverBase;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

import org.ros.message.Duration;
import org.ros.message.Time;

/**
 * Maneuver which executes complex speed commands provided by an ISpeedHarmInputs object.
 */
public class SpeedHarmonizationManeuver extends ComplexManeuverBase {

  private ISpeedHarmInputs speedHarmInputs_;
  private Duration timeout = Duration.fromMillis(3000);
  private ArbitratorService arbitratorService;

  /**
   * Constructor where user provides all relevant inputs
   *
   * @param planner The name of the plugin responsible for this maneuver
   * @param speedHarmInputs Input which provides the current desired commands from a speed harm trajectory source
   * @param inputs Input which provides the current state of the vehicle
   * @param commands The target for calculated commands
   * @param startDist The distance along the route to the maneuver starting point
   * @param endDist The distance along the route which marks the maneuver end point
   * @param minCompletionTime The minimum anticipated execution time
   * @param maxCompletionTime The maximum anticipated execution time
   * @param minExpectedSpeed The minimum expected speed
   * @param maxExpectedSpeed The maximum expected speed
   */
  public SpeedHarmonizationManeuver(IPlugin planner, ISpeedHarmInputs speedHarmInputs, IManeuverInputs inputs, IGuidanceCommands commands,
    IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
    double minExpectedSpeed, double maxExpectedSpeed) {

    super(planner, inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime,
      minExpectedSpeed, maxExpectedSpeed);
    speedHarmInputs_ = speedHarmInputs;
  }

  /**
   * Constructor where the expected speeds are calculated
   */
  public SpeedHarmonizationManeuver(IPlugin planner, ISpeedHarmInputs speedHarmInputs, IManeuverInputs inputs, IGuidanceCommands commands,
    IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime) {

    super(planner, inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime);
    speedHarmInputs_ = speedHarmInputs;
  }

  /**
   * Constructor where the expected speeds are calculated
   */
  public SpeedHarmonizationManeuver(IPlugin planner, ISpeedHarmInputs speedHarmInputs, IManeuverInputs inputs, IGuidanceCommands commands,
    IAccStrategy accStrategy, double startDist, double endDist, double minExpectedSpeed, double maxExpectedSpeed) {

    super(planner, inputs, commands, accStrategy, startDist, endDist, minExpectedSpeed, maxExpectedSpeed);
    speedHarmInputs_ = speedHarmInputs;
  }

  @Override protected double generateSpeedCommand() throws IllegalStateException {
    checkTimeout();

    return speedHarmInputs_.getSpeedCommand();
  }

  @Override protected double generateMaxAccelCommand() throws IllegalStateException {
    checkTimeout();

    double maxAccel = speedHarmInputs_.getMaxAccelLimit();

    if (maxAccel < 0.0) {
      throw new IllegalStateException("SpeedHarmonizationManeuver received negative maxAccel command: " + maxAccel);
    } else if (maxAccel > maxAccel_) {
      maxAccel = maxAccel_;
      log_.warn("Truncated max acceleration above limit to limit value");
    }
    return maxAccel;
  }

  private void checkTimeout() throws IllegalStateException {
    Duration timeElapsed = speedHarmInputs_.getTimeSinceLastUpdate();
    if (timeElapsed.compareTo(timeout) > 0) {
      if (arbitratorService == null) {
        log_.error("SpeedHarmonizationManeuver timeout. Timeout: " + timeout + " ElapsedTime: " + timeElapsed);
        throw new IllegalStateException("SpeedHarmonizationManeuver timeout. Timeout: " + timeout + " ElapsedTime: " + timeElapsed);
      } else {
        log_.warn("SpeedHarmonizationManeuver timeout. Timeout: " + timeout + " ElapsedTime: " + timeElapsed + ". Notifying arbitrator of failure.");
        arbitratorService.notifyTrajectoryFailure();
      }
    }
  }

  /**
   * Gets the command update timeout after which the maneuver will not execute a timestep
   * @return the duration of the timeout
   */
  public Duration getTimeout() {
    return timeout;
  }

  /**
   * Sets the command update timeout after which the maneuver will not execute a timestep
   */
  public void setTimeout(Duration timeout) {
    this.timeout = timeout;
  }

  /**
   * Sets the arbitration service the maneuver will use to force a replan in the event of a timeout.
   */
  public void setArbitratorService(ArbitratorService arbitratorService) {
    this.arbitratorService = arbitratorService;
  }
}
