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

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.util.ILogger;
import gov.dot.fhwa.saxton.carma.guidance.util.LoggerManager;
import org.ros.message.Time;

/**
 * Base class for all complex maneuver objects.
 */
public abstract class ComplexManeuverBase implements IComplexManeuver {

  protected IManeuverInputs inputs_;
  protected IGuidanceCommands commands_;
  protected ILogger log_ = LoggerManager.getLogger();
  protected IAccStrategy accStrategy_;
  protected double startDist_;
  protected double endDist_;
  protected Time minCompletionTime_;
  protected Time maxCompletionTime_;
  protected double minExpectedSpeed_;
  protected double maxExpectedSpeed_;
  protected double maxAccel_ = 0.999;     // m/s^2 absolute value; default is a conservative value

  /**
   * Constructor where user provides all relevant inputs
   *
   * @param inputs            Input which provides the current state of the vehicle
   * @param commands          The target for calculated commands
   * @param startDist         The distance along the route to the maneuver starting point
   * @param endDist           The distance along the route which marks the maneuver end point
   * @param minCompletionTime The minimum anticipated execution time
   * @param maxCompletionTime The maximum anticipated execution time
   * @param minExpectedSpeed  The minimum expected speed
   * @param maxExpectedSpeed  The maximum expected speed
   */
  protected ComplexManeuverBase(IManeuverInputs inputs, IGuidanceCommands commands, IAccStrategy accStrategy,
    double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
    double minExpectedSpeed, double maxExpectedSpeed) {
    startDist_ = startDist;
    endDist_ = endDist;
    minCompletionTime_ = minCompletionTime;
    maxCompletionTime_ = maxCompletionTime;
    minExpectedSpeed_ = minExpectedSpeed;
    maxExpectedSpeed_ = maxExpectedSpeed;
    inputs_ = inputs;
    commands_ = commands;
    accStrategy_ = accStrategy;
    validateBoundsFeasibility();
  }

  /**
   * Constructor where the expected speeds are calculated
   */
  public ComplexManeuverBase(IManeuverInputs inputs, IGuidanceCommands commands, IAccStrategy accStrategy,
    double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime) {
    inputs_ = inputs;
    commands_ = commands;
    accStrategy_ = accStrategy;
    startDist_ = startDist;
    endDist_ = endDist;
    minCompletionTime_ = minCompletionTime;
    maxCompletionTime_ = maxCompletionTime;
    minExpectedSpeed_ = (endDist_ - startDist_) / maxCompletionTime.toSeconds();
    maxExpectedSpeed_ = (endDist_ - startDist_) / minCompletionTime.toSeconds();
    validateBoundsFeasibility();
  }

  /**
   * Constructor where the completion times are calculated
   */
  public ComplexManeuverBase(IManeuverInputs inputs, IGuidanceCommands commands, IAccStrategy accStrategy,
    double startDist, double endDist, double minExpectedSpeed, double maxExpectedSpeed) {
    inputs_ = inputs;
    commands_ = commands;
    accStrategy_ = accStrategy;
    startDist_ = startDist;
    endDist_ = endDist;
    minExpectedSpeed_ = minExpectedSpeed;
    maxExpectedSpeed_ = maxExpectedSpeed;
    minCompletionTime_ =
      Time.fromMillis((long) (1000 * ((endDist_ - startDist_) / maxExpectedSpeed_)));
    maxCompletionTime_ =
      Time.fromMillis((long) (1000 * ((endDist_ - startDist_) / minExpectedSpeed_)));
    validateBoundsFeasibility();
  }

  @Override public String getManeuverName() {
    return this.getClass().getSimpleName();
  }

  @Override public Time getMinCompletionTime() {
    return minCompletionTime_;
  }

  @Override public Time getMaxCompletionTime() {
    return maxCompletionTime_;
  }

  @Override public final boolean executeTimeStep() throws IllegalStateException {
    if (inputs_.getDistanceFromRouteStart() > endDist_)
      return true;

    double speedCmd = generateSpeedCommand();
    double maxAccelCmd = generateMaxAccelCommand();
    double distToFrontVehicle = inputs_.getDistanceToFrontVehicle();
    double currentSpeed = inputs_.getCurrentSpeed();
    double frontVehicleSpeed = inputs_.getFrontVehicleSpeed();

    if (accStrategy_.evaluateAccTriggerConditions(distToFrontVehicle, currentSpeed, frontVehicleSpeed)) {
      speedCmd = accStrategy_.computeAccOverrideSpeed(distToFrontVehicle, frontVehicleSpeed, currentSpeed, speedCmd);
    }

    commands_.setCommand(speedCmd, maxAccelCmd);
    return false;
  }

  /**
   * Generates the next speed command. Is meant to be followed by a call to generateMaxAccelCommand
   *
   * @return The speed command
   */
  protected abstract double generateSpeedCommand();

  /**
   * Generates the next Max Accel Command. Is meant to be called following generateSpeedCommand
   *
   * @return The max accel command
   */
  protected abstract double generateMaxAccelCommand();

  @Override public double getMaxExpectedSpeed() {
    return maxExpectedSpeed_;
  }

  @Override public double getStartDistance() {
    return startDist_;
  }

  @Override public double getMinExpectedSpeed() {
    return minExpectedSpeed_;
  }

  @Override public double getEndDistance() {
    return endDist_;
  }

  /**
   * Verifies that the input bounds provided to this maneuver are valid and physically feasible
   * This function is meant to be called in the constructor
   *
   * @throws IllegalArgumentException if the inputs are invalid
   */
  protected void validateBoundsFeasibility() throws IllegalArgumentException {
    // Check distances bounds are valid
    if (startDist_ > endDist_ || startDist_ < -0.0 || endDist_ < -0.0 || startDist_ == endDist_) {
      throw new IllegalArgumentException(
        "Attempted to construct ComplexManeuver with invalid start and end points." + toString());
    }

    double minElapsedSeconds = minCompletionTime_.toSeconds();
    double maxElapsedSeconds = maxCompletionTime_.toSeconds();

    // Check completion times are valid
    if (minElapsedSeconds > maxElapsedSeconds || minElapsedSeconds < -0.0
      || maxElapsedSeconds < -0.0) {
      throw new IllegalArgumentException(
        "Attempted to construct ComplexManeuver with invalid completion times: " + toString());
    }

    // Check expected speeds are valid
    if (minExpectedSpeed_ > maxExpectedSpeed_ || minExpectedSpeed_ < -0.0
      || maxExpectedSpeed_ < -0.0) {
      throw new IllegalArgumentException(
        "Attempted to construct ComplexManeuver with invalid completion times: " + toString());
    }

    double deltaDist = endDist_ - startDist_;
    double minDeltaDist = minExpectedSpeed_ * minCompletionTime_.toSeconds();

    // Check compatibility between speeds, times, and distances
    if (minDeltaDist > deltaDist) { //TODO would it be worth having some wiggle room here?
      throw new IllegalArgumentException(
        "Attempted to construct ComplexManeuver with impossible constraints: " + toString());
    }

    double maxDeltaDist = maxExpectedSpeed_ * maxCompletionTime_.toSeconds();

    if (maxDeltaDist < deltaDist) { //TODO would it be worth having some wiggle room here?
      throw new IllegalArgumentException(
        "Attempted to construct ComplexManeuver with impossible constraints: " + toString());
    }
  }

  /**
   * Specifies the maximum acceleration allowed in the maneuver.  Note that this value will apply to both speeding
   * up and slowing down (symmetrical).
   * @param limit - max (absolute value) allowed, m/s^2
   */
  public void setMaxAccel(double limit) {
    if (limit > 0.0) { //can't be equal to zero
      maxAccel_ = limit;
    }
  }

  @Override public String toString() {
    return getManeuverName() + "{ , startDist_=" + startDist_ + ", endDist_=" + endDist_
      + ", minCompletionTime_=" + minCompletionTime_ + ", maxCompletionTime_=" + maxCompletionTime_
      + ", minExpectedSpeed_=" + minExpectedSpeed_ + ", maxExpectedSpeed_=" + maxExpectedSpeed_
      + '}';
  }
}
