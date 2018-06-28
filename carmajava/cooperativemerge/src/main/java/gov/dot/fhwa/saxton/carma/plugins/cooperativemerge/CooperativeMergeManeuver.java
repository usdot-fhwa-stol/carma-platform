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
package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

import org.ros.message.Time;

import gov.dot.fhwa.saxton.carma.guidance.ArbitratorService;
import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ComplexManeuverBase;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

/**
 * Maneuver which executes complex speed and steering commands provided by an ICooperativeMergeInputs object.
 */
public class CooperativeMergeManeuver extends ComplexManeuverBase {

  private ICooperativeMergeInputs commandInputs;

  /**
   * Constructor where user provides all relevant inputs
   *
   * @param planner The name of the plugin responsible for this maneuver
   * @param commandInputs Input which provides the current desired commands from a speed harm trajectory source
   * @param inputs Input which provides the current state of the vehicle
   * @param commands The target for calculated commands
   * @param startDist The distance along the route to the maneuver starting point
   * @param endDist The distance along the route which marks the maneuver end point
   * @param minCompletionTime The minimum anticipated execution time
   * @param maxCompletionTime The maximum anticipated execution time
   * @param minExpectedSpeed The minimum expected speed
   * @param maxExpectedSpeed The maximum expected speed
   * @param maxAccel The maximum acceleration ever allowed in this maneuver in m/s^2
   */
  public CooperativeMergeManeuver(IPlugin planner, ICooperativeMergeInputs commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
    IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
    double minExpectedSpeed, double maxExpectedSpeed, double maxAccel) {

    super(planner, inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime,
      minExpectedSpeed, maxExpectedSpeed);

    this.maxAccel_ = maxAccel;
    this.commandInputs = commandInputs;
  }
  protected CooperativeMergeManeuver(IPlugin planner, ICooperativeMergeInputs commandInputs,
          IManeuverInputs currentState, IGuidanceCommands commandsOutputs, IAccStrategy accStrategy,
          double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime, double maxAccel) {
      super(planner, currentState, commandsOutputs, accStrategy,
              startDist, endDist, minCompletionTime, maxCompletionTime);
      
      this.maxAccel_ = maxAccel;
      this.commandInputs = commandInputs;
  }

  protected CooperativeMergeManeuver(IPlugin planner, ICooperativeMergeInputs commandInputs,
          IManeuverInputs currentState, IGuidanceCommands commandsOutputs, IAccStrategy accStrategy,
          double startDist, double endDist, double minExpectedSpeed, double maxExpectedSpeed, double maxAccel) {
      super(planner, currentState, commandsOutputs, accStrategy,
              startDist, endDist, minExpectedSpeed, maxExpectedSpeed);
      
      this.maxAccel_ = maxAccel;
      this.commandInputs = commandInputs;
  }

  @Override protected double generateSpeedCommand() throws IllegalStateException {
    return commandInputs.getSpeedCommand();
  }

  @Override protected double generateSteeringCommand() throws IllegalStateException {
    return commandInputs.getSteeringCommand();
  }

  @Override protected double generateMaxAccelCommand() throws IllegalStateException {

    double maxAccel = commandInputs.getMaxAccelLimit();

    if (maxAccel < 0.0) {
      throw new IllegalStateException(this.getClass().getSimpleName() + " received negative maxAccel command: " + maxAccel);
    } else if (maxAccel > maxAccel_) {
      maxAccel = maxAccel_;
      log_.warn("Truncated max acceleration above limit to limit value");
    }
    return maxAccel;
  }
}
