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
package gov.dot.fhwa.saxton.carma.plugins.cooperativemerge;

import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

/**
 * Specifies the data input interface for a CooperativeMergeManeuver.
 * of the package.
 */
public class CooperativeMergeInputs implements ICooperativeMergeInputs{

  private volatile double speedCommand; // m/s
  private volatile double steeringCommand; // rad
  private volatile double maxAccel; // m/s^2
  private volatile boolean commandsSet = false;
  private final IManeuverInputs maneuverInputs;

  /**
   * Constructor
   */
  public CooperativeMergeInputs(IManeuverInputs maneuverInputs) {
    this.maneuverInputs = maneuverInputs;
  }

  /**
   * Synchronized set of current commands
   * 
   * @param speedCommand the speedCommand to set in m/s
   * @param maxAccel the maxAccel to set in m/s^2
   * @param steeringCommand the steeringCommand to set in rad
   */
  public synchronized void setCommands(double speed, double maxAccel, double steer) {
    this.commandsSet = true;
    this.speedCommand = speed;
    this.maxAccel = maxAccel;
    this.steeringCommand = steer;
  }

  @Override
  public synchronized double getSpeedCommand() {
    if (!commandsSet) {
      return maneuverInputs.getCurrentSpeed();
    }
    return speedCommand;
  }

  @Override
  public synchronized double getMaxAccelLimit() {
    if (!commandsSet) {
      return maneuverInputs.getMaxAccelLimit();
    }
    return maxAccel;
  }

  @Override
  public synchronized double getSteeringCommand() {
    if (!commandsSet) {
      return 0.0;
    }
    return steeringCommand;
  }

}