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

import org.ros.message.Duration;

/**
 * Specifies the data input interface for a SpeedHarmonizationManeuver. This keeps knowledge of the ROS network out of the remainder
 * of the package.
 */
public class CooperativeMergeInputs implements ICooperativeMergeInputs{

  private double speedCommand; // m/s
  private double steeringCommand; // rad
  private double maxAccel; // m/s^2
  private Duration timeSinceLastUpdate;

  /**
   * Constructor
   * 
   * @param maxAccel The maximum acceleration allowed in m/s^2
   */
  public CooperativeMergeInputs(double maxAccel) {
    this.maxAccel = maxAccel;
  }

  /**
   * @param speedCommand the speedCommand to set
   */
  public void setSpeedCommand(double speedCommand) {
    this.speedCommand = speedCommand;
  }

  @Override
  public double getSpeedCommand() {
    return speedCommand;
  }

  @Override
  public double getMaxAccelLimit() {
    return getMaxAccelLimit();
  }

  /**
   * @param timeSinceLastUpdate the timeSinceLastUpdate to set
   */
  public void setTimeSinceLastUpdate(Duration timeSinceLastUpdate) {
    this.timeSinceLastUpdate = timeSinceLastUpdate;
  }

  @Override
  public Duration getTimeSinceLastUpdate() {
    return timeSinceLastUpdate;
  }

  /**
   * @param steeringCommand the steeringCommand to set
   */
  public void setSteeringCommand(double steeringCommand) {
    this.steeringCommand = steeringCommand;
  }

}