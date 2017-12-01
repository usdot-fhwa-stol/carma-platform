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
 * Adaptive Cruise Control Strategy-pattern Interface
 * <p>
 * All implementations of ACC must extend this interface and have a corresponding
 * {@link IAccStrategyFactory} to instantiate them. Maneuvers will use these instances
 * in an online fashion to execute ACC functionality and Plugins will use these instances
 * to estimate the behavior of the ACC system such that they may plan around it.
 */
public interface IAccStrategy {
  public static double NO_FRONT_VEHICLE_DISTANCE = Double.POSITIVE_INFINITY;
  public static double NO_FRONT_VEHICLE_SPEED = Double.POSITIVE_INFINITY;

  /**
   * Set the amount of time (in ms) to account for as delay in vehicle response to speed commands
   */
  void setVehicleResponseDelay(double responseTime);

  /**
   * Set the desired amount of travel time between the current vehicle and front vehicle (in ms)
   */
  void setDesiredTimeGap(double timeGap);

  /**
   * Set the maximum acceleration configured for the vehicle
   */
  void setMaxAccel(double accel);

  /**
   * Set the upper and lower speed limits for the vehicle
   */
  void setSpeedLimit(double lowerSpeedLimit, double upperSpeedLimit);

  /**
   * Evaluate whether or not ACC would activate given the input conditions
   * 
   * @param distToFrontVehicle Distance, in meters, to the vehicle directly in front of the current vehicle.
   * Use NO_FRONT_VEHICLE_DISTANCE if no such vehicle is detected.
   * @param currentSpeed The current vehicles speed, in meters per second,
   * @param frontVehicleSpeed The front vehicle's speed, in meters per second. Use NO_FRONT_VEHICLE_SPEED if there
   * is no such vehicle detected
   * 
   * @return True or false if ACC would be engaged or not under the input conditions
   */
  boolean evaluateAccTriggerConditions(double distToFrontVehicle, double currentSpeed, double frontVehicleSpeed);

  /**
   * Compute the speed ACC would command to regulate the desired headway given the input conditions
   * 
   * @param distToFrontVehicle Distance, in meters, to the vehicle directly in front of the current vehicle.
   * Use NO_FRONT_VEHICLE_DISTANCE if no such vehicle is detected.
   * @param currentSpeed The current vehicles speed, in meters per second,
   * @param frontVehicleSpeed The front vehicle's speed, in meters per second. Use NO_FRONT_VEHICLE_SPEED if there
   * is no such vehicle detected
   * @param desiredSpeedCommand The desired speed of the current vehicle
   * 
   * @return The speed deemed necessary to acheieve and maintain the desired headway. If no change is required,
   * desiredSpeedCommand will be returned
   */
  double computeAccOverrideSpeed(double distToFrontVehicle, double frontVehicleSpeed, double currentSpeed, double desiredSpeedCommand);

  /**
   * Compute the desired headway (in meters) at currentSpeed
   */
  double computeDesiredHeadway(double currentSpeed);

  /**
   * Get the maximum acceleration value this ACC strategy is authorized to command, in m/s/s
   */
  double getMaxAccel();
}