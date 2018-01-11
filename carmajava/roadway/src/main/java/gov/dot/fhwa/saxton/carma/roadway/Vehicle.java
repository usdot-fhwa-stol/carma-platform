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

package gov.dot.fhwa.saxton.carma.roadway;

import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;

/**
 * Class representing a Vehicle on a roadway can be connected or have no communication.
 * In future development Vehicle can be extended to support more specializations such as car or truck.
 */
public class Vehicle extends Obstacle {
  protected ConnectedVehicleType connectedVehicleType;
  protected byte[] bsmId = {}; // The temporary bsm id. This variable is only set if the vehicle is connected.
  // May want to add negotiation id or active negotiation flag

  /**
   * Constructor
   */
  public Vehicle(int id, double downtrackDistance, double crosstrackDistance, Vector3D velocity,
    Vector3D acceleration, Vector3D size, Integer primaryLane) {
    super(id, downtrackDistance, crosstrackDistance, velocity, acceleration, size, primaryLane);
  }

  /**
   * Sets the connected vehicle type
   * @param type the connected vehicle type
   */
  public void setConnectedVehicleType(ConnectedVehicleType type) {
    this.connectedVehicleType = type;
  }

  /**
   * Gets the connected vehicle type
   * @return the connected vehicle type
   */
  public ConnectedVehicleType getConnectedVehicleType() {
   return connectedVehicleType;
  }

  /**
   * Returns true if the bsm id has been set
   * @return true if a valid bsm id has been set
   */
  public boolean hasBSMId() {
    return bsmId.length == 4;
  }

  /**
   * Sets the temporary bsm id of this vehicle
   * @param the bsm id
   */
  public void setBSMId(byte[] bsmId) {
    this.bsmId = bsmId;
  }

  /**
   * Gets the temporary bsm id which corresponds to this vehicle
   * Note: bsmId will be empty array if no id is available
   * @return the bsm id
   */
  public byte[] getBSMId() {
    return bsmId;
  }
}
