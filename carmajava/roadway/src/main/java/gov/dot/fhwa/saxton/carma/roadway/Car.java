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
 * Class representing a Car on a roadway can be connected or have no communication.
 */
public class Car extends Vehicle {

  /**
   * Constructor
   */
  public Car(int id, double downtrackDistance, double crosstrackDistance, Vector3D velocity,
    Vector3D acceleration, Vector3D size, Integer primaryLane) {
    super(id, downtrackDistance, crosstrackDistance, velocity, acceleration, size, primaryLane);
  }
}
