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

package gov.dot.fhwa.saxton.carma.guidance.util.trajectoryconverter;

/**
 * A helper class for storing the current state of a simulation when projecting a trajectory through time
 * This class is really and immutable struct and therefore lacks getters or setters for simplicity 
 * 
 * The simulation data contains
 * The current simulation time in seconds
 * The current downtrack distance in meters
 * The current segment downtrack distance in meters
 * The current segment index on the route
 */
public final class LongitudinalSimulationData {
  final double simTime;
  final double downtrack;
  final double segmentDowntrack;
  final int segmentIdx;

  LongitudinalSimulationData(double simTime, double downtrack, double segmentDowntrack, int segmentIdx) {
    this.simTime = simTime;
    this.downtrack = downtrack;
    this.segmentDowntrack = segmentDowntrack;
    this.segmentIdx = segmentIdx;
  }

  public boolean almostEquals(LongitudinalSimulationData data, double epsilon) {
    return 
      Math.abs(this.simTime - data.simTime) < epsilon &&
      Math.abs(this.downtrack - data.downtrack) < epsilon &&
      Math.abs(this.segmentDowntrack - data.segmentDowntrack) < epsilon &&
      this.segmentIdx == data.segmentIdx;
  }
}