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

package gov.dot.fhwa.saxton.carma.geometry.cartesian.spacialhashmap;

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
public final class ConflictSpace {
  double startDowntrack;
  double endDowntrack;
  double startTime;
  double endTime;
  int lane;

  ConflictSpace(double startDowntrack, double startTime, int lane) {
    this.startDowntrack = startDowntrack;
    this.startTime = startTime; 
    this.endDowntrack = startDowntrack; // TODO make note of this and next line in comments
    this.endTime = startTime;
    this.lane = lane;
  }

  public void setEndDowntrack(double endDowntrack) {
    this.endDowntrack = endDowntrack;
  }

  public void setEndTime(double endTime) {
    this.endTime = endTime;
  }

  public int getLane() {
    return this.lane;
  }

  public boolean almostEquals(ConflictSpace space, double epsilon) {
    return 
      Math.abs(this.startDowntrack - space.startDowntrack) < epsilon &&
      Math.abs(this.endDowntrack - space.endDowntrack) < epsilon &&
      Math.abs(this.startTime - space.startTime) < epsilon &&
      Math.abs(this.endTime - space.endTime) < epsilon &&
      this.lane == space.lane;
  }
}