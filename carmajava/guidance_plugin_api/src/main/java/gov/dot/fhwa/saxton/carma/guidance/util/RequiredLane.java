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

package gov.dot.fhwa.saxton.carma.guidance.util;

/**
 * Data holder class for describing the required lane for the vehicle at a location on the route
 */
public class RequiredLane {
  private double location;
  private int laneId;

  /**
   * Create a required lane object that indicates the vehicle must be in the lane with laneId by location.
   * Uses the downtrack waypoint on the route to determine the location.
   */
  public RequiredLane(double location, int laneId) {
    this.location = location;
    this.laneId = laneId;
  }

  /**
   * Get the downtrack distance of the end of the segment this limit pertains to, in meters.
   */
  public double getLocation() {
    return location;
  }

  /**
   * Get the id of the lane the vehicle should be in at the end of this segment
   */
  public int getLaneId() {
    return laneId;
  }
  
  public void setLocation(double location) {
      this.location = location;
  }
  
  @Override
  public boolean equals(Object o) {
    if (!(o instanceof RequiredLane)) {
      return false;
    } else {
      RequiredLane other = (RequiredLane) o;
      return laneId == other.getLaneId() && location == other.getLocation();
    }
  }

  @Override
  public int hashCode() {
    return 13 * Double.hashCode(location)  + 17 * Double.hashCode(laneId);
  }

  @Override
  public String toString() {
    return String.format("RequiredLane{location=%.02f, laneId=%d}", location, laneId);
  }
}

