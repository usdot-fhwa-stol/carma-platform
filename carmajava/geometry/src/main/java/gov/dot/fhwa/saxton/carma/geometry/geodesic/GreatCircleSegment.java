/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry.geodesic;

/**
 * Represents a line between two locations.
 * It is not necessarily straight in the traditional sense as the distance is calculated along the surface of the earth according to a curved earth model.
 */
public class GreatCircleSegment {
  protected Location loc1;
  protected Location loc2;
  protected double length;
  protected IDistanceStrategy distanceStrategy;

  /**
   * Constructor initializes this earth segment with the provided locations
   * @param loc1 First location
   * @param loc2 Second location
   */
  public GreatCircleSegment(Location loc1, Location loc2) {
    this.loc1 = loc1;
    this.loc2 = loc2;
    this.setDistanceStrategy(new HaversineStrategy());
  }

  /**
   * Calculates the cross-track great circle distance between a location and this segment.
   *
   * @param loc The location
   * @return The distance in meters
   */
  public double crossTrackDistance(Location loc) {
    return distanceStrategy.crossTrackDistance(loc, this);
  }

  /**
   * Calculates the downtrack distance along this segment for a provided location
   *
   * @param loc The location whose downtrack distance is being calculated
   * @return The distance in meters
   */
  public double downtrackDistance(Location loc) {
    return distanceStrategy.downtrackDistance(loc, this);
  }

  /**
   * Calculates location of a external point projected onto the segment
   *
   * @param loc The location whose projection is being calculated
   * @return The projected location
   */
  public Location projectOntoSegment(Location location) {
    return distanceStrategy.projectOntoSegment(location, this);
  }

  /**
   * Sets the distance strategy which will be used for calculations in this segment and update the length accordingly
   * @param distanceStrategy the new default distance strategy
   */
  public void setDistanceStrategy(IDistanceStrategy distanceStrategy) {
    this.distanceStrategy = distanceStrategy;
    this.length = loc1.distanceFrom(loc2, distanceStrategy);
  }

  /**
   * Get the starting location of this segment
   * @return the location
   */
  public Location getLoc1() {
    return loc1;
  }

  /**
   * Get the ending location of this segment
   * @return the location
   */
  public Location getLoc2() {
    return loc2;
  }

  /**
   * Get the length of this segment as calculated using this segments distance strategy
   * @return the length
   */
  public double getLength() {
    return length;
  }
}
