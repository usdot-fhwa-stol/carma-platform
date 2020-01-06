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
 * An interface which provides functions for calculating distances between location objects and segments.
 * This interface is implemented by different algorithms for calculating distances between points using curved earth models.
 */
public interface IDistanceStrategy {
  /**
   * Calculates the earth surface distance between two locations
   *
   * @param loc1 The first location
   * @param loc2 The second location
   * @return The distance in meters between the two provided locations
   */
  double distanceLoc2Loc(Location loc1, Location loc2);

  /**
   * Calculates the cross-track great circle distance between a location and great circle segment.
   *
   * @param loc The location
   * @param seg The great circle segment
   * @return The distance in meters
   */
  double crossTrackDistance(Location loc, GreatCircleSegment seg);

  /**
   * Calculates the downtrack distance along a segment of a provided location
   *
   * @param loc The location whose downtrack distance is being calculated
   * @param seg The great circle segment which defines the track
   * @return The distance in meters
   */
  double downtrackDistance(Location loc, GreatCircleSegment seg);

  /**
   * Calculates location of a external point projected onto a greate circle segment
   *
   * @param loc The location whose projection is being calculated
   * @param seg The great circle segment which defines the track
   * @return The projected location
   */
  Location projectOntoSegment(Location loc, GreatCircleSegment seg);
}
