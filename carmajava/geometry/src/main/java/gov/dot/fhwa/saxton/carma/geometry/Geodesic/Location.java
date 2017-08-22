/*
 * TODO: Copyright (C) 2017 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.geometry.Geodesic;

/**
 * A point in the WGS-84 coordinate system. A location has a latitude, longitude, and altitude.
 */
public class Location {
  double latitude;
  double longitude;
  double altitude;

  /**
   * Constructor initializes this location with the provided gps coordinates.
   * @param lat Latitude in degrees
   * @param lon Longitude in degrees
   * @param alt Altitude in meters
   */
  public Location(double lat, double lon, double alt) {
    this.latitude = lat;
    this.longitude = lon;
    this.altitude = alt;
  }

  /**
   * Calculates the distance in meters between this and another location.
   * @param loc The gps location to calculate this location's distance from
   * @param strategy The geodesic distance calculation strategy.
   * @return The distance in meters
   * TODO: Implement
   */
  public double distanceFrom(Location loc, IDistanceStrategy strategy) {
    return 0;
  }
}
