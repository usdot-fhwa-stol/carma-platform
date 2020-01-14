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
 * A point in the WGS-84 coordinate system. A location has a latitude, longitude, and altitude.
 */
public class Location {
  protected double latitude;
  protected double longitude;
  protected double altitude;
  // As radians (updated in setter functions);
  protected double latRad;
  protected double lonRad;

  /**
   * Default Constructor
   */
  public Location(){}

  /**
   * Constructor makes a new location as a deep copy of the provided location.
   * @param locToCopy the location duplicated
   */
  public Location(Location locToCopy) {
    this.setLocationData(locToCopy.getLatitude(), locToCopy.getLongitude(), locToCopy.getAltitude());
  }
  
  /**
   * Constructor initializes this location with the provided coordinates.
   * @param lat Latitude in degrees
   * @param lon Longitude in degrees
   * @param alt Altitude in meters
   */
  public Location(double lat, double lon, double alt) {
    setLocationData(lat,lon,alt);
  }

  /**
   * Calculates the distance in meters between this and another location.
   * @param loc The location to calculate this location's distance from
   * @param strategy The geodesic distance calculation strategy.
   * @return The distance in meters
   */
  public double distanceFrom(Location loc, IDistanceStrategy strategy) {
    return strategy.distanceLoc2Loc(this, loc);
  }

  /**
   * Gets the latitude
   * @return the latitude
   */
  public double getLatitude() {
    return latitude;
  }

  /**
   * Sets the latitude
   * @param latitude the latitude to be set
   */
  public void setLatitude(double latitude) {
    this.latitude = latitude;
    this.latRad = Math.toRadians(this.latitude);
  }

  /**
   * Gets the longitude
   * @return the longitude
   */
  public double getLongitude() {
    return longitude;
  }

  /**
   * Sets the longitude
   * @param longitude the longitude to be set
   */
  public void setLongitude(double longitude) {
    this.longitude = longitude;
    this.lonRad = Math.toRadians(this.longitude);
  }

  /**
   * Gets the altitude
   * @return the altitude to be set
   */
  public double getAltitude() {
    return altitude;
  }

  /**
   * Sets the altitude
   * @param altitude the altitude to be set
   */
  public void setAltitude(double altitude) {
    this.altitude = altitude;
  }

  /**
   * Sets the lat/lon/alt of this location in one function call
   * @param latitude the latitude to be set
   * @param longitude the longitude to be set
   * @param altitude the altitude to be set
   */
  public void setLocationData(double latitude, double longitude, double altitude) {
    this.setLatitude(latitude);
    this.setLongitude(longitude);
    this.setAltitude(altitude);
  }

  /**
   * Get the latitude in radians
   * @return the latitude in radians
   */
  public double getLatRad() {
    return latRad;
  }

  /**
   * Get the longitude in radians
   * @return the longitude in radians
   */
  public double getLonRad() {
    return lonRad;
  }

  /**
   * Check equality between another location to within some delta
   * @param loc2 The location to checked against
   * @param degDelta the delta for lat/lon (deg)
   * @param mDelta the delta in meters for altitude
   * @return True if the difference between both locations is within the deltas
   */
  public boolean almostEqual(Location loc2, double degDelta, double mDelta) {
    return Math.abs(latitude - loc2.getLatitude()) <= degDelta &&
      Math.abs(longitude - loc2.getLongitude()) <= degDelta &&
      Math.abs(altitude - loc2.getAltitude()) <= mDelta;
  }

  @Override public String toString() {
    return "Location{" + "latitude=" + latitude + ", longitude=" + longitude + ", altitude="
      + altitude + '}';
  }
}
