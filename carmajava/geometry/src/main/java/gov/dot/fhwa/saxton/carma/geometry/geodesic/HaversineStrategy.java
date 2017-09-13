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

package gov.dot.fhwa.saxton.carma.geometry.geodesic;

/**
 * Implements a distance strategy which uses great circle distances and the haversine formula.
 * Based off of the public MIT licensed code at http://www.movable-type.co.uk/scripts/latlong.html
 */
public class HaversineStrategy implements IDistanceStrategy{
  protected final double R = 6371000; // Mean earth radius for WGS84 ellipsoid. Defined as R = (2Rea+Reb) / 3

  @Override public double distanceLoc2Loc(Location loc1, Location loc2) {
    double lat1 = loc1.getLatRad();
    double lat2 = loc2.getLatRad();
    double lon1 = loc1.getLonRad();
    double lon2 = loc2.getLonRad();

    double j = Math.pow(Math.sin((lat2 - lat1)/2.0), 2);
    double k = Math.pow(Math.sin((lon2 - lon1)/2.0), 2);

    return 2 * R * Math.asin(Math.sqrt( j + Math.cos(lat1) * Math.cos(lat2) * k));
  }

  @Override public double crossTrackDistance(Location loc, GreatCircleSegment seg) {
    double angularDistance = distanceLoc2Loc(seg.loc1, loc) / R;
    double brearingStartToLoc = getInitialBearing(seg.loc1, loc);
    double brearingStartToEnd = getInitialBearing(seg.loc1, seg.loc2);
    double deltaBearing = brearingStartToLoc - brearingStartToEnd;

    return Math.asin(Math.sin(angularDistance) * Math.sin(deltaBearing)) * R;
  }

  @Override public double downtrackDistance(Location loc, GreatCircleSegment seg) {
    double angularDistance = distanceLoc2Loc(seg.loc1, loc) / R;
    double angularCrossTrackDistance = crossTrackDistance(loc, seg) / R;

    return Math.acos(Math.cos(angularDistance) / Math.cos(angularCrossTrackDistance)) * R;
  }

  /**
   * Helper function gets the initial bearing of an object which will travel the great cricle segment from start to end locations
   * @param start The start of the segment to be traveled
   * @param end The end of the segment to be traveled
   * @return The bearing at the start point in rad
   */
  protected double getInitialBearing(Location start, Location end) {
    double deltaLon = end.getLonRad()-start.getLonRad();
    double y = Math.sin(deltaLon) * Math.cos(end.getLatRad());
    double x = Math.cos(start.getLatRad())*Math.sin(end.getLatRad()) -
      Math.sin(start.getLatRad())*Math.cos(end.getLatRad())*Math.cos(deltaLon);

    return Math.atan2(y, x);
  }
}
