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

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import org.ros.rosjava_geometry.Transform;

/**
 * Implements a distance strategy which uses great circle distances and the haversine formula.
 * The cross track and downtrack distances are still calculated with flat earth assumptions
 * Based off of the public MIT licensed code at http://www.movable-type.co.uk/scripts/latlong.html
 */
public class HaversineStrategy implements IDistanceStrategy{
  protected final double R = 6371009; // Mean earth radius for WGS84 ellipsoid. Defined as R = (2Rea+Reb) / 3

  @Override public double distanceLoc2Loc(Location loc1, Location loc2) {
    double lat1 = loc1.getLatRad();
    double lat2 = loc2.getLatRad();
    double lon1 = loc1.getLonRad();
    double lon2 = loc2.getLonRad();

    double j = Math.pow(Math.sin((lat2 - lat1)/2.0), 2);
    double k = Math.pow(Math.sin((lon2 - lon1)/2.0), 2);

    double a = j + Math.cos(lat1) * Math.cos(lat2) * k;

    return 2 * R * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
    // Can also be implemented as 2 * R * Math.asin(Math.sqrt( j + Math.cos(lat1) * Math.cos(lat2) * k));
  }

  @Override public double crossTrackDistance(Location loc, GreatCircleSegment seg) {
    // Get vectors from earth center to path and external location
    GeodesicCartesianConverter gCC = new GeodesicCartesianConverter();
    Vector vec2StartPoint = new Vector(gCC.geodesic2Cartesian(seg.getLoc1(), Transform.identity()));
    Vector vec2EndPoint = new Vector(gCC.geodesic2Cartesian(seg.getLoc2(), Transform.identity()));
    Vector vec2ExternalPoint = new Vector(gCC.geodesic2Cartesian(loc, Transform.identity()));

    // Get vector from start to external point
    Vector startToExternalVec = vec2ExternalPoint.subtract(vec2StartPoint);

    // Get vector from start to end point
    Vector startToEndVec = vec2EndPoint.subtract(vec2StartPoint);

    // Get angle between both vectors
    double interiorAngle = getAngleBetweenVectors(startToExternalVec, startToEndVec);

    return startToExternalVec.magnitude() * Math.sin(interiorAngle);
  }

  @Override public double downtrackDistance(Location loc, GreatCircleSegment seg) {
    // Get vectors from earth center to path and external location
    GeodesicCartesianConverter gCC = new GeodesicCartesianConverter();
    Vector vec2StartPoint = new Vector(gCC.geodesic2Cartesian(seg.getLoc1(), Transform.identity()));
    Vector vec2EndPoint = new Vector(gCC.geodesic2Cartesian(seg.getLoc2(), Transform.identity()));
    Vector vec2ExternalPoint = new Vector(gCC.geodesic2Cartesian(loc, Transform.identity()));

    // Get vector from start to external point
    Vector startToExternalVec = vec2ExternalPoint.subtract(vec2StartPoint);

    // Get vector from start to end point
    Vector startToEndVec = vec2EndPoint.subtract(vec2StartPoint);

    // Get angle between both vectors
    double interiorAngle = getAngleBetweenVectors(startToExternalVec, startToEndVec);

    return startToExternalVec.magnitude() * Math.cos(interiorAngle);
  }

  /**
   * Helper function calculates the angle between two vectors.
   * @param vec1 the first vector
   * @param vec2 the second vector
   * @return The angle in rad between the two vectors
   */
  protected double getAngleBetweenVectors(Vector vec1, Vector vec2) {
    double vec1Mag = vec1.magnitude();
    double vec2Mag = vec2.magnitude();
    if (vec1Mag == 0 || vec2Mag == 0) {
      return 0;
    }
    return  Math.acos(vec1.dot(vec2) / (vec1.magnitude() * vec2.magnitude()));
  }
}
