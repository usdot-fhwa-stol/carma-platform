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

import gov.dot.fhwa.saxton.carma.geometry.GeodesicCartesianConverter;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Point3D;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector;
import gov.dot.fhwa.saxton.carma.geometry.cartesian.Vector3D;
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
    Vector3D vec2StartPoint = new Vector3D(gCC.geodesic2Cartesian(seg.getLoc1(), Transform.identity()));
    Vector3D vec2EndPoint = new Vector3D(gCC.geodesic2Cartesian(seg.getLoc2(), Transform.identity()));
    Vector3D vec2ExternalPoint = new Vector3D(gCC.geodesic2Cartesian(loc, Transform.identity()));

    // Get vector from start to external point
    Vector3D startToExternalVec = Vector3D.fromVector(vec2ExternalPoint.subtract(vec2StartPoint));

    // Get vector from start to end point
    Vector3D startToEndVec = Vector3D.fromVector(vec2EndPoint.subtract(vec2StartPoint));

    // Calculate the sign of the crosstrack distance
    // Find the vector normal to the path and third point
    Vector3D normalVec = startToEndVec.cross(startToExternalVec);
    // This forms a plane along the the path segment.
    // With points A (end point) B(start point) and C (point along normal vector)
    // Point X is the external point
    // B' = B-A  :  C' = C-A   : X' = X-A
    // The sign of the 3x3 determinate of the matrix [B' C' X'] determines the sign of the crosstrack distance
    // Calculate B' C' X'
    Vector3D bPrime = startToEndVec;
    Vector3D cPrime = Vector3D.fromVector(normalVec.subtract(vec2EndPoint));
    Vector3D xPrime = Vector3D.fromVector(vec2ExternalPoint.subtract(vec2EndPoint));

    double determinantOfPlane = Vector3D.get3by3Determinant(bPrime, cPrime, xPrime);
    double sign = determinantOfPlane < 0 ? 1.0 : -1.0; // if det is less than 0 location is on the right.

    // Get angle between both vectors
    double interiorAngle = startToExternalVec.getAngleBetweenVectors(startToEndVec);

    return startToExternalVec.magnitude() * Math.sin(interiorAngle) * sign;
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
    double interiorAngle = startToExternalVec.getAngleBetweenVectors(startToEndVec);

    return startToExternalVec.magnitude() * Math.cos(interiorAngle);
  }

  @Override public Location projectOntoSegment(Location loc, GreatCircleSegment seg) {
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
    double interiorAngle = startToExternalVec.getAngleBetweenVectors(startToEndVec);
    
    // Calculate downtrack distance
    double downtrackDistance = startToExternalVec.magnitude() * Math.cos(interiorAngle);

    // Find unit vector along segment from start to projection
    Vector invertedSegUnitVector = startToEndVec.getUnitVector().scalarMultiply(-1.0);

    // Find vector from start to projection
    Vector startToProjection = invertedSegUnitVector.scalarMultiply(downtrackDistance);

    // Find vector from earth to projected point
    Vector vec2Projection = vec2StartPoint.subtract(startToProjection);

    // Fine location of projected point
    Point3D projectionInECEF = new Point3D(vec2Projection.getDim(0), vec2Projection.getDim(1), vec2Projection.getDim(2));
    return gCC.cartesian2Geodesic(projectionInECEF, Transform.identity());
  }
}
