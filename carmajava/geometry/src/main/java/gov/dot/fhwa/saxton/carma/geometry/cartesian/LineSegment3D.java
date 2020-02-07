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

package gov.dot.fhwa.saxton.carma.geometry.cartesian;

/**
 * A representation of a line segment in 3-dimensional space.
 */
public class LineSegment3D extends LineSegment {

  /**
   * Constructor defines a line segment as the straight path between two provided points
   * @param p1 The first point
   * @param p2 The second point. Must be of the same dimension as the first point
   * @throws IllegalArgumentException Thrown if p1 and p2 are not the same dimension
   */
  public LineSegment3D(Point3D p1, Point3D p2) throws IllegalArgumentException{
    super(p1,p2);
  }

  /**
   * Constructor defines this line as a deep copy of the provided line segment
   * @param seg The segment to be copied
   */
  public LineSegment3D(LineSegment3D seg) {
    super(seg);
  }

  /**
   * Calculates the cross-track distance between a line and a point in 3D space.
   * This is 3D specific because it depends on the 3x3 determinant to sign the distance.
   *
   * @param loc The point
   * @return The distance in meters
   */
  public double crossTrackDistance(Point3D loc) {
    // Get vectors from earth center to path and external location
    Vector3D vec2StartPoint = new Vector3D((Point3D) p1);
    Vector3D vec2EndPoint = new Vector3D((Point3D) p2);
    Vector3D vec2ExternalPoint = new Vector3D((Point3D) loc);

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
    // The sign of the 3x3 determinant of the matrix [B' C' X'] determines the sign of the crosstrack distance
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

  /**
   * Calculates the downtrack distance along a segment of a provided point
   *
   * @param loc The point whose downtrack distance is being calculated
   * @return The distance in meters
   */
  public double downtrackDistance(Point3D loc) {
    // Get vectors from earth center to path and external location
    Vector vec2StartPoint = new Vector3D((Point3D) p1);
    Vector vec2EndPoint = new Vector3D((Point3D) p2);
    Vector vec2ExternalPoint = new Vector3D(loc);

    // Get vector from start to external point
    Vector startToExternalVec = vec2ExternalPoint.subtract(vec2StartPoint);

    // Get vector from start to end point
    Vector startToEndVec = vec2EndPoint.subtract(vec2StartPoint);

    // Get angle between both vectors
    double interiorAngle = startToExternalVec.getAngleBetweenVectors(startToEndVec);

    return startToExternalVec.magnitude() * Math.cos(interiorAngle);
  }

  /**
   * Calculates location of a external 3D point projected onto a line segment
   *
   * @param loc The point whose projection is being calculated
   * @return The projected location
   */
  public Point3D projectOntoSegment(Point3D loc) {
    return (Point3D) pointProjectedOnLine(loc);
  }
}
